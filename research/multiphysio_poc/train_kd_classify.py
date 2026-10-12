"""
Teacher/student distillation as 3-class (low/mid/high) classification, following
the 4-step plan from the 2026-10 supervisor review (see README "KD
classification pass"):

  Step 1 - visual-only baseline: the student MLP on facial AUs alone, no teacher.
           Answers "are the AUs discriminative on their own?" and gives the bar
           any distilled student has to clear.
  Step 2 - stronger teacher (highest priority): sweep wider/deeper MLPs over
           several privileged input sets (bio+EEG, with/without subject
           centering, with/without AUs), plus logistic-regression and
           gradient-boosting references. The teacher used for distillation is
           picked per fold by its OUT-OF-FOLD macro-F1 on the outer-train set,
           so the outer-test fold never influences the choice.
  Step 3 - student architecture: Linear(AU->64)-ReLU-Linear(64->32)-ReLU-
           Linear(32->3) with softmax on top, against train_distill.py's old
           single 32-unit hidden layer.
  Step 4 - distillation penalty sweep: loss = (CE + lambda * T^2 * KL) / (1 + lambda)
           for increasing lambda, up to lambda=inf (pure distillation, no hard
           labels at all).

Evaluation protocol (deliberately different from train_distill.py):
  - Outer: the same subject-wise 5-fold GroupKFold as train_distill.py --cv 5.
  - Inner: each outer-train set is split again by participant (GroupKFold,
    --inner folds). Every model is trained once per inner split and
    early-stopped on that split's held-out participants. The K members'
    softmax outputs are averaged for the outer-test prediction. The outer-test
    fold is never used for early stopping or selection. (train_distill.py's
    train_model() early-stops on the same fold it reports, which flatters
    its MLP numbers.)
  - The same inner split gives every outer-train trial an out-of-fold (OOF)
    teacher prediction from a teacher that never saw that participant. Used
    to pick the teacher (Step 2) and as the "cross-fitted" soft labels for
    distillation (Step 4) - see soften() for why that matters. MLP teachers
    early-stop on a further participant-wise slice of their inner-train set
    (teacher_es_splits()), NOT on the inner held-out participants they
    produce OOF predictions for. Otherwise their OOF F1 is the maximum over
    epochs on that very set - inflated, which biased teacher selection
    toward MLPs over logreg/gboost, and leaked label information into the
    "oof" soft labels.
  - Repeated over --seeds (MLP init, dropout, inner split); per-fold numbers
    are averaged over seeds, then mean+/-std is taken across the 5 folds.

Run `python prepare_dataset.py` first to build data/merged_trials.csv.
"""
import os

# Many tiny models trained in parallel worker processes: one BLAS/OpenMP
# thread per process is much faster than letting every process spawn 20.
os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")

import argparse
import json
import multiprocessing as mp
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from sklearn.ensemble import HistGradientBoostingClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import accuracy_score, f1_score
from sklearn.model_selection import GroupKFold, GroupShuffleSplit
from sklearn.preprocessing import OneHotEncoder

from dataset import TARGET_COLS, load_folds
from train_distill import tertile_bins

N_CLASSES = 3
CLASS_NAMES = ("low", "mid", "high")
OLD_HIDDEN = (32,)  # train_distill.py's mlp(): a single 32-unit hidden layer

# Step 2 teacher capacity sweep. Dropout grows with width so the bigger
# models don't just memorize ~450 training trials; --teacher-input-noise
# adds Gaussian input noise on top (see fit_mlp()).
TEACHER_ARCHS = {
    "mlp32": dict(hidden=(32,), dropout=0.3),
    "mlp128-64": dict(hidden=(128, 64), dropout=0.3),
    "mlp256-128-64": dict(hidden=(256, 128, 64), dropout=0.4),
    "mlp512-256-128": dict(hidden=(512, 256, 128), dropout=0.5),
}
TEACHER_INPUTS = ("physio", "physio_raw", "physio+AU", "physio_raw+AU")
SMALL_TEACHER = "mlp32 on physio"  # closest to train_distill.py's teacher


def mlp_classifier(in_dim, hidden, dropout):
    """Linear-ReLU-Dropout blocks, then a linear layer to N_CLASSES logits.
    Softmax sits on top: F.cross_entropy applies log-softmax internally
    during training, and predict() applies softmax explicitly. Adding an
    nn.Softmax layer here as well would softmax twice and silently flatten
    the gradients."""
    layers, d = [], in_dim
    for h in hidden:
        layers += [nn.Linear(d, h), nn.ReLU(), nn.Dropout(dropout)]
        d = h
    layers.append(nn.Linear(d, N_CLASSES))
    return nn.Sequential(*layers)


def fit_mlp(X_tr, y_tr, X_es, y_es, hidden, dropout, seed, args, soft_tr=None, kd_lambda=0.0,
            input_noise=0.0):
    """Full-batch AdamW with early stopping on the held-out (X_es, y_es)
    participants against the TRUE labels, for every model including the
    distilled students. --es-metric picks the criterion: "ce" (held-out
    cross-entropy) or "f1" (held-out macro-F1). On this dataset held-out CE
    bottoms out after only 10-50 epochs and then climbs as the model gets
    overconfident, sometimes while held-out F1 is still improving.
    input_noise adds N(0, input_noise^2) to the (standardized) inputs at
    every training step - a regularizer that slows memorization, which
    matters most for the wide Step 2 teachers. With soft_tr, the loss is
    (CE + lambda * T^2 * KL(soft_tr || student_T)) / (1 + lambda). Dividing by
    (1 + lambda) keeps the overall loss scale, and so the effective
    learning-rate/weight-decay balance, the same across the lambda sweep.
    lambda=inf means pure distillation."""
    torch.manual_seed(seed)
    model = mlp_classifier(X_tr.shape[1], hidden, dropout)
    opt = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=args.weight_decay)
    X_tr_t, y_tr_t = torch.from_numpy(X_tr), torch.from_numpy(y_tr).long()
    X_es_t, y_es_t = torch.from_numpy(X_es), torch.from_numpy(y_es).long()
    soft_t = torch.from_numpy(soft_tr).float() if soft_tr is not None else None
    alpha = 1.0 if np.isinf(kd_lambda) else kd_lambda / (1.0 + kd_lambda)
    T = args.temperature

    best, best_state, since_best = float("inf"), None, 0
    for _ in range(args.max_epochs):
        model.train()
        opt.zero_grad()
        X_in = X_tr_t + input_noise * torch.randn_like(X_tr_t) if input_noise > 0 else X_tr_t
        logits = model(X_in)
        loss = F.cross_entropy(logits, y_tr_t)
        if soft_t is not None and alpha > 0:
            kd = F.kl_div(F.log_softmax(logits / T, dim=1), soft_t, reduction="batchmean") * T * T
            loss = (1 - alpha) * loss + alpha * kd
        loss.backward()
        opt.step()

        model.eval()
        with torch.no_grad():
            es_logits = model(X_es_t)
        if args.es_metric == "ce":
            es_loss = F.cross_entropy(es_logits, y_es_t).item()
        else:
            es_loss = -f1_score(y_es, es_logits.argmax(dim=1).numpy(), average="macro")
        if es_loss < best - 1e-4:
            best, since_best = es_loss, 0
            best_state = {k: v.clone() for k, v in model.state_dict().items()}
        else:
            since_best += 1
            if since_best >= args.patience:
                break

    model.load_state_dict(best_state)
    model.eval()

    def predict(X):
        with torch.no_grad():
            return F.softmax(model(torch.from_numpy(X)), dim=1).numpy()
    return predict


def _sklearn_predict(model):
    """predict_proba with all N_CLASSES columns, even if an inner-train
    split happened to miss a class."""
    def predict(X):
        out = np.zeros((len(X), N_CLASSES), dtype=np.float32)
        out[:, model.classes_] = model.predict_proba(X)
        return out
    return predict


def fit_logreg(X_tr, y_tr, C=1.0):
    return _sklearn_predict(LogisticRegression(C=C, max_iter=5000).fit(X_tr, y_tr))


def fit_hgb(X_tr, y_tr, seed):
    return _sklearn_predict(HistGradientBoostingClassifier(
        max_iter=200, learning_rate=0.05, max_depth=3, random_state=seed).fit(X_tr, y_tr))


def cv_ensemble(fit, X_tr, X_te, inner):
    """One member per inner split: fit(k, inner_train_idx, inner_heldout_idx)
    returns a predict_proba function. Returns
      oof     [n_tr, C]: each outer-train trial predicted by the member that
                         did NOT train on its participant,
      test    [n_te, C]: outer-test prediction averaged over members,
      members [K][n_tr, C]: each member's prediction on the whole outer-train
                         set (in-sample on its own inner-train rows)."""
    oof = np.zeros((len(X_tr), N_CLASSES), dtype=np.float32)
    test = np.zeros((len(X_te), N_CLASSES), dtype=np.float32)
    members = []
    for k, (itr, ies) in enumerate(inner):
        predict = fit(k, itr, ies)
        oof[ies] = predict(X_tr[ies])
        test += predict(X_te) / len(inner)
        members.append(predict(X_tr))
    return oof, test, members


def soften(proba, T):
    """Temperature-softened teacher distribution, softmax(log(p) / T).

    Where the soft labels come from matters more than T here. A teacher
    asked to predict trials it was trained on ("insample") has largely
    memorized them, so its soft labels are close to the one-hot hard labels
    and distillation degenerates into label smoothing. "oof" soft labels
    come from a teacher that never saw that participant. They carry what
    the teacher actually generalizes, which is the knowledge distillation
    is meant to transfer."""
    logp = np.log(np.clip(proba, 1e-6, 1.0)) / T
    logp -= logp.max(axis=1, keepdims=True)
    p = np.exp(logp)
    return (p / p.sum(axis=1, keepdims=True)).astype(np.float32)


def scores(y_true, proba):
    pred = proba.argmax(axis=1)
    return dict(f1=f1_score(y_true, pred, average="macro"),
                acc=accuracy_score(y_true, pred),
                f1_per_class=f1_score(y_true, pred, average=None, labels=list(range(N_CLASSES))).tolist())


def teacher_inputs(tr_c, va_c, tr_r, va_r):
    """tr_c/va_c: subject-centered fold, tr_r/va_r: the same fold without
    centering. The teacher is training-time only, so it may use different
    preprocessing than the student. Uncentered ("raw") physio keeps each
    person's absolute HRV/EDA/EEG level, which centering removes along with
    ~1/3 of the label variance that sits between participants. The student
    always gets centered AUs."""
    return {
        "physio": (tr_c.teacher_X, va_c.teacher_X),
        "physio_raw": (tr_r.teacher_X, va_r.teacher_X),
        "physio+AU": (np.hstack([tr_c.teacher_X, tr_c.student_X]),
                      np.hstack([va_c.teacher_X, va_c.student_X])),
        "physio_raw+AU": (np.hstack([tr_r.teacher_X, tr_c.student_X]),
                          np.hstack([va_r.teacher_X, va_c.student_X])),
    }


def teacher_es_splits(inner, ids, seed, es_frac=0.2):
    """Per inner split k: (fit_idx, es_idx), both inside inner-train_k,
    split by participant. MLP teachers train on fit_idx and early-stop on
    es_idx, keeping inner_heldout_k clean for OOF predictions (see module
    docstring)."""
    out = []
    for k, (itr, _) in enumerate(inner):
        gss = GroupShuffleSplit(n_splits=1, test_size=es_frac, random_state=seed * 100 + k)
        fit_pos, es_pos = next(gss.split(itr, groups=ids[itr]))
        out.append((itr[fit_pos], itr[es_pos]))
    return out


def run_job(job):
    """All models for one (target, outer fold, seed). Returns
    (key, {model label: outer-test scores}, info)."""
    target_i, fold_i, seed, (tr_c, va_c, tr_r, va_r), args = job
    torch.set_num_threads(1)
    np.random.seed(seed)

    y_tr, y_te = tertile_bins(tr_c.y[:, target_i], va_c.y[:, target_i])
    inner = list(GroupKFold(n_splits=args.inner, shuffle=True, random_state=seed)
                 .split(tr_c.student_X, groups=tr_c.ids))
    es_splits = teacher_es_splits(inner, tr_c.ids, seed)
    S_tr, S_te = tr_c.student_X, va_c.student_X
    out, info = {}, {"train_class_counts": np.bincount(y_tr, minlength=N_CLASSES).tolist()}

    # --- reference baselines (not MLPs, no inner CV needed) ---
    majority = np.zeros((len(y_te), N_CLASSES), dtype=np.float32)
    majority[:, np.bincount(y_tr).argmax()] = 1.0
    out["ref: majority class"] = scores(y_te, majority)
    enc = OneHotEncoder(handle_unknown="ignore", sparse_output=False).fit(tr_c.task.reshape(-1, 1))
    out["ref: task identity only (LR)"] = scores(
        y_te, fit_logreg(enc.transform(tr_c.task.reshape(-1, 1)), y_tr)(enc.transform(va_c.task.reshape(-1, 1))))
    out["ref: student LR on AUs"] = scores(y_te, fit_logreg(S_tr, y_tr)(S_te))

    # --- Step 2: teacher sweep ---
    teachers = {}
    inputs = teacher_inputs(tr_c, va_c, tr_r, va_r)
    for inp in args.teacher_inputs:
        T_tr, T_te = inputs[inp]
        cands = {f"{arch} on {inp}": (lambda k, itr, ies, spec=TEACHER_ARCHS[arch]: fit_mlp(
                     T_tr[es_splits[k][0]], y_tr[es_splits[k][0]], T_tr[es_splits[k][1]], y_tr[es_splits[k][1]],
                     spec["hidden"], spec["dropout"], seed * 100 + k, args,
                     input_noise=args.teacher_input_noise))
                 for arch in args.teacher_archs}
        cands[f"logreg on {inp}"] = lambda k, itr, ies: fit_logreg(T_tr[itr], y_tr[itr], C=0.1)
        if not args.no_hgb:
            cands[f"gboost on {inp}"] = lambda k, itr, ies: fit_hgb(T_tr[itr], y_tr[itr], seed * 100 + k)
        for name, fit in cands.items():
            oof, test, members = cv_ensemble(fit, T_tr, T_te, inner)
            teachers[name] = dict(oof=oof, members=members)
            out[f"teacher: {name}"] = scores(y_te, test)
            info[f"teacher: {name}"] = dict(
                oof_f1=f1_score(y_tr, oof.argmax(axis=1), average="macro"),
                insample_acc=float(np.mean([accuracy_score(y_tr[itr], members[k][itr].argmax(axis=1))
                                            for k, (itr, _) in enumerate(inner)])))

    best_teacher = max(teachers, key=lambda n: info[f"teacher: {n}"]["oof_f1"])
    info["best_teacher"] = best_teacher

    # --- Steps 1, 3, 4: students on AUs only ---
    def student(hidden, soft=None, kd_lambda=0.0):
        def fit(k, itr, ies):
            return fit_mlp(S_tr[itr], y_tr[itr], S_tr[ies], y_tr[ies], hidden, args.student_dropout,
                           seed * 100 + 50 + k, args, input_noise=args.student_input_noise,
                           soft_tr=None if soft is None else soft(k, itr), kd_lambda=kd_lambda)
        _, test, _ = cv_ensemble(fit, S_tr, S_te, inner)
        return scores(y_te, test)

    out["student: MLP 32 (old arch), no KD"] = student(OLD_HIDDEN)
    out[f"student: MLP {'-'.join(map(str, args.student_hidden))}, no KD"] = student(args.student_hidden)

    kd_teachers = {"best teacher": best_teacher}
    if SMALL_TEACHER in teachers:
        kd_teachers["small teacher"] = SMALL_TEACHER
    for name in args.extra_kd_teachers:
        if name not in teachers:
            raise ValueError(f"--extra-kd-teachers {name!r} not in this run's sweep: {sorted(teachers)}")
        kd_teachers[name] = name
    for role, tname in kd_teachers.items():
        t = teachers[tname]
        soft_sources = {
            "insample": lambda k, itr, t=t: soften(t["members"][k][itr], args.temperature),
            "oof": lambda k, itr, t=t: soften(t["oof"][itr], args.temperature),
        }
        for mode in args.soft_modes:
            for lam in args.kd_lambdas:
                out[f"KD: {role}, {mode} soft labels, lambda={lam:g}"] = student(
                    args.student_hidden, soft_sources[mode], lam)
    return (TARGET_COLS[target_i], fold_i, seed), out, info


# ------------------------------------------------------------------ reporting

def aggregate(job_results, target, n_folds):
    """{label: (per-fold seed-averaged F1 [n_folds], per-fold acc [n_folds], per-class F1 [3])}."""
    per = {}
    for (t, fold_i, _), out, _ in job_results:
        if t != target:
            continue
        for label, s in out.items():
            d = per.setdefault(label, {"f1": [[] for _ in range(n_folds)], "acc": [[] for _ in range(n_folds)],
                                       "pc": []})
            d["f1"][fold_i].append(s["f1"])
            d["acc"][fold_i].append(s["acc"])
            d["pc"].append(s["f1_per_class"])
    return {label: (np.array([np.mean(v) for v in d["f1"]]), np.array([np.mean(v) for v in d["acc"]]),
                    np.mean(d["pc"], axis=0)) for label, d in per.items()}


def print_rows(agg, labels, baseline=None, extra=None, delta_header="dF1 vs no-KD"):
    head = f"  {'':62s}{'macro-F1':>15s}{'accuracy':>15s}"
    if baseline:
        head += f"{delta_header:>17s}{'folds up':>9s}"
    if extra:
        head += extra[0]
    print(head)
    for label in labels:
        f1, acc, _ = agg[label]
        line = f"  {label:62s}{f1.mean():8.3f}+/-{f1.std():.3f}{acc.mean():8.3f}+/-{acc.std():.3f}"
        if baseline:
            d = f1 - agg[baseline][0]
            line += f"{d.mean():+10.3f}+/-{d.std():.3f}{int((d > 0).sum()):>5d}/{len(d)}"
        if extra:
            line += extra[1](label)
        print(line)


def report(job_results, target, args):
    n_folds = args.cv
    agg = aggregate(job_results, target, n_folds)
    infos = [info for (t, _, _), _, info in job_results if t == target]
    counts = {f: info["train_class_counts"] for (t, f, _), _, info in job_results if t == target}
    student_label = f"student: MLP {'-'.join(map(str, args.student_hidden))}, no KD"

    print(f"\n{'=' * 100}\n{target}: 3-class (low/mid/high) classification, subject-wise {n_folds}-fold CV x "
          f"{len(args.seeds)} seeds, every MLP = {args.inner}-member inner-CV ensemble")
    print(f"train class counts per fold (low/mid/high): {[counts[f] for f in sorted(counts)]}")

    print("\n-- Reference baselines --")
    print_rows(agg, [l for l in agg if l.startswith("ref:")])

    print("\n-- Step 1 + 3: visual-only students, no teacher --")
    print_rows(agg, ["student: MLP 32 (old arch), no KD", student_label])

    print("\n-- Step 2: teacher sweep (outer-test scores; selection uses OOF F1 on outer-train only) --")
    tlabels = sorted([l for l in agg if l.startswith("teacher:")], key=lambda l: -agg[l][0].mean())
    picks = [i["best_teacher"] for i in infos]

    def teacher_extra(label):
        key = label
        oof = np.mean([i[key]["oof_f1"] for i in infos])
        ins = np.mean([i[key]["insample_acc"] for i in infos])
        return f"{oof:10.3f}{ins:12.3f}{picks.count(label[len('teacher: '):]):>8d}/{len(picks)}"
    print_rows(agg, tlabels, extra=(f"{'OOF F1':>10s}{'train acc':>12s}{'picked':>11s}", teacher_extra))

    print("\n-- Step 4: distillation into the 2-layer AU student "
          f"(T={args.temperature:g}, loss = (CE + lambda*KD)/(1+lambda)) --")
    kd = [l for l in agg if l.startswith("KD:")]
    print_rows(agg, [student_label] + kd, baseline=student_label)

    best_kd = max(kd, key=lambda l: agg[l][0].mean()) if kd else None
    print("\n-- Per-class F1 (low / mid / high), averaged over folds and seeds --")
    for label in ["ref: student LR on AUs", student_label, best_kd]:
        if label:
            pc = agg[label][2]
            print(f"  {label:62s}" + "  ".join(f"{n}={v:.3f}" for n, v in zip(CLASS_NAMES, pc)))
    return agg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", default="data")
    ap.add_argument("--targets", nargs="+", default=["Arousal"], choices=TARGET_COLS)
    ap.add_argument("--cv", type=int, default=5, help="outer subject-wise GroupKFold folds")
    ap.add_argument("--inner", type=int, default=5,
                    help="inner subject-wise folds per outer-train set: early stopping, ensemble "
                         "members, OOF teacher predictions")
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2])
    ap.add_argument("--teacher-inputs", nargs="+", default=list(TEACHER_INPUTS), choices=TEACHER_INPUTS)
    ap.add_argument("--teacher-archs", nargs="+", default=list(TEACHER_ARCHS), choices=list(TEACHER_ARCHS))
    ap.add_argument("--no-hgb", action="store_true", help="skip the gradient-boosting teacher references")
    ap.add_argument("--student-hidden", type=int, nargs="+", default=[64, 32])
    ap.add_argument("--student-dropout", type=float, default=0.3)
    ap.add_argument("--extra-kd-teachers", nargs="*", default=[],
                    help='also distill from these fixed teachers, by sweep name, e.g. "gboost on physio_raw"')
    ap.add_argument("--kd-lambdas", type=float, nargs="+", default=[1, 3, 10, float("inf")],
                    help="distillation weight relative to CE; inf = pure distillation")
    ap.add_argument("--soft-modes", nargs="+", default=["insample", "oof"], choices=["insample", "oof"])
    ap.add_argument("--temperature", type=float, default=2.0)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--weight-decay", type=float, default=1e-2)
    ap.add_argument("--max-epochs", type=int, default=1000)
    ap.add_argument("--patience", type=int, default=100)
    ap.add_argument("--es-metric", choices=["ce", "f1"], default="f1",
                    help="early-stopping criterion on the inner held-out participants (README: f1 "
                         "lifts the best MLP teacher from ~0.35 to ~0.42 Arousal macro-F1 vs ce)")
    ap.add_argument("--teacher-input-noise", type=float, default=0.5,
                    help="std of Gaussian noise added to standardized teacher inputs during training "
                         "(README: 0.5 with --es-metric f1 gave the most stable teachers)")
    ap.add_argument("--student-input-noise", type=float, default=0.0)
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument("--out", default=None, help="optional path to dump all per-job scores as JSON")
    args = ap.parse_args()
    args.student_hidden = tuple(args.student_hidden)

    merged_path = Path(args.data_dir) / "merged_trials.csv"
    folds = []
    for (tr_c, va_c, _), (tr_r, va_r, _) in zip(load_folds(merged_path, args.cv, subject_center=True),
                                                load_folds(merged_path, args.cv, subject_center=False)):
        assert (tr_c.ids == tr_r.ids).all() and np.allclose(tr_c.y, tr_r.y), "centered/raw folds misaligned"
        folds.append((tr_c, va_c, tr_r, va_r))

    jobs = [(TARGET_COLS.index(t), fold_i, seed, fold, args)
            for t in args.targets for fold_i, fold in enumerate(folds) for seed in args.seeds]
    print(f"{len(jobs)} jobs ({len(args.targets)} targets x {args.cv} folds x {len(args.seeds)} seeds) "
          f"on {args.workers} workers")
    with mp.get_context("fork").Pool(args.workers) as pool:
        job_results = []
        for i, res in enumerate(pool.imap_unordered(run_job, jobs), 1):
            job_results.append(res)
            print(f"  done {i}/{len(jobs)}: {res[0]}", flush=True)

    for t in args.targets:
        report(job_results, t, args)

    if args.out:
        dump = [dict(target=t, fold=f, seed=s, scores=out, info=info) for (t, f, s), out, info in job_results]
        Path(args.out).write_text(json.dumps(dump, indent=1))
        print(f"\nwrote per-job scores -> {args.out}")


if __name__ == "__main__":
    main()
