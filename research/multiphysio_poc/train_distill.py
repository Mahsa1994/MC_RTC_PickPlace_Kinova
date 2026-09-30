"""
Teacher/student distillation proof of concept on MultiPhysio-HRC.

Teacher: contact-based physiological features (HRV/EDA from bio_features_60s
+ EEG band power from eeg_features_5s) - "privileged", training-time only,
matches what the FRQNT proposal's HR+GSR sensors give.
Student: facial Action Unit features only - fully contactless, matches the
real-time-deployable modality for the Kinova pick-and-place task.
Target: SAM Valence/Arousal/Dominance - the same PAD framework the proposal
already uses for ground truth.

Trains three models to show what distillation actually buys you:
  1. teacher            - upper bound, uses privileged features
  2. student_no_distill - lower bound, AUs only, label loss only
  3. student_distill    - AUs only, label loss + matches the teacher's prediction

Run `python prepare_dataset.py` first to build data/merged_trials.csv.
"""
import argparse
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn
from sklearn.dummy import DummyClassifier
from sklearn.linear_model import LogisticRegression, RidgeCV
from sklearn.metrics import f1_score

from dataset import TARGET_COLS, load_folds, load_splits


def mlp(in_dim, out_dim=3, hidden=16, dropout=0.3):
    return nn.Sequential(
        nn.Linear(in_dim, hidden), nn.ReLU(), nn.Dropout(dropout),
        nn.Linear(hidden, out_dim),
    )


def ridge_baseline(X_tr, y_tr, X_va, y_va):
    """Cross-validated linear ridge, per target - a sane, hard-to-overfit
    reference point for whether the MLP's extra capacity is buying anything
    (README item 2: 91 features / 546 trials is a lot of dimensionality for
    an unregularized 2-layer MLP)."""
    alphas = np.logspace(-2, 4, 25)
    preds = np.zeros_like(y_va)
    for j in range(y_tr.shape[1]):
        model = RidgeCV(alphas=alphas).fit(X_tr, y_tr[:, j])
        preds[:, j] = model.predict(X_va)
    mae = np.mean(np.abs(preds - y_va), axis=0)
    ss_res = np.sum((y_va - preds) ** 2, axis=0)
    ss_tot = np.sum((y_va - y_va.mean(axis=0)) ** 2, axis=0)
    r2 = 1 - ss_res / np.clip(ss_tot, 1e-9, None)
    return mae, r2


def train_model(model, X_tr, y_tr, X_va, y_va, extra_targets_tr=None, kd_weight=0.0,
                 epochs=300, lr=1e-3, weight_decay=1e-2):
    opt = torch.optim.Adam(model.parameters(), lr=lr, weight_decay=weight_decay)
    X_tr_t, y_tr_t = torch.tensor(X_tr), torch.tensor(y_tr)
    X_va_t, y_va_t = torch.tensor(X_va), torch.tensor(y_va)
    extra_t = torch.tensor(extra_targets_tr) if extra_targets_tr is not None else None

    best_val, best_state = float("inf"), None
    for _ in range(epochs):
        model.train()
        opt.zero_grad()
        pred = model(X_tr_t)
        loss = nn.functional.mse_loss(pred, y_tr_t)
        if extra_t is not None and kd_weight > 0:
            loss = loss + kd_weight * nn.functional.mse_loss(pred, extra_t)
        loss.backward()
        opt.step()

        model.eval()
        with torch.no_grad():
            val_loss = nn.functional.mse_loss(model(X_va_t), y_va_t).item()
        if val_loss < best_val:
            best_val = val_loss
            best_state = {k: v.clone() for k, v in model.state_dict().items()}

    model.load_state_dict(best_state)
    return model, best_val


def evaluate(model, X, y):
    model.eval()
    with torch.no_grad():
        pred = model(torch.tensor(X)).numpy()
    mae = np.mean(np.abs(pred - y), axis=0)
    ss_res = np.sum((y - pred) ** 2, axis=0)
    ss_tot = np.sum((y - y.mean(axis=0)) ** 2, axis=0)
    r2 = 1 - ss_res / np.clip(ss_tot, 1e-9, None)
    return mae, r2


def run_fold(train, val, meta, args):
    """Train all 5 models (teacher MLP/ridge, student MLP/ridge, student+KD)
    on one train/val split and return {label: (mae, r2)}."""
    teacher = mlp(meta["teacher_dim"], hidden=args.hidden)
    teacher, _ = train_model(teacher, train.teacher_X, train.y, val.teacher_X, val.y,
                              epochs=args.epochs, weight_decay=args.weight_decay)
    teacher_mae, teacher_r2 = evaluate(teacher, val.teacher_X, val.y)
    teacher_ridge_mae, teacher_ridge_r2 = ridge_baseline(train.teacher_X, train.y, val.teacher_X, val.y)

    with torch.no_grad():
        teacher.eval()
        teacher_train_pred = teacher(torch.tensor(train.teacher_X)).numpy()

    student_plain = mlp(meta["student_dim"], hidden=args.hidden)
    student_plain, _ = train_model(student_plain, train.student_X, train.y, val.student_X, val.y,
                                    epochs=args.epochs, weight_decay=args.weight_decay)
    plain_mae, plain_r2 = evaluate(student_plain, val.student_X, val.y)
    student_ridge_mae, student_ridge_r2 = ridge_baseline(train.student_X, train.y, val.student_X, val.y)

    student_kd = mlp(meta["student_dim"], hidden=args.hidden)
    student_kd, _ = train_model(student_kd, train.student_X, train.y, val.student_X, val.y,
                                 extra_targets_tr=teacher_train_pred, kd_weight=args.kd_weight,
                                 epochs=args.epochs, weight_decay=args.weight_decay)
    kd_mae, kd_r2 = evaluate(student_kd, val.student_X, val.y)

    return {
        "teacher, MLP": (teacher_mae, teacher_r2),
        "teacher, ridge": (teacher_ridge_mae, teacher_ridge_r2),
        "student, no KD, MLP": (plain_mae, plain_r2),
        "student, no KD, ridge": (student_ridge_mae, student_ridge_r2),
        "student, with KD, MLP": (kd_mae, kd_r2),
    }


def print_table(title, rows, names, fmt="{:12.3f}"):
    print(f"\n=== {title} ===")
    print(f"{'':24s}" + "".join(f"{n:>12s}" for n in names))
    for label, vals in rows:
        print(f"{label:24s}" + "".join(fmt.format(v) for v in vals))


def print_cv_table(title, rows, names):
    """rows: [(label, mean[3], std[3])] -> "mean±std" per cell."""
    print(f"\n=== {title} ===")
    print(f"{'':24s}" + "".join(f"{n:>16s}" for n in names))
    for label, mean, std in rows:
        print(f"{label:24s}" + "".join(f"{m:7.3f}+/-{s:<6.3f}" for m, s in zip(mean, std)))


def tertile_bins(y_tr_cont, y_va_cont):
    """Cut a continuous target into train-quantile tertiles (low/mid/high).
    Cut points come from TRAIN only, no leakage - shared by classify_folds()
    and pca_analysis.py."""
    q1, q2 = np.quantile(y_tr_cont, [1 / 3, 2 / 3])
    return np.digitize(y_tr_cont, [q1, q2]), np.digitize(y_va_cont, [q1, q2])


def tertile_f1(X_tr, y_tr, X_va, y_va):
    clf = LogisticRegression(max_iter=2000).fit(X_tr, y_tr)
    return f1_score(y_va, clf.predict(X_va), average="macro")


def classify_folds(merged_path, n_splits, subject_center):
    """Coarse low/mid/high (tertile) classification instead of exact
    regression, per README item 3 - self-reported SAM scores on a small
    ordinal scale are noisy, so a coarser target may show cleaner signal
    than exact regression even when R^2 is near zero."""
    results = {t: {"teacher": [], "student": [], "majority-class": []} for t in TARGET_COLS}
    for train, val, meta in load_folds(merged_path, n_splits=n_splits, subject_center=subject_center):
        for ti, t in enumerate(TARGET_COLS):
            y_tr, y_va = tertile_bins(train.y[:, ti], val.y[:, ti])

            for name, X_tr, X_va in [("teacher", train.teacher_X, val.teacher_X),
                                      ("student", train.student_X, val.student_X)]:
                results[t][name].append(tertile_f1(X_tr, y_tr, X_va, y_va))

            dummy = DummyClassifier(strategy="most_frequent").fit(train.student_X, y_tr)
            results[t]["majority-class"].append(f1_score(y_va, dummy.predict(val.student_X), average="macro"))
    return results


def print_classify_report(results, n_splits, subject_center):
    print(f"\n=== Tertile classification macro-F1 across {n_splits} folds, mean+/-std "
          f"(subject_center={subject_center}) ===")
    for t in TARGET_COLS:
        print(f"{t}:")
        for name in ("teacher", "student", "majority-class"):
            vals = results[t][name]
            print(f"  {name:15s} {np.mean(vals):.3f} +/- {np.std(vals):.3f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", default="data")
    ap.add_argument("--epochs", type=int, default=300)
    ap.add_argument("--kd-weight", type=float, default=1.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--hidden", type=int, default=32)
    ap.add_argument("--weight-decay", type=float, default=1e-1)
    ap.add_argument("--no-subject-center", action="store_true",
                     help="disable per-participant feature centering (on by default)")
    ap.add_argument("--cv", type=int, default=0,
                     help="run subject-wise GroupKFold with this many folds and report "
                          "mean+/-std across folds, instead of a single train/val split "
                          "(much less noisy on this small a dataset - see README)")
    ap.add_argument("--classify", action="store_true",
                     help="also report tertile (low/mid/high) classification macro-F1 "
                          "via GroupKFold (5 folds, or --cv if set), see README item 3")
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    merged_path = Path(args.data_dir) / "merged_trials.csv"
    names = ["Valence", "Arousal", "Dominance"]

    if args.cv:
        fold_results = []
        for fold_i, (train, val, meta) in enumerate(
                load_folds(merged_path, n_splits=args.cv, subject_center=not args.no_subject_center)):
            print(f"[fold {fold_i}] train={train.teacher_X.shape[0]} val={val.teacher_X.shape[0]} "
                  f"({len(set(train.ids))} vs {len(set(val.ids))} participants)")
            fold_results.append(run_fold(train, val, meta, args))

        labels = list(fold_results[0].keys())
        mae_rows, r2_rows = [], []
        for label in labels:
            maes = np.stack([f[label][0] for f in fold_results])
            r2s = np.stack([f[label][1] for f in fold_results])
            mae_rows.append((label, maes.mean(axis=0), maes.std(axis=0)))
            r2_rows.append((label, r2s.mean(axis=0), r2s.std(axis=0)))
        print_cv_table(f"Validation MAE across {args.cv} folds, mean+/-std "
                       f"(subject_center={not args.no_subject_center})", mae_rows, names)
        print_cv_table(f"Validation R^2 across {args.cv} folds, mean+/-std "
                       f"(subject_center={not args.no_subject_center})", r2_rows, names)
        if args.classify:
            results = classify_folds(merged_path, args.cv, not args.no_subject_center)
            print_classify_report(results, args.cv, not args.no_subject_center)
        return

    train, val, meta = load_splits(merged_path, seed=args.seed, subject_center=not args.no_subject_center)
    print(f"train: {train.teacher_X.shape[0]} trials, val: {val.teacher_X.shape[0]} trials "
          f"(subject-wise split, {len(set(train.ids))} vs {len(set(val.ids))} participants)")
    print(f"teacher_dim={meta['teacher_dim']} (bio+eeg), student_dim={meta['student_dim']} (AUs), "
          f"subject_center={not args.no_subject_center}")

    results = run_fold(train, val, meta, args)
    mae_rows = [(label, results[label][0]) for label in results]
    r2_rows = [(label, results[label][1]) for label in results]
    print_table("Validation MAE (standardized units, lower is better)", mae_rows, names)
    print_table("Validation R^2 (higher is better)", r2_rows, names)

    if args.classify:
        results = classify_folds(merged_path, 5, not args.no_subject_center)
        print_classify_report(results, 5, not args.no_subject_center)


if __name__ == "__main__":
    main()
