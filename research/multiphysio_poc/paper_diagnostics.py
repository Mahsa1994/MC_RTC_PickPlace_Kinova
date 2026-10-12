"""
Which evaluation choices could produce the paper's Table I-II physio numbers?

reproduce_paper.py follows the paper's stated protocol (LOSO, labels min-max
normalized within subject). On the released data its physio models don't beat
predicting the training mean, while the paper reports RMSE ~0.20. The paper's
code isn't published and its dataset version (Zenodo 17225571, "v1") was
deleted in Dec 2025. So this script re-runs the physio rows of that protocol
under the alternative choices most likely to explain the gap:

  split     loso         : leave-one-subject-out (paper's stated protocol)
            kfold-trial  : 5-fold, whole trials held out, but every subject is in
                           both train and test (subject leakage only)
            kfold-window : 5-fold over individual windows, shuffled - the
                           kfold_validation() helper in the authors' repo
                           (src/utils/validation_utils.py). Windows of one trial,
                           sharing its label, land in both train and test.
  labels    subject      : min-max within subject (paper's statement)
            globalmax    : score / global max (the paper's Table I header gives
                           each score's mu and max)

Classification classes are per subject (mu +/- sd/2) in every case, so the label
normalization only changes regression. Metric per held-out group (subject for
loso, fold for k-fold), mean +/- std, same as reproduce_paper.py.
"""
import os

os.environ.setdefault("OMP_NUM_THREADS", "1")

import argparse
import multiprocessing as mp
from pathlib import Path

import numpy as np
from sklearn.model_selection import GroupKFold, KFold

import reproduce_paper as rp

D = {}


def groups_for(split, d):
    if split == "loso":
        return d["ids"]
    if split == "kfold-trial":
        g = np.zeros(len(d["trial"]), dtype=int)
        for f, (_, te) in enumerate(GroupKFold(n_splits=5, shuffle=True, random_state=0)
                                    .split(d["trial"], groups=d["trial"])):
            g[te] = f
        return g
    g = np.zeros(len(d["trial"]), dtype=int)
    for f, (_, te) in enumerate(KFold(n_splits=5, shuffle=True, random_state=0).split(d["trial"])):
        g[te] = f
    return g


def run_job(job):
    split, norm, target, kind, model, group = job
    d, lab = D["physio"], D["lab"]
    if kind == "reg":
        col = f"{target}_norm" if norm == "subject" else f"{target}_gmax"
    else:
        col = f"{target}_cls"
    y = lab[col].to_numpy()[d["trial"]]
    te = D["groups"][split] == group
    if model == "mean":
        pred = (np.full(te.sum(), y[~te].mean(), dtype=np.float32) if kind == "reg"
                else np.eye(3, dtype=np.float32)[np.full(te.sum(), np.bincount(y[~te], minlength=3).argmax())])
    else:
        pred = rp.fit_predict(model, kind, d["X"][~te], y[~te], d["X"][te])
    return job, rp.subject_score(kind, y[te], pred)[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", default="data")
    ap.add_argument("--workers", type=int, default=4)
    args = ap.parse_args()

    feat_dir = Path(args.data_dir) / "raw" / "features"
    lab = rp.load_labels(feat_dir)
    for t in rp.TARGETS:
        lab[f"{t}_gmax"] = lab[t] / lab[t].max()
    D["lab"] = lab
    D["physio"] = rp.load_modality(feat_dir, "physio", lab)
    D["groups"] = {s: groups_for(s, D["physio"]) for s in ("loso", "kfold-trial", "kfold-window")}

    conditions = [("kfold-window", "subject"), ("kfold-window", "globalmax"),
                  ("kfold-trial", "subject"), ("kfold-trial", "globalmax"), ("loso", "globalmax")]
    jobs = []
    for split, norm in conditions:
        for target in rp.TARGETS:
            kinds = ("reg", "cls") if norm == "subject" else ("reg",)
            for kind in kinds:
                for model in ("mean",) + rp.MODELS:
                    for g in np.unique(D["groups"][split]):
                        jobs.append((split, norm, target, kind, model, g))
    jobs.sort(key=lambda j: -(10 if j[4] == "RF" and j[3] == "reg" else 1))
    print(f"{len(jobs)} physio fits on {args.workers} workers", flush=True)
    with mp.get_context("fork").Pool(args.workers) as pool:
        res = dict(pool.imap_unordered(run_job, jobs, chunksize=4))

    for target in rp.TARGETS:
        for kind in ("reg", "cls"):
            print(f"\n{target} {'RMSE' if kind == 'reg' else 'macro-F1'} on physio windows "
                  f"(paper: {', '.join(f'{m} {v}' for m, v in zip(rp.MODELS, rp.PAPER[(target, kind)]['physio']))})")
            print(f"  {'split / labels':30s}" + "".join(f"{m:>18s}" for m in ("mean",) + rp.MODELS))
            for split, norm in conditions:
                if kind == "cls" and norm != "subject":
                    continue
                cells = []
                for model in ("mean",) + rp.MODELS:
                    v = np.array([s for (sp, nm, tg, kd, m, g), s in res.items()
                                  if (sp, nm, tg, kd, m) == (split, norm, target, kind, model)])
                    cells.append(f"{v.mean():.3f} +/- {v.std():.3f}")
                print(f"  {split + ' / ' + norm:30s}" + "".join(f"{c:>18s}" for c in cells))


if __name__ == "__main__":
    main()
