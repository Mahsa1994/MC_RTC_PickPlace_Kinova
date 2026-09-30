"""
PCA sanity check for the teacher/student distillation PoC (README item
"Feature selection / dimensionality"): the RidgeCV baseline in
train_distill.py matches or beats the MLP most of the time, suggesting most
of the 91 teacher / 40 student features aren't pulling weight. This checks
two things:

1. How many components does it actually take to capture most of the
   variance - is a handful of components doing most of the work?
2. Do the top components load on features that make physiological sense
   (HRV/EDA/EEG-band or AU features grouped in a plausible way), or are they
   dominated by something that still looks like noise/artifact - i.e. were
   the "right" features kept by the earlier winsorization/centering fixes?
   Then checks whether reducing to those components preserves (or costs)
   the ridge-recoverable regression/classification signal via the same
   subject-wise CV harness used elsewhere in this PoC.

Run `python prepare_dataset.py` first to build data/merged_trials.csv.
"""
import argparse
from pathlib import Path

import numpy as np
from sklearn.decomposition import PCA
from sklearn.preprocessing import StandardScaler

from dataset import TARGET_COLS, _prepare, load_folds
from train_distill import ridge_baseline, tertile_bins, tertile_f1


def explained_variance_curve(X, cols, name, checkpoints=(0.80, 0.90, 0.95)):
    pca = PCA(n_components=min(X.shape)).fit(X)
    cum = np.cumsum(pca.explained_variance_ratio_)
    print(f"\n[{name}] {len(cols)} features, {X.shape[0]} trials -> components needed "
          f"for cumulative explained variance:")
    for c in checkpoints:
        k = int(np.searchsorted(cum, c) + 1)
        print(f"  {c:.0%}: {k} components")
    return pca


def print_top_loadings(pca, cols, name, n_components=3, top_k=8):
    print(f"\n[{name}] top |loading| features per component (sign = direction):")
    for i in range(min(n_components, pca.components_.shape[0])):
        comp = pca.components_[i]
        order = np.argsort(-np.abs(comp))[:top_k]
        print(f"  PC{i + 1} ({pca.explained_variance_ratio_[i]:.1%} of variance):")
        for j in order:
            print(f"    {cols[j]:30s} {comp[j]:+.3f}")


def cv_pca_performance(merged_path, n_splits, subject_center, var_threshold):
    """Same subject-wise GroupKFold harness as train_distill.py --cv, but
    comparing full-dimensional features against a PCA reduction fit on
    TRAIN only per fold (var_threshold chooses n_components, no leakage)."""
    reg = {"teacher": {"full": [], "pca": []}, "student": {"full": [], "pca": []}}
    clf = {t: {"teacher_full": [], "teacher_pca": [], "student_full": [], "student_pca": []}
           for t in TARGET_COLS}
    n_comps = {"teacher": [], "student": []}

    for train, val, meta in load_folds(merged_path, n_splits=n_splits, subject_center=subject_center):
        for name, X_tr, X_va in [("teacher", train.teacher_X, val.teacher_X),
                                  ("student", train.student_X, val.student_X)]:
            pca = PCA(n_components=var_threshold, svd_solver="full").fit(X_tr)
            X_tr_pca, X_va_pca = pca.transform(X_tr), pca.transform(X_va)
            n_comps[name].append(pca.n_components_)

            _, full_r2 = ridge_baseline(X_tr, train.y, X_va, val.y)
            _, pca_r2 = ridge_baseline(X_tr_pca, train.y, X_va_pca, val.y)
            reg[name]["full"].append(full_r2)
            reg[name]["pca"].append(pca_r2)

            for ti, t in enumerate(TARGET_COLS):
                y_tr, y_va = tertile_bins(train.y[:, ti], val.y[:, ti])
                clf[t][f"{name}_full"].append(tertile_f1(X_tr, y_tr, X_va, y_va))
                clf[t][f"{name}_pca"].append(tertile_f1(X_tr_pca, y_tr, X_va_pca, y_va))

    return reg, clf, n_comps


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", default="data")
    ap.add_argument("--cv", type=int, default=5)
    ap.add_argument("--no-subject-center", action="store_true")
    ap.add_argument("--var-threshold", type=float, default=0.90,
                     help="fraction of variance the PCA reduction must retain (per fold, train-fit)")
    ap.add_argument("--top-k", type=int, default=8, help="features shown per component loading")
    args = ap.parse_args()
    subject_center = not args.no_subject_center

    merged_path = Path(args.data_dir) / "merged_trials.csv"
    names = ["Valence", "Arousal", "Dominance"]

    # 1. Diagnostic: fit PCA once on the whole (preprocessed) dataset to look
    # at explained variance and loadings. Unsupervised (no labels), same
    # justification as the global winsorize/_usable_columns steps in
    # dataset.py - not used for any performance number below.
    df, teacher_cols, student_cols = _prepare(merged_path, subject_center)
    teacher_X = StandardScaler().fit_transform(df[teacher_cols].to_numpy(dtype=np.float32))
    student_X = StandardScaler().fit_transform(df[student_cols].to_numpy(dtype=np.float32))

    teacher_pca = explained_variance_curve(teacher_X, teacher_cols, "teacher (bio+eeg)")
    print_top_loadings(teacher_pca, teacher_cols, "teacher (bio+eeg)", top_k=args.top_k)
    student_pca = explained_variance_curve(student_X, student_cols, "student (AUs)")
    print_top_loadings(student_pca, student_cols, "student (AUs)", top_k=args.top_k)

    # 2. Performance: does reducing to var_threshold's worth of components
    # preserve the ridge-recoverable regression/classification signal, via
    # the same subject-wise CV harness as train_distill.py?
    reg, clf, n_comps = cv_pca_performance(merged_path, args.cv, subject_center, args.var_threshold)

    print(f"\n=== Ridge R^2, full-dim vs PCA({args.var_threshold:.0%} variance), "
          f"{args.cv}-fold mean+/-std (subject_center={subject_center}) ===")
    for name in ("teacher", "student"):
        k = n_comps[name]
        print(f"{name} (avg {np.mean(k):.1f} components kept):")
        full_r2 = np.stack(reg[name]["full"])
        pca_r2 = np.stack(reg[name]["pca"])
        print(f"  {'full':6s}" + "".join(f"{n:>12s}" for n in names))
        print(f"  {'':6s}" + "".join(f"{v:12.3f}" for v in full_r2.mean(axis=0)))
        print(f"  {'pca':6s}" + "".join(f"{v:12.3f}" for v in pca_r2.mean(axis=0)))

    print(f"\n=== Tertile classification macro-F1, full-dim vs PCA({args.var_threshold:.0%} variance) ===")
    for t in TARGET_COLS:
        print(f"{t}:")
        for name in ("teacher", "student"):
            full_vals = clf[t][f"{name}_full"]
            pca_vals = clf[t][f"{name}_pca"]
            print(f"  {name:8s} full={np.mean(full_vals):.3f}+/-{np.std(full_vals):.3f}  "
                  f"pca={np.mean(pca_vals):.3f}+/-{np.std(pca_vals):.3f}")


if __name__ == "__main__":
    main()
