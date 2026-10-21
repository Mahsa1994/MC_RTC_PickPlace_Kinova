"""
Valence/arousal circumplex quadrants from heart rate (HRV), skin conductance
(EDA) and facial action units (AUs) only, on the cleaned dataset
(clean_dataset.py). Compares two ways of getting a quadrant:

  (a) classify: one 4-class classifier on the quadrant label;
  (b) regress:  two regressors on within-participant z-scored valence and
      arousal (V_z, A_z). The quadrant is read off the signs of the two
      predictions. The labels are defined by exactly those signs (above/below
      the participant's own mean), so (b) also yields a continuous intensity.

Features:
- HRV (ECG) and EDA come from bio_features_60s.csv: window-level
  winsorized to [0.5, 99.5] percentiles, then averaged per trial.
- AUs: 20 action units x per-trial mean/std (merged_trials.csv).
- Every feature is z-scored within its participant. The EDA showed the raw
  features mostly encode identity. This is transductive but label-free: at
  deployment it needs an unlabelled calibration recording. Missing values ->
  0 (the participant's average).
- Evaluated on trials that have both physio and AUs, so all feature sets
  are scored on the same trials. VR tasks drop out here (no face video).

Evaluation:
- Participant-wise 5-fold GroupKFold, repeated with --repeats different
  participant groupings; test participants are never seen in training.
- Out-of-fold predictions are pooled per repeat. Metrics are mean +/- std
  over repeats.
- References:
  - chance: random guessing with the class priors gives macro-F1 = 0.25;
  - task identity only: one-hot task name through the same (a)/(b) pipeline.

    ~/.venvs/mphrc/bin/python train_va_quadrants.py --data-dir ./data
"""
import os

os.environ.setdefault("OMP_NUM_THREADS", "2")

import argparse
import json
import warnings
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.stats import spearmanr, wilcoxon
from sklearn.ensemble import (HistGradientBoostingClassifier, HistGradientBoostingRegressor, RandomForestClassifier,
                              RandomForestRegressor)
from sklearn.linear_model import LogisticRegression, Ridge
from sklearn.metrics import balanced_accuracy_score, confusion_matrix, f1_score
from sklearn.model_selection import GroupKFold

from clean_dataset import KEY, QUADRANTS, clean
from prepare_dataset import normalize_class
from reproduce_paper import attach_trials

warnings.filterwarnings("ignore", category=RuntimeWarning)

FEATURE_SETS = {"HR+EDA": ("hrv", "eda"), "AUs": ("au",), "HR+EDA+AUs": ("hrv", "eda", "au")}
MODELS = {  # family -> (classifier for (a), regressor for (b))
    "linear": (lambda s: LogisticRegression(C=0.1, class_weight="balanced", max_iter=5000),
               lambda s: Ridge(alpha=10.0)),
    "random forest": (lambda s: RandomForestClassifier(n_estimators=300, min_samples_leaf=3,
                                                       class_weight="balanced_subsample", n_jobs=2, random_state=s),
                      lambda s: RandomForestRegressor(n_estimators=300, min_samples_leaf=5, max_features=0.33,
                                                      n_jobs=2, random_state=s)),
    "gradient boosting": (lambda s: HistGradientBoostingClassifier(max_depth=3, learning_rate=0.05, max_iter=200,
                                                                   class_weight="balanced", random_state=s),
                          lambda s: HistGradientBoostingRegressor(max_depth=3, learning_rate=0.05, max_iter=200,
                                                                  random_state=s)),
}


# ------------------------------------------------------------------ data

def load(data_dir):
    lab, _, _ = clean(data_dir)
    feat_dir = Path(data_dir) / "raw" / "features"
    bio = pd.read_csv(feat_dir / "bio_features_60s.csv")
    bio["Class"] = normalize_class(bio["Class"])
    cols = {"hrv": [c for c in bio.columns if c.startswith("HRV_")],
            "eda": [c for c in bio.columns if c.startswith("EDA_")]}
    allc = cols["hrv"] + cols["eda"]
    bio[allc] = bio[allc].replace([np.inf, -np.inf], np.nan)
    bio[allc] = bio[allc].clip(bio[allc].quantile(0.005), bio[allc].quantile(0.995), axis=1)
    phys = attach_trials(bio[KEY + allc], lab).groupby("trial")[allc].mean()

    merged = pd.read_csv(Path(data_dir) / "merged_trials.csv")
    cols["au"] = [c for c in merged.columns if c.startswith("AU_")]
    au = lab[KEY].reset_index().merge(merged[KEY + cols["au"]], on=KEY, how="inner").set_index("index")[cols["au"]]

    keep = lab.index[lab.index.isin(phys.index) & lab.index.isin(au.index)]
    lab = lab.loc[keep]
    X = pd.concat([phys.loc[keep], au.loc[keep]], axis=1)
    usable = X.columns[X.notna().mean() > 0.5]
    X = X[usable]
    Z = (X - X.groupby(lab["ID"]).transform("mean")) / X.groupby(lab["ID"]).transform("std")
    Z = Z.replace([np.inf, -np.inf], np.nan).fillna(0.0)
    Z = Z.loc[:, Z.std() > 1e-9]
    groups = {k: [c for c in v if c in Z.columns] for k, v in cols.items()}
    return lab.reset_index(drop=True), Z.reset_index(drop=True), groups


def matrix(Z, groups, feature_set, lab):
    if feature_set == "task identity":
        return pd.get_dummies(lab["Class"]).to_numpy(float)
    return Z[[c for g in FEATURE_SETS[feature_set] for c in groups[g]]].to_numpy(float)


def quadrant_from_signs(v, a):
    v_hi, a_hi = v > 0, a > 0
    return np.select([v_hi & ~a_hi, v_hi & a_hi, ~v_hi & ~a_hi], [0, 1, 2], default=3)


# ------------------------------------------------------------------ CV

def run(lab, Z, groups, repeats):
    y_q, v_z, a_z = lab["quadrant"].to_numpy(), lab["V_z"].fillna(0).to_numpy(), lab["A_z"].fillna(0).to_numpy()
    ids = lab["ID"].to_numpy()
    feature_sets = list(FEATURE_SETS) + ["task identity"]
    oof = {}  # (feature_set, family, approach, repeat) -> dict of OOF arrays
    for fs in feature_sets:
        X = matrix(Z, groups, fs, lab)
        fams = ["linear"] if fs == "task identity" else list(MODELS)
        for fam in fams:
            make_clf, make_reg = MODELS[fam]
            for r in range(repeats):
                pq = np.zeros(len(lab), dtype=int)
                pv, pa = np.zeros(len(lab)), np.zeros(len(lab))
                for tr, te in GroupKFold(n_splits=5, shuffle=True, random_state=r).split(X, y_q, ids):
                    pq[te] = make_clf(r).fit(X[tr], y_q[tr]).predict(X[te])
                    pv[te] = make_reg(r).fit(X[tr], v_z[tr]).predict(X[te])
                    pa[te] = make_reg(r).fit(X[tr], a_z[tr]).predict(X[te])
                oof[(fs, fam, "a", r)] = dict(q=pq)
                oof[(fs, fam, "b", r)] = dict(q=quadrant_from_signs(pv, pa), v=pv, a=pa)
            print(f"  done: {fs} / {fam}", flush=True)
    return oof


def metrics(lab, pred):
    y = lab["quadrant"].to_numpy()
    v_true, a_true = np.isin(y, [0, 1]), np.isin(y, [1, 3])
    v_pred, a_pred = np.isin(pred["q"], [0, 1]), np.isin(pred["q"], [1, 3])
    m = dict(macro_f1=f1_score(y, pred["q"], average="macro"), bal_acc=balanced_accuracy_score(y, pred["q"]),
             valence_bal_acc=balanced_accuracy_score(v_true, v_pred),
             arousal_bal_acc=balanced_accuracy_score(a_true, a_pred))
    if "v" in pred:  # within-participant rank agreement of the continuous predictions
        for dim, col in (("v", "V_z"), ("a", "A_z")):
            rs = [spearmanr(lab.loc[g.index, col], pred[dim][g.index]).statistic
                  for _, g in lab.groupby("ID") if lab.loc[g.index, col].nunique() > 1]
            m[f"{dim}_within_spearman"] = float(np.nanmean(rs))
    return m


def per_participant_f1(lab, oof, key, repeats):
    out = {}
    for pid, g in lab.groupby("ID"):
        out[pid] = np.mean([f1_score(lab.loc[g.index, "quadrant"], oof[key + (r,)]["q"][g.index], average="macro",
                                     labels=[0, 1, 2, 3], zero_division=0) for r in range(repeats)])
    return pd.Series(out)


# ------------------------------------------------------------------ report

def report(lab, oof, repeats, out_dir):
    rows = []
    for (fs, fam, ap, r), pred in oof.items():
        rows.append(dict(features=fs, model=fam, approach=ap, repeat=r, **metrics(lab, pred)))
    df = pd.DataFrame(rows)
    agg = df.groupby(["features", "model", "approach"]).agg(["mean", "std"]).drop(columns="repeat")
    pd.set_option("display.width", 200)
    print("\n=== quadrant results (mean +/- std over repeats; chance macro-F1 = 0.25) ===")
    show = df.groupby(["features", "model", "approach"])[
        ["macro_f1", "bal_acc", "valence_bal_acc", "arousal_bal_acc", "v_within_spearman", "a_within_spearman"]].mean()
    show.insert(1, "macro_f1_std", df.groupby(["features", "model", "approach"])["macro_f1"].std())
    print(show.round(3).to_string())

    sensors = df[df.features != "task identity"]
    piv = sensors.groupby(["features", "model", "approach"])["macro_f1"].mean().unstack("approach")
    print(f"\n(b) beats (a) in {(piv['b'] > piv['a']).sum()}/{len(piv)} sensor feature-set x model combos, "
          f"mean difference {(piv['b'] - piv['a']).mean():+.3f} macro-F1")
    best = {ap: sensors[sensors.approach == ap].groupby(["features", "model"])["macro_f1"].mean().idxmax() for ap in "ab"}
    print(f"\nbest (a): {best['a']}  macro-F1 {df[(df.approach == 'a') & (df.features == best['a'][0]) & (df.model == best['a'][1])].macro_f1.mean():.3f}")
    print(f"best (b): {best['b']}  macro-F1 {df[(df.approach == 'b') & (df.features == best['b'][0]) & (df.model == best['b'][1])].macro_f1.mean():.3f}")
    fa = per_participant_f1(lab, oof, (*best["a"], "a"), repeats)
    fb = per_participant_f1(lab, oof, (*best["b"], "b"), repeats)
    w = wilcoxon(fa, fb)
    print(f"per-participant macro-F1, best (a) vs best (b): {fa.mean():.3f} vs {fb.mean():.3f}, "
          f"(b) better for {(fb > fa).sum()}/{len(fa)} participants, Wilcoxon p = {w.pvalue:.3f}")
    for ap in "ab":
        cm = confusion_matrix(lab["quadrant"], oof[(*best[ap], ap, 0)]["q"], labels=[0, 1, 2, 3], normalize="true")
        print(f"confusion (rows = true, row-normalized) best ({ap}):\n" + pd.DataFrame(
            cm, index=list(QUADRANTS.values()), columns=["calm", "happy", "sad/bored", "stressed"]).round(2).to_string())

    plot(df, lab, oof, best, out_dir)
    Path(out_dir, "va_quadrants.json").write_text(json.dumps(
        {f"{k[0]} | {k[1]} | {k[2]}": {c: float(v) for c, v in row.items()}
         for k, row in show.iterrows()}, indent=1))


def plot(df, lab, oof, best, out_dir):
    SURFACE, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
    S1, S2 = "#2a78d6", "#eb6834"
    BLUE = ["#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5", "#256abf", "#184f95", "#0d366b"]
    plt.rcParams.update({"figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
                         "axes.edgecolor": AXIS, "axes.spines.top": False, "axes.spines.right": False,
                         "xtick.major.size": 0, "ytick.major.size": 0, "xtick.color": MUTED, "ytick.color": MUTED,
                         "xtick.labelcolor": INK2, "ytick.labelcolor": INK2, "axes.labelcolor": INK2,
                         "axes.titlesize": 10, "axes.titleweight": "semibold", "axes.titlelocation": "left",
                         "font.size": 8.5, "legend.frameon": False, "grid.color": GRID, "savefig.dpi": 160,
                         "savefig.bbox": "tight"})
    task = df[df.features == "task identity"].groupby("approach")["macro_f1"].mean()
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.8), sharey=True)
    fsets = list(FEATURE_SETS)
    x = np.arange(len(fsets))
    for ax, fam in zip(axes, MODELS):
        for i, (ap, color, name) in enumerate([("a", S1, "(a) 4-class classifier"),
                                               ("b", S2, "(b) V/A regression -> signs")]):
            d = df[(df.model == fam) & (df.approach == ap)].groupby("features")["macro_f1"].agg(["mean", "std"]).reindex(fsets)
            xs = x + (i - 0.5) * 0.3
            ax.bar(xs, d["mean"], width=0.28, color=color, label=name)
            ax.errorbar(xs, d["mean"], yerr=d["std"], fmt="none", ecolor=INK2, elinewidth=1)
            for xi, v in zip(xs, d["mean"]):
                ax.text(xi, v + 0.012, f"{v:.2f}", ha="center", color=INK2, fontsize=7.5)
        ax.axhline(0.25, color=INK, linewidth=1.5)
        ax.text(len(fsets) - 0.5, 0.255, "chance", ha="right", va="bottom", color=INK2, fontsize=7.5)
        ax.axhline(task["a"], color=MUTED, linewidth=1)
        ax.text(-0.45, task["a"] + 0.005, f"task identity only ({task['a']:.2f})", va="bottom", color=MUTED, fontsize=7)
        ax.set_xticks(x, fsets)
        ax.set_title(fam)
        ax.grid(axis="y")
        ax.set_axisbelow(True)
    axes[0].set_ylabel("quadrant macro-F1 (unseen participants)")
    axes[0].set_ylim(0, 0.5)
    axes[-1].legend(loc="upper right")
    fig.suptitle("Circumplex quadrant from heart rate, skin conductance and facial AUs: (a) classify vs (b) regress",
                 x=0.01, ha="left", fontsize=10, fontweight="semibold")
    fig.tight_layout()
    fig.savefig(Path(out_dir, "va_quadrants_f1.png"))
    plt.close(fig)

    from matplotlib.colors import LinearSegmentedColormap
    cmap = LinearSegmentedColormap.from_list("seq", BLUE)
    names = ["calm", "happy", "sad/bored", "stressed"]
    fig, axes = plt.subplots(1, 2, figsize=(9, 4))
    for ax, ap in zip(axes, "ab"):
        cm = confusion_matrix(lab["quadrant"], oof[(*best[ap], ap, 0)]["q"], labels=[0, 1, 2, 3], normalize="true")
        ax.imshow(cm, cmap=cmap, vmin=0, vmax=1)
        for i in range(4):
            for j in range(4):
                ax.text(j, i, f"{cm[i, j]:.2f}", ha="center", va="center", fontsize=8,
                        color="white" if cm[i, j] > 0.5 else INK)
        ax.set_xticks(range(4), names)
        ax.set_yticks(range(4), names)
        ax.set_xlabel("predicted")
        ax.set_ylabel("true")
        ax.set_title(f"best ({ap}): {best[ap][1]}, {best[ap][0]}")
        for sp in ax.spines.values():
            sp.set_visible(False)
    fig.tight_layout()
    fig.savefig(Path(out_dir, "va_quadrants_confusion.png"))
    plt.close(fig)
    print(f"figures -> {out_dir}/va_quadrants_f1.png, va_quadrants_confusion.png")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", default="data")
    ap.add_argument("--repeats", type=int, default=5)
    ap.add_argument("--out-dir", default="results")
    args = ap.parse_args()
    lab, Z, groups = load(args.data_dir)
    bal = lab["quadrant_name"].value_counts(normalize=True).round(3).to_dict()
    print(f"{len(lab)} trials with physio + AUs, {lab.ID.nunique()} participants; features: "
          + ", ".join(f"{k} {len(v)}" for k, v in groups.items()) + f"; quadrant balance {bal}")
    oof = run(lab, Z, groups, args.repeats)
    Path(args.out_dir).mkdir(exist_ok=True)
    report(lab, oof, args.repeats, args.out_dir)


if __name__ == "__main__":
    main()
