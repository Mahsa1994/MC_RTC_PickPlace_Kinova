"""
Exploratory analysis of the MultiPhysio-HRC release (Zenodo 18668043): what the
dataset contains, and how usable it is for estimating stress, cognitive load
and affect from physiology. Writes eda/figures/*.png and prints the numbers
quoted in eda/EDA.md.

Uses the released per-window features (data/raw/features/), labels.csv, and the
facial-AU columns of data/merged_trials.csv, with the same loaders and
label joins as reproduce_paper.py.

    ~/.venvs/mphrc/bin/python explore_dataset.py --data-dir ./data
"""
import os

os.environ.setdefault("OMP_NUM_THREADS", "4")

import argparse
import warnings
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.colors import LinearSegmentedColormap, ListedColormap
from matplotlib.patches import Patch
from scipy.stats import rankdata
from sklearn.decomposition import PCA
from sklearn.ensemble import RandomForestClassifier
from sklearn.manifold import TSNE
from sklearn.metrics import balanced_accuracy_score
from sklearn.model_selection import GroupKFold, StratifiedKFold

import reproduce_paper as rp

warnings.filterwarnings("ignore", category=RuntimeWarning)

# ------------------------------------------------------------------ style
# Reference palette from the dataviz skill (references/palette.md), light mode.
SURFACE, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
S1, S2, S3 = "#2a78d6", "#eb6834", "#1baf7a"  # first three categorical slots: all-pairs CVD-safe
DEEMPH = "#d4d3cc"  # de-emphasis gray for "everything else"
BLUE = ["#cde2fb", "#b7d3f6", "#9ec5f4", "#86b6ef", "#6da7ec", "#5598e7", "#3987e5", "#2a78d6",
        "#256abf", "#1c5cab", "#184f95", "#104281", "#0d366b"]
SEQ = LinearSegmentedColormap.from_list("seq", BLUE)
DIV = LinearSegmentedColormap.from_list("div", ["#104281", "#3987e5", "#9ec5f4", "#f0efec",
                                                "#f2aaa8", "#e34948", "#a32a2a"])

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "axes.edgecolor": AXIS, "axes.linewidth": 0.8, "axes.labelcolor": INK2, "axes.titlecolor": INK,
    "axes.titlesize": 10, "axes.titleweight": "semibold", "axes.titlelocation": "left",
    "axes.spines.top": False, "axes.spines.right": False, "axes.grid": False,
    "grid.color": GRID, "grid.linewidth": 0.8, "grid.linestyle": "-",
    "xtick.color": MUTED, "ytick.color": MUTED, "xtick.labelcolor": INK2, "ytick.labelcolor": INK2,
    "xtick.major.size": 0, "ytick.major.size": 0, "font.size": 8.5, "text.color": INK,
    "legend.frameon": False, "legend.fontsize": 8, "savefig.dpi": 160, "savefig.bbox": "tight",
})

TASK_ORDER = ["rest", "meditation", "stroopeasy", "stroophard", "n-back", "mat", "hanoi",
              "vr-plank", "vr-job-sim", "manual-task", "cobot-task"]
TASK_NAME = {"rest": "Rest", "meditation": "Meditation", "stroopeasy": "Stroop easy", "stroophard": "Stroop hard",
             "n-back": "N-back", "mat": "MAT (arithmetic)", "hanoi": "Tower of Hanoi", "vr-plank": "VR plank",
             "vr-job-sim": "VR job sim", "manual-task": "Manual disassembly", "cobot-task": "Cobot disassembly"}
FAMILY = {"rest": "Rest", "meditation": "Rest", "manual-task": "HRC work", "cobot-task": "HRC work"}
FAMILY_ORDER, FAMILY_COLOR = ["Rest", "Lab stressors", "HRC work"], {"Rest": S3, "Lab stressors": S2, "HRC work": S1}
LABELS = ["STAI", "NASA", "Mental_Demand", "Physical_Demand", "Temporal_Demand", "Performance", "Effort",
          "Frustration", "Valence", "Arousal", "Dominance"]
LABEL_NAME = {"STAI": "STAI-Y1 (stress)", "NASA": "NASA-TLX (load)", "Mental_Demand": "TLX mental",
              "Physical_Demand": "TLX physical", "Temporal_Demand": "TLX temporal", "Performance": "TLX performance",
              "Effort": "TLX effort", "Frustration": "TLX frustration", "Valence": "SAM valence",
              "Arousal": "SAM arousal", "Dominance": "SAM dominance"}
MOD_NAME = {"physio": "Physio (ECG/EDA/EMG/RESP)", "eeg": "EEG", "voice": "Voice", "au": "Facial AUs"}
MOD_SHORT = {"physio": "Physio", "eeg": "EEG", "voice": "Voice", "au": "Facial AUs"}
OUT = Path("eda/figures")


def family(task):
    return FAMILY.get(task, "Lab stressors")


def hgrid(ax):
    ax.grid(axis="x")
    ax.set_axisbelow(True)


def vgrid(ax):
    ax.grid(axis="y")
    ax.set_axisbelow(True)


def save(fig, name):
    OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT / name)
    plt.close(fig)
    print(f"  -> {OUT / name}")


# ------------------------------------------------------------------ data

def load(data_dir):
    feat_dir = Path(data_dir) / "raw" / "features"
    lab = rp.load_labels(feat_dir)
    lab["task"] = lab["Class"]
    lab["family"] = lab["task"].map(family)
    mods = {m: rp.load_modality(feat_dir, m, lab) for m in rp.MODALITIES}

    trial = {}
    for m, d in mods.items():
        t = d["raw"].groupby("trial")[d["cols"]].mean()
        trial[m] = t
    merged = pd.read_csv(Path(data_dir) / "merged_trials.csv")
    au_cols = [c for c in merged.columns if c.startswith("AU_")]
    au = lab[["ID", "Class", "Repetition"]].reset_index().merge(
        merged[["ID", "Class", "Repetition"] + au_cols], on=["ID", "Class", "Repetition"], how="inner")
    trial["au"] = au.set_index("index")[au_cols]
    return lab, mods, trial


def zscore(X):
    mu, sd = np.nanmean(X, axis=0), np.nanstd(X, axis=0)
    return np.nan_to_num((X - mu) / np.where(sd > 0, sd, 1.0))


def subject_z(X, ids):
    """z-score each feature within subject (removes per-person baseline and scale)."""
    out = np.empty_like(X, dtype=float)
    for s in np.unique(ids):
        m = ids == s
        out[m] = zscore(X[m])
    return out


def trial_matrix(trial, lab, m, min_coverage=0.9):
    """Trial x feature matrix: drop features missing in >10% of trials, impute the rest
    with the column median."""
    t = trial[m]
    t = t.loc[:, t.notna().mean() >= min_coverage]
    X = t.to_numpy(float)
    med = np.nanmedian(X, axis=0)
    X = np.where(np.isnan(X), med, X)
    keep = np.nanstd(X, axis=0) > 0
    return X[:, keep], t.index.to_numpy(), [c for c, k in zip(t.columns, keep) if k]


# ------------------------------------------------------------------ figures

def fig_coverage(lab, trial):
    pres = pd.DataFrame({m: lab.index.isin(trial[m].index) for m in ["physio", "eeg", "voice", "au"]}, index=lab.index)
    lab = lab.assign(n_mod=pres.sum(axis=1))
    subj = sorted(lab["ID"].unique())
    grid = np.full((len(subj), len(TASK_ORDER)), np.nan)
    for (s, t), v in lab.groupby(["ID", "task"])["n_mod"].max().items():
        grid[subj.index(s), TASK_ORDER.index(t)] = v

    fig, (a, b) = plt.subplots(1, 2, figsize=(11, 6.6), gridspec_kw=dict(width_ratios=[1.35, 1]))
    cmap = ListedColormap([GRID, BLUE[3], BLUE[6], BLUE[9], BLUE[12]])  # 0..4 modalities, ordinal
    a.imshow(np.ma.masked_invalid(grid), aspect="auto", cmap=cmap, vmin=-0.5, vmax=4.5, interpolation="nearest")
    a.set_xticks(range(len(TASK_ORDER)), [TASK_NAME[t] for t in TASK_ORDER], rotation=40, ha="right")
    a.set_yticks(range(0, len(subj), 4), [str(s) for s in subj[::4]])
    a.set_ylabel("participant ID")
    a.set_title("Which participant did which task, and with how many modalities")
    for sp in a.spines.values():
        sp.set_visible(False)
    a.legend(handles=[Patch(color=c, label=l) for c, l in zip(
        [SURFACE, GRID, BLUE[3], BLUE[6], BLUE[9], BLUE[12]],
        ["no labelled trial", "label only", "1 modality", "2", "3", "all 4 (physio, EEG, voice, AU)"])],
        loc="upper left", bbox_to_anchor=(0, -0.2), ncol=3, handlelength=1, handleheight=1)

    shares = {MOD_NAME[m]: pres[m].mean() for m in ["physio", "eeg", "voice", "au"]}
    shares["All four"] = pres.all(axis=1).mean()
    shares["Physio + EEG + AU"] = pres[["physio", "eeg", "au"]].all(axis=1).mean()
    names = list(shares)[::-1]
    vals = [shares[n] * 100 for n in names]
    b.barh(names, vals, height=0.5, color=S1)
    for i, v in enumerate(vals):
        b.text(v + 1, i, f"{v:.0f}%", va="center", color=INK2)
    b.set_xlim(0, 112)
    b.set_xlabel(f"% of the {len(lab)} labelled trials")
    b.set_title("Share of labelled trials with each modality")
    hgrid(b)
    fig.tight_layout()
    save(fig, "01_coverage.png")
    print(f"coverage: {len(subj)} participants, {len(lab)} labelled trials; "
          + ", ".join(f"{k} {v:.0%}" for k, v in shares.items()))


def fig_volume(lab, mods):
    eeg = mods["eeg"]
    per_trial = pd.Series(eeg["trial"]).value_counts() * 5 / 60  # minutes of 5 s EEG windows
    df = pd.DataFrame({"min": per_trial, "task": lab.loc[per_trial.index, "task"].to_numpy()})
    g = df.groupby("task")["min"]
    stats = pd.DataFrame({"med": g.median(), "q1": g.quantile(0.25), "q3": g.quantile(0.75), "n": g.size()}).reindex(TASK_ORDER)
    windows = pd.Series(mods["physio"]["trial"]).value_counts()
    wf = pd.DataFrame({"w": windows, "task": lab.loc[windows.index, "task"].to_numpy()}).groupby("task")["w"].sum()

    fig, (a, b) = plt.subplots(1, 2, figsize=(10, 3.8), sharey=True)
    y = np.arange(len(TASK_ORDER))[::-1]
    a.barh(y, stats["med"], height=0.5, color=S1)
    a.errorbar(stats["med"], y, xerr=[stats["med"] - stats["q1"], stats["q3"] - stats["med"]],
               fmt="none", ecolor=INK2, elinewidth=1, capsize=0)
    a.set_yticks(y, [TASK_NAME[t] for t in TASK_ORDER])
    a.set_xlabel("minutes per trial (median, IQR whisker)")
    a.set_title("How long each task lasts")
    hgrid(a)
    share = (wf.reindex(TASK_ORDER) / wf.sum() * 100).fillna(0)
    b.barh(y, share, height=0.5, color=S1)
    for yi, v in zip(y, share):
        b.text(v + 0.5, yi, f"{v:.0f}%", va="center", color=INK2)
    b.set_xlabel("% of all 60 s physio windows")
    b.set_title("Where the window-level samples come from")
    hgrid(b)
    fig.tight_layout()
    save(fig, "02_volume.png")
    print("windows share HRC work: "
          f"{share[['manual-task', 'cobot-task']].sum():.0f}% of physio windows; trial minutes median by task: "
          + ", ".join(f"{t} {v:.1f}" for t, v in stats["med"].items()))


def fig_labels_by_task(lab):
    labels = ["STAI", "NASA", "Valence", "Arousal", "Dominance"]
    fig, axes = plt.subplots(1, 5, figsize=(13, 4.2), sharey=True)
    y = np.arange(len(TASK_ORDER))[::-1]
    for ax, l in zip(axes, labels):
        data = [lab.loc[lab.task == t, l].dropna().to_numpy() for t in TASK_ORDER]
        bp = ax.boxplot(data, positions=y, vert=False, widths=0.5, patch_artist=True, showfliers=False,
                        medianprops=dict(color=INK, linewidth=1.5), whiskerprops=dict(color=AXIS),
                        capprops=dict(color=AXIS), boxprops=dict(linewidth=0))
        for p in bp["boxes"]:
            p.set_facecolor(BLUE[3])
        ax.set_title(LABEL_NAME[l])
        hgrid(ax)
    axes[0].set_yticks(y, [TASK_NAME[t] for t in TASK_ORDER])
    fig.suptitle("Self-reports by task (box = IQR, line = median)", x=0.01, ha="left", fontsize=10,
                 fontweight="semibold")
    fig.tight_layout()
    save(fig, "03_labels_by_task.png")


def r2_onehot(y, *factors):
    X = np.column_stack([np.ones(len(y))] + [pd.get_dummies(f, drop_first=True).to_numpy(float) for f in factors])
    beta, *_ = np.linalg.lstsq(X, y, rcond=None)
    return 1 - np.sum((y - X @ beta) ** 2) / np.sum((y - y.mean()) ** 2)


def fig_variance(lab):
    rows = []
    for l in LABELS:
        d = lab[["ID", "task", l]].dropna()
        r_s = r2_onehot(d[l].to_numpy(float), d["ID"].to_numpy())
        r_st = r2_onehot(d[l].to_numpy(float), d["ID"].to_numpy(), d["task"].to_numpy())
        r_t = r2_onehot(d[l].to_numpy(float), d["task"].to_numpy())
        rows.append((l, r_s, r_st - r_s, 1 - r_st, r_t))
    v = pd.DataFrame(rows, columns=["label", "subject", "task", "residual", "task_alone"]).set_index("label")

    fig, ax = plt.subplots(figsize=(9, 4.4))
    y = np.arange(len(LABELS))[::-1]
    left = np.zeros(len(LABELS))
    gap = 0.004
    for col, color, name in [("subject", S1, "who the participant is"), ("task", S2, "which task (beyond the person)"),
                             ("residual", DEEMPH, "left over: within-task variation + noise")]:
        w = v[col].to_numpy() * 100
        ax.barh(y, np.maximum(w - gap * 100, 0), left=left, height=0.55, color=color, label=name)
        for yi, l0, wi in zip(y, left, w):
            if wi >= 7:
                ax.text(l0 + wi / 2, yi, f"{wi:.0f}%", ha="center", va="center",
                        color="white" if color != DEEMPH else INK, fontsize=7.5)
        left += w
    ax.set_yticks(y, [LABEL_NAME[l] for l in LABELS])
    ax.set_xlim(0, 100)
    ax.set_xlabel("% of label variance (subject entered first, then task)")
    ax.set_title("What explains the self-reports: the person, the task, or neither")
    ax.legend(loc="upper left", bbox_to_anchor=(0, -0.13), ncol=3)
    for sp in ["left", "bottom"]:
        ax.spines[sp].set_visible(False)
    fig.tight_layout()
    save(fig, "04_variance_decomposition.png")
    print("variance decomposition (subject / task-beyond-subject / residual / task alone):")
    print((v * 100).round(1).to_string())
    return v


def norm_entropy(x, k):
    _, c = np.unique(x, return_counts=True)
    p = c / c.sum()
    return float(-(p * np.log2(p)).sum() / np.log2(k))


def fig_entropy(lab):
    scales = {l: 11 for l in LABELS if l in ("Mental_Demand", "Physical_Demand", "Temporal_Demand",
                                              "Performance", "Effort", "Frustration")}
    scales.update({"Valence": 5, "Arousal": 5, "Dominance": 5})
    order = ["Valence", "Arousal", "Dominance", "Mental_Demand", "Physical_Demand", "Temporal_Demand",
             "Performance", "Effort", "Frustration"]
    ent, distinct = {}, {}
    for l in order:
        g = lab[["ID", l]].dropna().groupby("ID")[l]
        ent[l] = g.apply(lambda x: norm_entropy(x.to_numpy(), scales[l]))
        distinct[l] = g.nunique()

    fig, (a, b) = plt.subplots(1, 2, figsize=(11, 4), gridspec_kw=dict(width_ratios=[1.4, 1]), sharey=True)
    y = np.arange(len(order))[::-1]
    rng = np.random.default_rng(0)
    for yi, l in zip(y, order):
        e = ent[l].to_numpy()
        a.scatter(e, yi + rng.uniform(-0.18, 0.18, len(e)), s=10, color=BLUE[4], linewidths=0)
        a.plot([np.median(e)] * 2, [yi - 0.3, yi + 0.3], color=INK, linewidth=2, solid_capstyle="round")
    a.set_yticks(y, [LABEL_NAME[l] for l in order])
    a.set_xlim(0, 1)
    a.set_xlabel("normalized entropy of a participant's ratings (0 = always the same answer, 1 = uniform)")
    a.set_title("How much of each rating scale participants actually use")
    hgrid(a)
    flat = [(distinct[l] <= 2).mean() * 100 for l in order]
    b.barh(y, flat, height=0.5, color=S1)
    for yi, v in zip(y, flat):
        b.text(v + 1, yi, f"{v:.0f}%", va="center", color=INK2)
    b.set_xlim(0, max(flat) * 1.25 + 5)
    b.set_xlabel("% of participants using <= 2 distinct values")
    b.set_title("Near-flat responders")
    hgrid(b)
    fig.tight_layout()
    save(fig, "05_label_entropy.png")
    print("median normalized entropy: " + ", ".join(f"{l} {ent[l].median():.2f}" for l in order))
    print("flat responders (<=2 values): " + ", ".join(f"{l} {f:.0f}%" for l, f in zip(order, flat)))


def fig_label_corr(lab):
    d = lab[["ID"] + LABELS].dropna()
    c = d[LABELS] - d.groupby("ID")[LABELS].transform("mean")  # within-subject
    r = c.rank().corr().to_numpy()
    fig, ax = plt.subplots(figsize=(7.4, 6.2))
    im = ax.imshow(r, cmap=DIV, vmin=-1, vmax=1)
    names = [LABEL_NAME[l] for l in LABELS]
    ax.set_xticks(range(len(LABELS)), names, rotation=45, ha="right")
    ax.set_yticks(range(len(LABELS)), names)
    for i in range(len(LABELS)):
        for j in range(len(LABELS)):
            if i != j and abs(r[i, j]) >= 0.3:
                ax.text(j, i, f"{r[i, j]:.2f}", ha="center", va="center", fontsize=7,
                        color="white" if abs(r[i, j]) > 0.6 else INK)
    for sp in ax.spines.values():
        sp.set_visible(False)
    cb = fig.colorbar(im, ax=ax, shrink=0.7)
    cb.outline.set_visible(False)
    cb.set_label("within-participant Spearman r (labels shown for |r| >= 0.3)")
    ax.set_title("Do the questionnaires agree with each other (within a participant)?")
    fig.tight_layout()
    save(fig, "06_label_correlations.png")
    print(f"within-subject Spearman: STAI-NASA {r[0, 1]:.2f}, STAI-Arousal {r[0, 9]:.2f}, "
          f"NASA-Arousal {r[1, 9]:.2f}, STAI-Valence {r[0, 8]:.2f}")


def feature_family(m, c):
    if m == "physio":
        p = c.split("_")[0]
        return {"RRV": "Resp. (RSP/RRV/RAV)", "RSP": "Resp. (RSP/RRV/RAV)", "RAV": "Resp. (RSP/RRV/RAV)"}.get(p, p)
    if m == "eeg":
        if "/" in c:
            return "EEG theta/alpha ratios"
        if c.endswith("En"):
            return "EEG entropy"
        return "EEG band power"
    if m == "voice":
        return "Voice MFCC stats" if "MFCC" in c else "Voice prosody (f0/jitter/shimmer/HNR)"
    return "Facial AU mean" if "_mean_" in c else "Facial AU std"


def fig_feature_quality(mods, trial):
    rows = []
    feat_dir = Path("data/raw/features")
    for m in ["physio", "eeg", "voice"]:
        path, meta = rp.MODALITIES[m]
        raw = pd.read_csv(feat_dir / path)
        cols = [c for c in raw.columns if c not in meta]
        X = raw[cols].apply(pd.to_numeric, errors="coerce").replace([np.inf, -np.inf], np.nan)
        for c in cols:
            x = X[c]
            med = x.median()
            mad = (x - med).abs().median()
            rows.append(dict(family=feature_family(m, c), missing=x.isna().mean(),
                             constant=float(x.nunique(dropna=True) <= 1),
                             outlier=float(((x - med).abs() > 10 * 1.4826 * mad).mean()) if mad > 0 else 0.0))
    au = trial["au"]
    for c in au.columns:
        x = au[c]
        med, mad = x.median(), (x - x.median()).abs().median()
        rows.append(dict(family=feature_family("au", c), missing=x.isna().mean(), constant=float(x.nunique() <= 1),
                         outlier=float(((x - med).abs() > 10 * 1.4826 * mad).mean()) if mad > 0 else 0.0))
    q = pd.DataFrame(rows).groupby("family").agg(n=("missing", "size"), missing=("missing", "mean"),
                                                 constant=("constant", "mean"), outlier=("outlier", "mean"))
    order = ["HRV", "EDA", "EMG", "Resp. (RSP/RRV/RAV)", "EEG band power", "EEG entropy", "EEG theta/alpha ratios",
             "Voice prosody (f0/jitter/shimmer/HNR)", "Voice MFCC stats", "Facial AU mean", "Facial AU std"]
    q = q.reindex(order)
    fig, axes = plt.subplots(1, 3, figsize=(12, 4), sharey=True)
    y = np.arange(len(order))[::-1]
    for ax, col, title, xl in [(axes[0], "missing", "Missing / non-finite values", "% of values"),
                               (axes[1], "constant", "Constant (useless) features", "% of features in family"),
                               (axes[2], "outlier", "Extreme outliers (> 10 robust SD)", "% of values")]:
        v = q[col].to_numpy() * 100
        ax.barh(y, v, height=0.5, color=S1)
        for yi, vi in zip(y, v):
            ax.text(vi + max(v.max(), 1) * 0.02, yi, f"{vi:.1f}%", va="center", color=INK2, fontsize=7.5)
        ax.set_xlim(0, max(v.max(), 1) * 1.25)
        ax.set_title(title)
        ax.set_xlabel(xl)
        hgrid(ax)
    axes[0].set_yticks(y, [f"{f} ({int(n)})" for f, n in zip(order, q["n"])])
    fig.suptitle("Feature quality by family (window-level files; AUs per trial). (n) = number of features",
                 x=0.01, ha="left", fontsize=10, fontweight="semibold")
    fig.tight_layout()
    save(fig, "07_feature_quality.png")
    print("feature quality (% missing / % constant / % outlier values):")
    print((q[["missing", "constant", "outlier"]] * 100).round(2).to_string())


def fig_redundancy(lab, trial):
    rows = []
    for m in ["physio", "eeg", "voice", "au"]:
        X, idx, cols = trial_matrix(trial, lab, m)
        Z = subject_z(X, lab.loc[idx, "ID"].to_numpy())
        ev = PCA().fit(Z).explained_variance_
        pr = ev.sum() ** 2 / (ev ** 2).sum()
        k90 = int(np.searchsorted(np.cumsum(ev) / ev.sum(), 0.9) + 1)
        r = np.corrcoef(Z, rowvar=False)
        iu = np.triu_indices_from(r, 1)
        a = np.abs(r[iu])
        rows.append(dict(mod=m, features=len(cols), k90=k90, pr=pr, r09=(a > 0.9).mean(), r07=(a > 0.7).mean()))
    d = pd.DataFrame(rows).set_index("mod")
    fig, (a, b) = plt.subplots(1, 2, figsize=(11, 3.8))
    x = np.arange(len(d))
    for i, (col, color, name) in enumerate([("features", S1, "features"), ("k90", S2, "PCs for 90% variance"),
                                            ("pr", S3, "effective dimension (participation ratio)")]):
        a.bar(x + (i - 1) * 0.26, d[col], width=0.24, color=color, label=name)
        for xi, v in zip(x, d[col]):
            a.text(xi + (i - 1) * 0.26, v + 6, f"{v:.0f}", ha="center", color=INK2, fontsize=7.5)
    a.set_xticks(x, [MOD_SHORT[m] for m in d.index])
    a.set_title("How many independent directions each modality really has")
    a.legend(loc="upper left", bbox_to_anchor=(0, -0.1), ncol=3)
    vgrid(a)
    b.bar(x - 0.14, d["r07"] * 100, width=0.26, color=S1, label="|r| > 0.7")
    b.bar(x + 0.14, d["r09"] * 100, width=0.26, color=S2, label="|r| > 0.9")
    b.set_xticks(x, [MOD_SHORT[m] for m in d.index])
    b.set_ylabel("% of feature pairs")
    b.set_title("Redundant feature pairs (within-participant z-scored)")
    b.legend()
    vgrid(b)
    fig.tight_layout()
    save(fig, "08_redundancy.png")
    print("redundancy:\n" + d.round(3).to_string())


def nn_rates(Z, subj, task):
    sq = (Z ** 2).sum(1)
    D = sq[:, None] + sq[None, :] - 2 * Z @ Z.T  # squared Euclidean, without an N x N x F array
    np.fill_diagonal(D, np.inf)
    nn = D.argmin(axis=1)
    same_s, same_t = (subj[nn] == subj).mean(), (task[nn] == task).mean()
    n = len(subj)
    cs = np.mean([(np.sum(subj == s) - 1) / (n - 1) for s in subj])
    ct = np.mean([(np.sum(task == t) - 1) / (n - 1) for t in task])
    return same_s, same_t, cs, ct


def fig_similarity(lab, trial):
    rows = []
    for m in ["physio", "eeg", "voice", "au"]:
        X, idx, _ = trial_matrix(trial, lab, m)
        subj, task = lab.loc[idx, "ID"].to_numpy(), lab.loc[idx, "task"].to_numpy()
        raw = nn_rates(zscore(X), subj, task)
        cen = nn_rates(subject_z(X, subj), subj, task)
        rows.append(dict(mod=m, raw_s=raw[0], raw_t=raw[1], cen_s=cen[0], cen_t=cen[1], chance_s=raw[2], chance_t=raw[3]))
    d = pd.DataFrame(rows).set_index("mod")
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8), sharey=True)
    x = np.arange(len(d))
    for ax, key, title in [(axes[0], "s", "Nearest neighbour is the SAME PARTICIPANT"),
                           (axes[1], "t", "Nearest neighbour is the SAME TASK")]:
        ax.bar(x - 0.14, d[f"raw_{key}"] * 100, width=0.26, color=S1, label="features as released (global z-score)")
        ax.bar(x + 0.14, d[f"cen_{key}"] * 100, width=0.26, color=S2, label="z-scored within each participant")
        ax.scatter(x, d[f"chance_{key}"] * 100, marker="_", s=500, color=INK, linewidths=2, label="chance", zorder=3)
        for xi, v1, v2 in zip(x, d[f"raw_{key}"] * 100, d[f"cen_{key}"] * 100):
            ax.text(xi - 0.14, v1 + 1.5, f"{v1:.0f}", ha="center", color=INK2, fontsize=7.5)
            ax.text(xi + 0.14, v2 + 1.5, f"{v2:.0f}", ha="center", color=INK2, fontsize=7.5)
        ax.set_xticks(x, [MOD_SHORT[m] for m in d.index])
        ax.set_title(title)
        vgrid(ax)
    axes[0].set_ylabel("% of trials")
    axes[0].set_ylim(0, 105)
    axes[1].legend(loc="upper right")
    fig.suptitle("Trial similarity: is a trial most similar to the same person or to the same task?",
                 x=0.01, ha="left", fontsize=10, fontweight="semibold")
    fig.tight_layout()
    save(fig, "09_nearest_neighbour.png")
    print("nearest-neighbour rates:\n" + (d * 100).round(1).to_string())


def decode(X, y, groups=None, seed=0):
    """Balanced accuracy of a RandomForest; groups -> participant-wise 5-fold, else stratified 5-fold."""
    pred = np.empty_like(y)
    splits = (GroupKFold(n_splits=5).split(X, y, groups) if groups is not None
              else StratifiedKFold(n_splits=5, shuffle=True, random_state=seed).split(X, y))
    for tr, te in splits:
        clf = RandomForestClassifier(n_estimators=300, n_jobs=4, random_state=seed).fit(X[tr], y[tr])
        pred[te] = clf.predict(X[te])
    return balanced_accuracy_score(y, pred)


def fig_decodability(lab, trial):
    rows = []
    for m in ["physio", "eeg", "voice", "au"]:
        X, idx, _ = trial_matrix(trial, lab, m)
        l = lab.loc[idx]
        subj = l["ID"].to_numpy()
        keep_s = pd.Series(subj).map(pd.Series(subj).value_counts()).to_numpy() >= 5  # stratified 5-fold needs >=5
        Zs = subject_z(X, subj)
        rows.append(dict(
            mod=m,
            subject=decode(zscore(X)[keep_s], subj[keep_s]),
            subject_chance=1 / len(np.unique(subj[keep_s])),
            task=decode(Zs, l["task"].to_numpy(), groups=subj), task_chance=1 / l["task"].nunique(),
            stai=decode(Zs, l["STAI_cls"].to_numpy(), groups=subj),
            nasa=decode(Zs, l["NASA_cls"].to_numpy(), groups=subj),
            arousal=decode(Zs, np.digitize(l["Arousal"].to_numpy(), [1.5, 2.5]), groups=subj),
        ))
    d = pd.DataFrame(rows).set_index("mod")
    panels = [("subject", "Recognise WHO it is\n(trials held out, people seen)", "subject_chance"),
              ("task", "Recognise WHICH TASK\n(unseen participants)", "task_chance"),
              ("stai", "Stress class, STAI\n(unseen participants)", None),
              ("nasa", "Cognitive-load class, NASA\n(unseen participants)", None),
              ("arousal", "SAM arousal 1 / 2 / 3-5\n(unseen participants)", None)]
    fig, axes = plt.subplots(1, 5, figsize=(14, 3.9), sharey=True)
    x = np.arange(len(d))
    for ax, (col, title, ch) in zip(axes, panels):
        ax.bar(x, d[col] * 100, width=0.5, color=S1)
        chance = d[ch].to_numpy() * 100 if ch else np.full(len(d), 100 / 3)
        ax.scatter(x, chance, marker="_", s=400, color=INK, linewidths=2, zorder=3, label="chance")
        for xi, v in zip(x, d[col] * 100):
            ax.text(xi, v + 2, f"{v:.0f}", ha="center", color=INK2, fontsize=7.5)
        ax.set_xticks(x, ["Physio", "EEG", "Voice", "AUs"])
        ax.set_title(title, fontsize=9)
        vgrid(ax)
    axes[0].set_ylabel("balanced accuracy, %")
    axes[0].set_ylim(0, 105)
    axes[-1].legend(loc="upper right")
    fig.suptitle("What the features encode: person identity vs task vs self-reported state (RandomForest, 5-fold)",
                 x=0.01, ha="left", fontsize=10, fontweight="semibold")
    fig.tight_layout()
    save(fig, "10_decodability.png")
    print("decodability (balanced accuracy):\n" + (d * 100).round(1).to_string())


def fig_embedding(lab, trial):
    fig, axes = plt.subplots(2, 3, figsize=(12, 7.6))
    for row, m in enumerate(["physio", "eeg"]):
        X, idx, _ = trial_matrix(trial, lab, m)
        l = lab.loc[idx]
        subj, fam = l["ID"].to_numpy(), l["family"].to_numpy()
        E_raw = TSNE(perplexity=30, random_state=0, init="pca").fit_transform(zscore(X))
        E_cen = TSNE(perplexity=30, random_state=0, init="pca").fit_transform(subject_z(X, subj))
        top = pd.Series(subj).value_counts().index[:3]
        ax = axes[row, 0]
        rest = ~np.isin(subj, top)
        ax.scatter(*E_raw[rest].T, s=9, color=DEEMPH, linewidths=0, label="other participants")
        for s, c in zip(top, [S1, S2, S3]):
            mm = subj == s
            ax.scatter(*E_raw[mm].T, s=22, color=c, edgecolors=SURFACE, linewidths=1, label=f"participant {s}")
        ax.set_title(f"{MOD_SHORT[m]}, as released:\n3 participants highlighted")
        ax.legend(loc="lower left", fontsize=7, markerscale=1)
        for col, E, t in [(1, E_raw, "as released:\ncoloured by task family"),
                          (2, E_cen, "z-scored within participant:\ncoloured by task family")]:
            ax = axes[row, col]
            for f in FAMILY_ORDER:
                mm = fam == f
                ax.scatter(*E[mm].T, s=12, color=FAMILY_COLOR[f], edgecolors=SURFACE, linewidths=0.6, label=f)
            ax.set_title(f"{MOD_SHORT[m]}, {t}")
            if col == 2:
                ax.legend(loc="lower left", fontsize=7)
        for ax in axes[row]:
            ax.set_xticks([])
            ax.set_yticks([])
            for sp in ax.spines.values():
                sp.set_visible(False)
    fig.suptitle("t-SNE maps of trials (each dot = one trial)", x=0.01, ha="left", fontsize=10, fontweight="semibold")
    fig.tight_layout()
    save(fig, "11_tsne.png")


def standardized_ranks(Z):
    R = np.apply_along_axis(rankdata, 0, Z)
    return (R - R.mean(0)) / (R.std(0) + 1e-12)


def spearman_matrix(R, y):
    """|Spearman r| of each feature with y; R = standardized_ranks(features), both within-subject centered."""
    ry = rankdata(y)
    ry = (ry - ry.mean()) / (ry.std() + 1e-12)
    return np.abs(R.T @ ry / len(ry))


def fig_information(lab, trial, n_perm=200):
    targets = ["STAI", "NASA", "Arousal"]
    rng = np.random.default_rng(0)
    rows, scatter = [], {}
    for m in ["physio", "eeg", "voice", "au"]:
        X, idx, cols = trial_matrix(trial, lab, m)
        l = lab.loc[idx]
        subj = l["ID"].to_numpy()
        Zc = X - pd.DataFrame(X).groupby(subj).transform("mean").to_numpy()
        # task effect per feature, within subject: eta^2 of task on the centred feature
        tm = pd.DataFrame(Zc).groupby(l["task"].to_numpy()).transform("mean").to_numpy()
        eta = (tm ** 2).sum(0) / ((Zc ** 2).sum(0) + 1e-12)
        R = standardized_ranks(Zc)
        for t in targets:
            y = l[t].to_numpy(float)
            yc = y - pd.Series(y).groupby(subj).transform("mean").to_numpy()
            r = spearman_matrix(R, yc)
            null_max = []
            for _ in range(n_perm):  # shuffle labels within participant: keeps person structure
                yp = yc.copy()
                for s in np.unique(subj):
                    mm = subj == s
                    yp[mm] = rng.permutation(yp[mm])
                null_max.append(spearman_matrix(R, yp).max())
            thr = np.quantile(null_max, 0.95)
            rows.append(dict(mod=m, target=t, max_r=r.max(), thr=thr, n_sig=int((r > thr).sum()), n=len(r),
                             top=cols[int(r.argmax())]))
            scatter[(m, t)] = (eta, r, thr)
    d = pd.DataFrame(rows)

    fig, axes = plt.subplots(2, 4, figsize=(13, 6.4), sharex=True, sharey=True)
    for j, m in enumerate(["physio", "eeg", "voice", "au"]):
        for i, t in enumerate(["NASA", "STAI"]):
            eta, r, thr = scatter[(m, t)]
            ax = axes[i, j]
            ax.scatter(eta, r, s=9, color=S1, alpha=0.6, linewidths=0)
            ax.axhline(thr, color=INK2, linewidth=1)
            ax.text(0.99, thr + 0.01, "max |r| under shuffled labels (95%)", ha="right", va="bottom",
                    color=INK2, fontsize=6.5, transform=ax.get_yaxis_transform())
            ax.set_title(f"{MOD_SHORT[m]} vs {LABEL_NAME[t]}", fontsize=8.5)
            ax.grid(True)
            ax.set_axisbelow(True)
            if i == 1:
                ax.set_xlabel("how strongly the feature tracks the TASK\n(within-participant eta squared)")
            if j == 0:
                ax.set_ylabel("|Spearman r| with the label\n(within participant)")
    fig.suptitle("Do features that follow the label just follow the task? (each dot = one feature)",
                 x=0.01, ha="left", fontsize=10, fontweight="semibold")
    fig.tight_layout()
    save(fig, "12_feature_label_information.png")
    print("feature-label information (within-subject |Spearman|, permutation 95% max threshold):")
    print(d.round(3).to_string(index=False))
    eta_corr = {k: np.corrcoef(v[0], v[1])[0, 1] for k, v in scatter.items()}
    print("corr(task eta^2, |r| with label) per modality/target: "
          + ", ".join(f"{m}/{t} {c:.2f}" for (m, t), c in eta_corr.items()))


def fig_task_similarity(lab, trial):
    tasks = [t for t in TASK_ORDER if (lab["task"] == t).sum() >= 10]  # one shared order for every panel
    fig, axes = plt.subplots(1, 4, figsize=(14, 4.2), constrained_layout=True)
    for i, (ax, m) in enumerate(zip(axes, ["physio", "eeg", "voice", "au"])):
        X, idx, _ = trial_matrix(trial, lab, m)
        l = lab.loc[idx]
        Z = subject_z(X, l["ID"].to_numpy())
        P = np.full((len(tasks), Z.shape[1]), np.nan)
        for k, t in enumerate(tasks):
            mm = l["task"].to_numpy() == t
            if mm.sum() >= 10:
                P[k] = Z[mm].mean(0)
        P = P / np.linalg.norm(P, axis=1, keepdims=True)
        S = P @ P.T  # NaN rows/cols (task not recorded in this modality) stay blank
        im = ax.imshow(np.ma.masked_invalid(S), cmap=DIV, vmin=-1, vmax=1)
        ax.set_xticks(range(len(tasks)), [TASK_NAME[t] for t in tasks], rotation=60, ha="right", fontsize=7)
        ax.set_yticks(range(len(tasks)), [TASK_NAME[t] for t in tasks] if i == 0 else [], fontsize=7)
        ax.set_title(MOD_SHORT[m] + (" (no VR recordings)" if m == "au" else ""), fontsize=9)
        for sp in ax.spines.values():
            sp.set_visible(False)
    cb = fig.colorbar(im, ax=axes, shrink=0.75)
    cb.outline.set_visible(False)
    cb.set_label("cosine similarity of task profiles")
    fig.suptitle("Which tasks look alike physiologically (task-average deviation from each participant's own baseline)",
                 x=0.01, ha="left", fontsize=10, fontweight="semibold")
    save(fig, "13_task_similarity.png")


def fig_repetitions(lab):
    fig, axes = plt.subplots(1, 3, figsize=(11, 3.4), sharex=True)
    for ax, t in zip(axes, ["NASA", "STAI", "Arousal"]):
        for task, color in [("manual-task", S2), ("cobot-task", S1)]:
            d = lab[lab.task == task].groupby("Repetition")[t]
            m, se, n = d.mean(), d.std() / np.sqrt(d.size()), d.size()
            ax.fill_between(m.index, m - 1.96 * se, m + 1.96 * se, color=color, alpha=0.1, linewidth=0)
            ax.plot(m.index, m, color=color, linewidth=2, marker="o", markersize=5, markeredgecolor=SURFACE,
                    markeredgewidth=1.5, label=TASK_NAME[task])
            ax.text(m.index[-1] + 0.12, m.iloc[-1], TASK_NAME[task].split()[0], va="center", color=INK2, fontsize=7.5)
        ax.set_title(LABEL_NAME[t])
        ax.set_xticks(range(1, 6))
        ax.set_xlim(0.8, 5.9)
        ax.set_xlabel("repetition")
        vgrid(ax)
    axes[0].legend(loc="upper right")
    fig.suptitle("Do ratings drift over the 5 repetitions of the disassembly tasks? (mean, 95% CI)",
                 x=0.01, ha="left", fontsize=10, fontweight="semibold")
    fig.tight_layout()
    save(fig, "14_repetitions.png")
    for task in ["manual-task", "cobot-task"]:
        g = lab[lab.task == task].groupby("Repetition")[["NASA", "STAI", "Arousal"]].mean().round(2)
        print(f"{task} by repetition:\n{g.to_string()}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", default="data")
    ap.add_argument("--only", nargs="*", help="figure numbers to (re)draw, e.g. 1 4 9")
    args = ap.parse_args()
    lab, mods, trial = load(args.data_dir)
    print(f"{len(lab)} labelled trials, {lab.ID.nunique()} participants; trial-level features: "
          + ", ".join(f"{m} {t.shape}" for m, t in trial.items()))
    figs = {1: lambda: fig_coverage(lab, trial), 2: lambda: fig_volume(lab, mods), 3: lambda: fig_labels_by_task(lab),
            4: lambda: fig_variance(lab), 5: lambda: fig_entropy(lab), 6: lambda: fig_label_corr(lab),
            7: lambda: fig_feature_quality(mods, trial), 8: lambda: fig_redundancy(lab, trial),
            9: lambda: fig_similarity(lab, trial), 10: lambda: fig_decodability(lab, trial),
            11: lambda: fig_embedding(lab, trial), 12: lambda: fig_information(lab, trial),
            13: lambda: fig_task_similarity(lab, trial), 14: lambda: fig_repetitions(lab)}
    for k, f in figs.items():
        if not args.only or str(k) in args.only:
            print(f"\n[figure {k}]")
            f()


if __name__ == "__main__":
    main()
