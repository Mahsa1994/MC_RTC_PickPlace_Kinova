"""
Is the heart-rate signal in MultiPhysio-HRC healthy and informative?

The raw ECG isn't downloaded (15 GB physiological_data.zip). The 60 s window
features in bio_features_60s.csv carry enough to judge it:
  - heart rate: HR = 60000 / HRV_MedianNN (bpm). The median interval is
    robust to a few bad beats; the mean is not.
  - beat-detection artifacts, using the standard RR-artifact rule (an
    interval > 30% off the typical one):
      extra / false beat:  HRV_MinNN < 0.7 x HRV_MedianNN
      missed beat:         HRV_MaxNN > 1.3 x HRV_MedianNN
    A window with either is "artifact". Its variability features (RMSSD,
    SDNN, pNN50, LF/HF...) are then dominated by detection errors, not by the
    heart.

Writes eda/figures/hr_*.png and prints the numbers quoted in eda/EDA.md.

    ~/.venvs/mphrc/bin/python inspect_heart_rate.py --data-dir ./data
"""
import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.stats import spearmanr

from clean_dataset import KEY, clean
from explore_dataset import (AXIS, BLUE, DEEMPH, DIV, INK, INK2, MUTED, S1, S2, SURFACE, TASK_NAME, TASK_ORDER,
                             hgrid, save, vgrid)
from prepare_dataset import normalize_class
from reproduce_paper import attach_trials


def load_windows(data_dir):
    bio = pd.read_csv(Path(data_dir) / "raw" / "features" / "bio_features_60s.csv")
    bio["Class"] = normalize_class(bio["Class"])
    b = bio[KEY + ["Window", "HRV_MeanNN", "HRV_MedianNN", "HRV_MinNN", "HRV_MaxNN", "HRV_RMSSD"]].dropna().copy()
    b["extra"] = b.HRV_MinNN < 0.7 * b.HRV_MedianNN
    b["missed"] = b.HRV_MaxNN > 1.3 * b.HRV_MedianNN
    b["artifact"] = b.extra | b.missed
    b["HR"] = 60000 / b.HRV_MedianNN
    rest = b[b.Class == "rest"].groupby("ID")["HR"].median()  # baseline: all rest windows incl. unlabelled rep 0
    b["dHR"] = b["HR"] - b["ID"].map(rest)
    return b


def fig_quality(b):
    fig, axes = plt.subplots(1, 3, figsize=(14, 4), gridspec_kw=dict(width_ratios=[1.1, 1.6, 1]))
    ax = axes[0]
    bins = np.logspace(np.log10(5), np.log10(900), 40)
    ax.hist(b.loc[~b.artifact, "HRV_RMSSD"], bins=bins, color=S1, alpha=0.85, label="artifact-free windows")
    ax.hist(b.loc[b.artifact, "HRV_RMSSD"], bins=bins, color=S2, alpha=0.6, label="windows with extra/missed beats")
    ax.axvspan(20, 60, color=DEEMPH, alpha=0.5, zorder=0)
    ax.text(34, ax.get_ylim()[1] * 0.95, "typical\nhealthy", ha="center", va="top", color=INK2, fontsize=7.5)
    ax.set_xscale("log")
    ax.set_xlabel("RMSSD per 60 s window (ms, log scale)")
    ax.set_ylabel("windows")
    ax.set_title("Beat-to-beat variability: real vs artifact")
    ax.legend(loc="upper left", bbox_to_anchor=(0, -0.16), fontsize=7.5)
    vgrid(ax)

    ax = axes[1]
    per = b.groupby("ID")["artifact"].mean().sort_values() * 100
    ax.bar(range(len(per)), per, width=0.7, color=S1)
    ax.axhline(per.median(), color=INK, linewidth=1.2)
    ax.text(0, per.median() + 2, f"median participant: {per.median():.0f}% of windows", color=INK2, fontsize=7.5)
    ax.set_xticks(range(len(per)), [str(i) for i in per.index], rotation=90, fontsize=6)
    ax.set_ylabel("% of 60 s windows with artifacts")
    ax.set_ylim(0, 105)
    ax.set_title("ECG beat-detection artifacts per participant")
    vgrid(ax)

    ax = axes[2]
    task = (b.groupby("Class")["artifact"].mean() * 100).reindex(TASK_ORDER)
    y = np.arange(len(TASK_ORDER))[::-1]
    ax.barh(y, task, height=0.5, color=S1)
    for yi, v in zip(y, task):
        ax.text(v + 1, yi, f"{v:.0f}%", va="center", color=INK2, fontsize=7.5)
    ax.set_yticks(y, [TASK_NAME[t] for t in TASK_ORDER])
    ax.set_xlim(0, 110)
    ax.set_xlabel("% of windows with artifacts")
    ax.set_title("...by task (rest is no cleaner)")
    hgrid(ax)
    fig.tight_layout()
    save(fig, "hr_01_signal_quality.png")
    print(f"artifact windows: {b.artifact.mean():.0%} (extra beat {b.extra.mean():.0%}, missed {b.missed.mean():.0%}); "
          f"RMSSD median clean {b.loc[~b.artifact, 'HRV_RMSSD'].median():.1f} ms vs artifact "
          f"{b.loc[b.artifact, 'HRV_RMSSD'].median():.1f} ms; participants <20% artifacts: {(per < 20).sum()}/{len(per)}, "
          f">80%: {(per > 80).sum()}")


def task_change(b, clean_only):
    w = b[~b.artifact] if clean_only else b
    t = w.groupby(["ID", "Class"])["dHR"].median().unstack()
    return t


def fig_by_task(b):
    fig, (a, c) = plt.subplots(1, 2, figsize=(12, 4.2), sharey=True)
    y = np.arange(len(TASK_ORDER))[::-1]
    data = [b.loc[b.Class == t, "HR"].to_numpy() for t in TASK_ORDER]
    bp = a.boxplot(data, positions=y, vert=False, widths=0.5, patch_artist=True, showfliers=False,
                   medianprops=dict(color=INK, linewidth=1.5), whiskerprops=dict(color=AXIS), capprops=dict(color=AXIS),
                   boxprops=dict(linewidth=0))
    for p in bp["boxes"]:
        p.set_facecolor(BLUE[3])
    a.set_yticks(y, [TASK_NAME[t] for t in TASK_ORDER])
    a.set_xlabel("heart rate, bpm (60000 / median RR, per 60 s window)")
    a.set_title("Absolute heart rate by task (all participants pooled)")
    hgrid(a)

    rows = {}
    for i, (clean_only, color, name) in enumerate([(False, S1, "all windows"), (True, S2, "artifact-free windows only")]):
        t = task_change(b, clean_only).reindex(columns=TASK_ORDER)
        m, se, n = t.mean(), t.std() / np.sqrt(t.notna().sum()), t.notna().sum()
        yy = y + (0.15 if i == 0 else -0.15)
        c.errorbar(m, yy, xerr=1.96 * se, fmt="o", color=color, markersize=5, elinewidth=1.5, capsize=0,
                   markeredgecolor=SURFACE, markeredgewidth=1, label=name)
        rows[name] = pd.DataFrame({"mean_dHR": m.round(1), "ci95": (1.96 * se).round(1), "n_participants": n})
    c.axvline(0, color=INK, linewidth=1)
    c.set_xlabel("change from the participant's own rest HR, bpm (mean, 95% CI over participants)")
    c.set_title("Within-person HR change by task")
    c.legend(loc="lower right")
    hgrid(c)
    fig.tight_layout()
    save(fig, "hr_02_by_task.png")
    for name, r in rows.items():
        print(f"dHR by task ({name}):\n{r.to_string()}")


def fig_participant_task(b):
    t = task_change(b, clean_only=False).reindex(columns=TASK_ORDER)
    art = b.groupby("ID")["artifact"].mean()
    t = t.loc[art.sort_values().index]
    fig, (ax, side) = plt.subplots(1, 2, figsize=(10, 8.5), gridspec_kw=dict(width_ratios=[5, 1]), sharey=True)
    lim = 20
    im = ax.imshow(np.ma.masked_invalid(t.to_numpy()), aspect="auto", cmap=DIV, vmin=-lim, vmax=lim,
                   interpolation="nearest")
    ax.set_xticks(range(len(TASK_ORDER)), [TASK_NAME[x] for x in TASK_ORDER], rotation=40, ha="right")
    ax.set_yticks(range(len(t)), [str(i) for i in t.index], fontsize=6.5)
    ax.set_ylabel("participant (sorted: cleanest ECG at the top)")
    ax.set_title("HR change from own rest (bpm), per participant and task")
    for sp in ax.spines.values():
        sp.set_visible(False)
    cb = fig.colorbar(im, ax=ax, orientation="horizontal", fraction=0.03, pad=0.12)
    cb.set_label("bpm vs rest (blue = slower, red = faster; blank = task not recorded)")
    cb.outline.set_visible(False)
    side.barh(range(len(t)), art.loc[t.index] * 100, height=0.7, color=S1)
    side.set_xlim(0, 100)
    side.set_xlabel("% artifact\nwindows")
    side.set_title("ECG quality")
    hgrid(side)
    fig.tight_layout()
    save(fig, "hr_03_participant_task.png")


def trial_table(b, lab):
    w = attach_trials(b, lab)
    agg = w.groupby("trial").agg(HR=("HR", "median"), RMSSD=("HRV_RMSSD", "median"),
                                 art=("artifact", "mean"))
    clean_hr = w[~w.artifact].groupby("trial")["HR"].median().rename("HR_clean")
    return lab.join(agg, how="inner").join(clean_hr)


def fig_informative(b, lab):
    t = trial_table(b, lab)
    targets = [("Arousal", "SAM arousal"), ("STAI", "STAI stress"), ("NASA", "NASA-TLX load"), ("Valence", "SAM valence")]
    fig, (a, c) = plt.subplots(1, 2, figsize=(12, 4.2), gridspec_kw=dict(width_ratios=[1.6, 1]))
    rng = np.random.default_rng(0)
    summary = []
    for i, (col, name) in enumerate(targets):
        for j, (feat, color, fname) in enumerate([("HR", S1, "HR (all windows)"), ("HR_clean", S2, "HR (artifact-free)"),
                                                  ("RMSSD", MUTED, "RMSSD (all windows)")]):
            rs = []
            for _, g in t.groupby("ID"):
                g = g.dropna(subset=[feat, col])
                if len(g) >= 6 and g[col].nunique() > 1 and g[feat].nunique() > 1:
                    rs.append(spearmanr(g[feat], g[col]).statistic)
            rs = np.array(rs)
            x0 = i + (j - 1) * 0.26
            a.scatter(x0 + rng.uniform(-0.07, 0.07, len(rs)), rs, s=9, color=color, alpha=0.55, linewidths=0,
                      label=fname if i == 0 else None)
            a.plot([x0 - 0.1, x0 + 0.1], [np.median(rs)] * 2, color=INK, linewidth=2, solid_capstyle="round")
            summary.append(dict(target=col, feature=feat, median_r=np.median(rs), n=len(rs),
                                share_positive=(rs > 0).mean()))
    a.axhline(0, color=INK, linewidth=1)
    a.set_xticks(range(len(targets)), [n for _, n in targets])
    a.set_ylabel("within-participant Spearman r\n(trial HR or RMSSD vs self-report)")
    a.set_ylim(-1, 1)
    a.set_title("Does a person's HR follow their own ratings? (dot = participant, bar = median)")
    a.legend(loc="lower left", fontsize=7.5, ncol=3)
    vgrid(a)

    rest = t[t.Class == "rest"].pivot_table(index="ID", columns="Repetition", values="HR")
    rest = rest.dropna()
    lo, hi = rest.min().min() - 3, rest.max().max() + 3
    c.plot([lo, hi], [lo, hi], color=AXIS, linewidth=1)
    c.scatter(rest.iloc[:, 0], rest.iloc[:, 1], s=22, color=S1, edgecolors=SURFACE, linewidths=1)
    r = np.corrcoef(rest.iloc[:, 0], rest.iloc[:, 1])[0, 1]
    c.text(0.04, 0.94, f"r = {r:.2f}, n = {len(rest)} participants", transform=c.transAxes, color=INK2)
    c.set_xlabel("rest 1 HR (bpm)")
    c.set_ylabel("rest 2 HR (bpm)")
    c.set_title("Is resting HR stable within a person?")
    c.grid(True)
    c.set_axisbelow(True)
    fig.tight_layout()
    save(fig, "hr_04_informativeness.png")
    s = pd.DataFrame(summary)
    print("within-participant Spearman, median over participants:\n" + s.round(3).to_string(index=False))
    print(f"rest1 vs rest2 HR: r = {r:.2f} (n = {len(rest)}), mean |diff| {np.abs(rest.iloc[:, 0] - rest.iloc[:, 1]).mean():.1f} bpm")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", default="data")
    args = ap.parse_args()
    b = load_windows(args.data_dir)
    lab, _, _ = clean(args.data_dir)
    print(f"{len(b)} windows, {b.ID.nunique()} participants; clean label set {len(lab)} trials")
    fig_quality(b)
    fig_by_task(b)
    fig_participant_task(b)
    fig_informative(b, lab)


if __name__ == "__main__":
    main()
