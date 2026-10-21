"""
Builds a cleaned trial-label table for valence/arousal modelling, with
circumplex-quadrant classes defined relative to each participant.

Every exclusion rule is a data/label-quality rule, fixed before any model is
trained. Nothing is removed because a model does badly on it, which would
inflate results. Rules (thresholds are CLI flags):

  trial level
    T1  conflicting duplicates: labels.csv has 15 (ID, Class, Repetition) keys
        with two DIFFERENT questionnaires each (mostly rest/meditation). We
        can't tell which belongs to the recording, so both rows are dropped.
        Earlier loaders silently kept the first.
  participant level
    P1  too few trials: < --min-trials labelled trials. The participant's own
        mean, which the quadrant split uses, is unreliable.
    P2  degenerate valence or arousal: the rating is constant, or one side of
        the participant-mean split has < --min-side trials. A "high vs low
        relative to yourself" class means nothing there.
    P3  core sensors unusable: physio or EEG present for < --min-coverage of
        the participant's trials, or > --max-hrv-missing of their HRV values
        missing (ECG artifacts).
  window level (reported here; loaders apply it via frozen_eeg_windows())
    W1  dead EEG: a 5 s window whose median band power across channels is
        < 1e-4, or an exact repeat of an earlier window of the same trial.
        Typical windows have median power 0.24 and 99% sit above 0.016; below
        that is a thin tail with no sharp gap, so 1e-4 (~2,400x below the
        median) is a chosen, conservative "lost contact" cutoff. It flags 71
        of 40,976 windows (0.17%) in 11 trials. Worst case: in participant
        1070's stroophard trial the features cycle through 3 identical windows
        with alpha power ~1e-13.

Not excluded: participants missing facial AUs (modality-specific, handle at
model time), VR job sim trials (valid, just rare), the 10000-10978 IDs
(checked: distinct people, not copies).

Quadrants (Russell's circumplex, per participant): high = above the
participant's own mean rating on the clean trials, ties count as low.
  0 HV-LA "calm/relaxed"   1 HV-HA "happy/excited"
  2 LV-LA "sad/bored"      3 LV-HA "stressed"
The participant mean uses all their trials (the paper defines its per-subject
STAI/NASA classes the same way). At deployment it would come from a
calibration session.

    python clean_dataset.py --data-dir ./data      # -> data/clean_labels.csv + printed report
"""
import argparse
from pathlib import Path

import numpy as np
import pandas as pd

from prepare_dataset import normalize_class

KEY = ["ID", "Class", "Repetition"]
QUADRANTS = {0: "HV-LA calm/relaxed", 1: "HV-HA happy/excited", 2: "LV-LA sad/bored", 3: "LV-HA stressed"}


def _feature_keys(path):
    """(ID, Class, label-Repetition) keys a feature file covers, applying the
    0- vs 1-indexed single-run convention (feature rep r also matches label r+1)."""
    df = pd.read_csv(path, usecols=KEY)
    df["Class"] = normalize_class(df["Class"])
    k = df.drop_duplicates()
    return set(k.itertuples(index=False, name=None)) | set(
        k.assign(Repetition=k.Repetition + 1).itertuples(index=False, name=None))


def bad_eeg_windows(eeg, min_power=1e-4):
    """Boolean mask over eeg_features_5s rows: True for a dead window (rule W1)."""
    bands = [c for c in eeg.columns if c.endswith(("_Delta", "_Theta", "_Alpha", "_Beta", "_Gamma"))]
    feat = [c for c in eeg.columns if c not in KEY + ["Window"]]
    flat = eeg[bands].median(axis=1) < min_power
    repeat = eeg.duplicated(subset=KEY + feat, keep="first")
    return flat | repeat


def minority_side(x):
    hi = int((x > x.mean()).sum())
    return min(hi, len(x) - hi)


def clean(data_dir, min_trials=8, min_side=2, min_coverage=0.5, max_hrv_missing=0.2):
    feat_dir = Path(data_dir) / "raw" / "features"
    raw = pd.read_csv(feat_dir / "labels.csv")
    raw["Class"] = normalize_class(raw["Class"])
    report = []

    conflict = raw.duplicated(subset=KEY, keep=False)
    report.append(("T1 conflicting duplicate questionnaires", f"{int(conflict.sum())} rows "
                   f"({raw[conflict][KEY].drop_duplicates().shape[0]} trials)", "trials"))
    lab = raw[~conflict].copy()

    g = lab.groupby("ID")
    excluded = {}
    for pid in g.size().index[g.size() < min_trials]:
        excluded.setdefault(pid, []).append(f"P1 only {g.size()[pid]} trials")
    for dim in ("Valence", "Arousal"):
        for pid, x in g[dim]:
            if x.nunique() == 1:
                excluded.setdefault(pid, []).append(f"P2 constant {dim.lower()}")
            elif minority_side(x) < min_side:
                excluded.setdefault(pid, []).append(f"P2 {dim.lower()} split {minority_side(x)} vs {len(x) - minority_side(x)}")

    keys = {m: _feature_keys(feat_dir / f) for m, f in [("physio", "bio_features_60s.csv"), ("eeg", "eeg_features_5s.csv")]}
    for m, ks in keys.items():
        cov = lab.assign(has=[k in ks for k in lab[KEY].itertuples(index=False, name=None)]).groupby("ID")["has"].mean()
        for pid in cov.index[cov < min_coverage]:
            excluded.setdefault(pid, []).append(f"P3 {m} for only {cov[pid]:.0%} of trials")
    bio = pd.read_csv(feat_dir / "bio_features_60s.csv")
    hrv = [c for c in bio.columns if c.startswith("HRV_")]
    miss = bio.groupby("ID")[hrv].apply(lambda x: x.isna().to_numpy().mean())
    for pid in miss.index[miss > max_hrv_missing]:
        excluded.setdefault(pid, []).append(f"P3 {miss[pid]:.0%} of HRV values missing")

    lab = lab[~lab.ID.isin(excluded)].reset_index(drop=True)

    eeg = pd.read_csv(feat_dir / "eeg_features_5s.csv")
    eeg["Class"] = normalize_class(eeg["Class"])
    bad = bad_eeg_windows(eeg)
    report.append(("W1 dead EEG windows (drop in loaders via bad_eeg_windows())", f"{int(bad.sum())} windows in "
                   f"{eeg[bad][KEY].drop_duplicates().shape[0]} trial(s), "
                   f"participants {sorted(eeg[bad].ID.unique().tolist())}", "windows"))

    for dim, short in (("Valence", "V"), ("Arousal", "A"), ("Dominance", "D")):
        gm = lab.groupby("ID")[dim]
        lab[f"{short}_rel"] = lab[dim] - gm.transform("mean")
        lab[f"{short}_z"] = lab[f"{short}_rel"] / gm.transform("std").replace(0, np.nan)
    v_hi, a_hi = lab["V_rel"] > 0, lab["A_rel"] > 0
    lab["quadrant"] = np.select([v_hi & ~a_hi, v_hi & a_hi, ~v_hi & ~a_hi], [0, 1, 2], default=3)
    lab["quadrant_name"] = lab["quadrant"].map(QUADRANTS)
    lab["stressed"] = (lab["quadrant"] == 3).astype(int)
    return lab, excluded, report


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", default="data")
    ap.add_argument("--min-trials", type=int, default=8)
    ap.add_argument("--min-side", type=int, default=2)
    ap.add_argument("--min-coverage", type=float, default=0.5)
    ap.add_argument("--max-hrv-missing", type=float, default=0.2)
    args = ap.parse_args()

    lab, excluded, report = clean(args.data_dir, args.min_trials, args.min_side, args.min_coverage,
                                  args.max_hrv_missing)
    raw = pd.read_csv(Path(args.data_dir) / "raw" / "features" / "labels.csv")
    print(f"labels.csv: {len(raw)} rows, {raw.ID.nunique()} participants")
    for rule, what, _ in report:
        print(f"  {rule}: {what}")
    print(f"  participants excluded: {len(excluded)}")
    for pid, why in sorted(excluded.items()):
        print(f"    {pid}: {'; '.join(why)}")
    print(f"clean: {len(lab)} trials, {lab.ID.nunique()} participants "
          f"({lab.groupby('ID').size().min()}-{lab.groupby('ID').size().max()} trials each)")
    bal = lab["quadrant_name"].value_counts()
    print("quadrant balance: " + ", ".join(f"{k} {v} ({v / len(lab):.0%})" for k, v in bal.items()))
    print(f"stressed vs not: {lab.stressed.mean():.0%} stressed")
    out = Path(args.data_dir) / "clean_labels.csv"
    lab.to_csv(out, index=False)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
