"""
Loads the merged per-trial table (see prepare_dataset.py) into numpy arrays
for the teacher/student distillation PoC, with a subject-wise (grouped by
participant ID) train/val split so no participant's trials leak across the
split - the usual leakage bug in per-trial physiological datasets.
"""
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd
from sklearn.model_selection import GroupKFold, GroupShuffleSplit
from sklearn.preprocessing import StandardScaler

TARGET_COLS = ["Valence", "Arousal", "Dominance"]


@dataclass
class Split:
    teacher_X: np.ndarray
    student_X: np.ndarray
    y: np.ndarray
    ids: np.ndarray


def _usable_columns(df: pd.DataFrame, cols: list) -> list:
    """Drop columns that are entirely NaN or constant - real sensor feature
    tables like this one routinely have both, and an MLP will happily
    produce NaN loss forever if you don't filter them out first."""
    sub = df[cols]
    keep = sub.columns[sub.notna().all() & (sub.std(numeric_only=True) > 1e-9)]
    return list(keep)


def _winsorize(df: pd.DataFrame, cols: list, lower: float = 0.005, upper: float = 0.995) -> pd.DataFrame:
    """Clip each feature column to its [0.5, 99.5] percentile range. Some
    ratio-based EEG features (e.g. theta/alpha band-power ratios) blow up to
    absurd magnitudes (~1e11 vs a normal range of ~1-3) when the denominator
    band power is near zero - almost certainly a sensor/computation
    artifact, not a real physiological reading. Left unclipped, a single
    such value dominates that participant's mean under subject centering
    and blows up StandardScaler's variance, which then wrecks MLP training
    on every feature (through the shared hidden layers), not just that one
    column. Must run before centering/scaling, on the whole df (unsupervised,
    feature-only - no label involved, same as the global _usable_columns
    filter already applied)."""
    df = df.copy()
    lo = df[cols].quantile(lower)
    hi = df[cols].quantile(upper)
    df[cols] = df[cols].clip(lower=lo, upper=hi, axis=1)
    return df


def _subject_center(df: pd.DataFrame, cols: list) -> pd.DataFrame:
    """Subtract each participant's own mean from their feature rows, so the
    model sees deviation from that person's baseline instead of absolute
    level - removes "which person is this" variance before the model ever
    sees it. Uses only that participant's own rows (no label, no cross-
    participant leakage), and since train/val participants are disjoint
    (GroupShuffleSplit) this is safe to do once on the full df before
    splitting."""
    df = df.copy()
    df[cols] = df[cols] - df.groupby("ID")[cols].transform("mean")
    return df


def _prepare(merged_path: Path, subject_center: bool):
    df = pd.read_csv(merged_path)
    df = df.dropna(subset=TARGET_COLS)

    teacher_cols = _usable_columns(df, [c for c in df.columns if c.startswith(("bio_", "eeg_"))])
    student_cols = _usable_columns(df, [c for c in df.columns if c.startswith("AU_")])

    df = _winsorize(df, teacher_cols + student_cols)
    if subject_center:
        df = _subject_center(df, teacher_cols + student_cols)
    return df, teacher_cols, student_cols


def _build_split(df: pd.DataFrame, train_idx, val_idx, teacher_cols, student_cols):
    def build(idx):
        sub = df.iloc[idx]
        return (
            sub[teacher_cols].to_numpy(dtype=np.float32),
            sub[student_cols].to_numpy(dtype=np.float32),
            sub[TARGET_COLS].to_numpy(dtype=np.float32),
            sub["ID"].to_numpy(),
        )

    t_tr, s_tr, y_tr, id_tr = build(train_idx)
    t_va, s_va, y_va, id_va = build(val_idx)

    teacher_scaler = StandardScaler().fit(t_tr)
    student_scaler = StandardScaler().fit(s_tr)
    y_scaler = StandardScaler().fit(y_tr)

    train = Split(teacher_scaler.transform(t_tr).astype(np.float32),
                  student_scaler.transform(s_tr).astype(np.float32),
                  y_scaler.transform(y_tr).astype(np.float32), id_tr)
    val = Split(teacher_scaler.transform(t_va).astype(np.float32),
                student_scaler.transform(s_va).astype(np.float32),
                y_scaler.transform(y_va).astype(np.float32), id_va)

    meta = dict(teacher_dim=t_tr.shape[1], student_dim=s_tr.shape[1],
                teacher_cols=teacher_cols, student_cols=student_cols, y_scaler=y_scaler)
    return train, val, meta


def load_splits(merged_path: Path, val_frac: float = 0.2, seed: int = 0, subject_center: bool = True):
    df, teacher_cols, student_cols = _prepare(merged_path, subject_center)
    splitter = GroupShuffleSplit(n_splits=1, test_size=val_frac, random_state=seed)
    train_idx, val_idx = next(splitter.split(df, groups=df["ID"]))
    return _build_split(df, train_idx, val_idx, teacher_cols, student_cols)


def load_folds(merged_path: Path, n_splits: int = 5, subject_center: bool = True):
    """Subject-wise GroupKFold generator - a single train/val split leaves
    only ~11 held-out participants, which is a noisy way to judge whether a
    change (e.g. subject centering) actually helps. Averaging metrics over
    several folds gives a more trustworthy signal on this small dataset."""
    df, teacher_cols, student_cols = _prepare(merged_path, subject_center)
    splitter = GroupKFold(n_splits=n_splits)
    for train_idx, val_idx in splitter.split(df, groups=df["ID"]):
        yield _build_split(df, train_idx, val_idx, teacher_cols, student_cols)
