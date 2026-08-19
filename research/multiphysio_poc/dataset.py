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
from sklearn.model_selection import GroupShuffleSplit
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


def load_splits(merged_path: Path, val_frac: float = 0.2, seed: int = 0):
    df = pd.read_csv(merged_path)
    df = df.dropna(subset=TARGET_COLS)

    teacher_cols = _usable_columns(df, [c for c in df.columns if c.startswith(("bio_", "eeg_"))])
    student_cols = _usable_columns(df, [c for c in df.columns if c.startswith("AU_")])

    splitter = GroupShuffleSplit(n_splits=1, test_size=val_frac, random_state=seed)
    train_idx, val_idx = next(splitter.split(df, groups=df["ID"]))

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
