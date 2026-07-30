"""
Builds a merged, per-trial feature table from the MultiPhysio-HRC dataset
(Bussolan et al., Robotics 2025, doi.org/10.3390/robotics14120184,
CC-BY-4.0, https://zenodo.org/records/18668043) for a teacher/student
distillation proof of concept: contact-based physiological features
(teacher, "privileged" at training time only) distilled into a facial-
Action-Unit-only student (fully contactless, deployable in real time from
an ordinary webcam).

Downloads only features.zip (~410MB of pre-extracted features), NOT the
15.3GB raw physiological_data.zip - the pre-extracted HRV/EDA/EEG/AU
features are exactly what a teacher/student PoC needs, and moving 15GB
around is neither necessary nor practical here.

Usage:
    python prepare_dataset.py --data-dir ./data
    python prepare_dataset.py --data-dir ./data --skip-download   # if features.zip already present
"""
import argparse
import re
import urllib.request
import zipfile
from pathlib import Path

import numpy as np
import pandas as pd

ZENODO_RECORD = "18668043"
FEATURES_URL = f"https://zenodo.org/api/records/{ZENODO_RECORD}/files/features.zip/content"

# labels.csv uses slightly different Class spelling/casing than the feature
# files, and - more importantly - a different Repetition convention for
# single-instance tasks (labels.csv: Repetition=1; bio/eeg feature files:
# Repetition=0). See README.md "Known data quirks". We lowercase everything
# and let join_with_fallback() retry with Repetition+1 when an exact match
# on (ID, Class, Repetition) isn't found.
CLASS_ALIASES = {
    "vr-job simulator": "vr-job-sim",
}

# aus_data.csv encodes the repetition INSIDE the Class string instead of in
# its Repetition column (e.g. "cobot-task-3", "rest-2") - its own
# Repetition column is a different axis entirely (repeated video segments
# within one trial). See parse_aus_class_repetition().
MULTI_REP_PREFIXES = ("cobot-task", "manual-task", "rest")


def download_features(data_dir: Path) -> Path:
    data_dir.mkdir(parents=True, exist_ok=True)
    zip_path = data_dir / "features.zip"
    if zip_path.exists():
        print(f"[prepare] {zip_path} already exists, skipping download")
        return zip_path
    print(f"[prepare] downloading {FEATURES_URL} (~410MB)...")
    urllib.request.urlretrieve(FEATURES_URL, zip_path)
    return zip_path


def extract_features(zip_path: Path, data_dir: Path) -> Path:
    out_dir = data_dir / "raw"
    marker = out_dir / ".extracted"
    if not marker.exists():
        print(f"[prepare] extracting {zip_path} -> {out_dir}")
        with zipfile.ZipFile(zip_path) as zf:
            zf.extractall(out_dir)
        marker.parent.mkdir(parents=True, exist_ok=True)
        marker.touch()
    candidates = list(out_dir.rglob("labels.csv"))
    if not candidates:
        raise RuntimeError(f"Couldn't find labels.csv under {out_dir}")
    return candidates[0].parent


def normalize_class(series: pd.Series) -> pd.Series:
    s = series.astype(str).str.strip().str.lower()
    return s.replace(CLASS_ALIASES)


def aggregate_windowed(path: Path, prefix: str) -> pd.DataFrame:
    """bio_features_60s.csv / eeg_features_5s.csv already have one row per
    time window - mean-pool across windows to one row per (ID, Class,
    Repetition) so it aligns with one questionnaire label per trial."""
    df = pd.read_csv(path)
    df["Class"] = normalize_class(df["Class"])
    feat_cols = [c for c in df.columns if c not in ("ID", "Class", "Repetition", "Window")]
    agg = df.groupby(["ID", "Class", "Repetition"])[feat_cols].mean()
    agg.columns = [f"{prefix}_{c}" for c in agg.columns]
    return agg.reset_index()


def parse_aus_class_repetition(class_str: str):
    """aus_data.csv puts the repetition number inside the Class string for
    multi-instance tasks ('cobot-task-3' -> ('cobot-task', 3)), unlike
    labels.csv/bio/eeg which keep Class and Repetition as separate columns.
    Parse it into the same shape labels.csv uses so the join lines up."""
    s = str(class_str).strip().lower()
    s = CLASS_ALIASES.get(s, s)
    for prefix in MULTI_REP_PREFIXES:
        m = re.match(rf"^{re.escape(prefix)}-(\d+)$", s)
        if m:
            return prefix, int(m.group(1))
    if s in MULTI_REP_PREFIXES:
        # Bare "rest"/"cobot-task"/"manual-task" with no repetition suffix -
        # an unlabeled/baseline take with no unambiguous labels.csv match.
        # Repetition=-1 never matches, so it's dropped instead of silently
        # averaged into repetition 1's numbers.
        return s, -1
    return s, 1


def aggregate_aus(path: Path, chunksize: int = 200_000) -> pd.DataFrame:
    """aus_data.csv is per-video-frame (1.5M+ rows, no Window column, and
    the single biggest file at ~500MB) - aggregate to mean+std per AU in
    chunks so this doesn't need the whole file in memory at once. This
    pattern also scales to whatever much larger video-derived feature
    table your own trials eventually produce. Groups on the PARSED
    (Class, Repetition) (see parse_aus_class_repetition), not the file's
    own Repetition column, which is a different axis (repeated video
    segments within one trial) that we average over here."""
    sums, sqsums, counts, au_cols = {}, {}, {}, None
    for chunk in pd.read_csv(path, chunksize=chunksize):
        parsed = chunk["Class"].map(parse_aus_class_repetition)
        chunk = chunk.assign(_Class=[p[0] for p in parsed], _Rep=[p[1] for p in parsed])
        if au_cols is None:
            au_cols = [c for c in chunk.columns if c not in ("ID", "Class", "Repetition", "_Class", "_Rep")]
        for key, g in chunk.groupby(["ID", "_Class", "_Rep"]):
            vals = g[au_cols].to_numpy(dtype=np.float64)
            s, sq, n = vals.sum(axis=0), (vals ** 2).sum(axis=0), vals.shape[0]
            if key in sums:
                sums[key] += s
                sqsums[key] += sq
                counts[key] += n
            else:
                sums[key], sqsums[key], counts[key] = s, sq, n

    rows = []
    for key, s in sums.items():
        if key[2] < 0:
            continue  # unmapped baseline take, see parse_aus_class_repetition
        n = counts[key]
        mean = s / n
        var = np.maximum(sqsums[key] / n - mean ** 2, 0.0)
        rows.append(list(key) + list(mean) + list(np.sqrt(var)))
    cols = ["ID", "Class", "Repetition"] + [f"AU_mean_{c}" for c in au_cols] + [f"AU_std_{c}" for c in au_cols]
    return pd.DataFrame(rows, columns=cols)


def join_with_fallback(base: pd.DataFrame, other: pd.DataFrame, name: str) -> pd.DataFrame:
    """Inner join on (ID, Class, Repetition); rows that don't match are
    retried with Repetition+1 on `other` (handles the 0- vs 1-indexed
    single-instance-task mismatch described above). Prints a match-rate
    breakdown rather than silently dropping mismatches."""
    keys = ["ID", "Class", "Repetition"]
    exact = base.merge(other, on=keys, how="inner")

    matched = base.set_index(keys).index.isin(exact.set_index(keys).index)
    remainder = base.loc[~matched].copy()

    other_shifted = other.copy()
    other_shifted["Repetition"] = other_shifted["Repetition"] + 1
    fallback = remainder.merge(other_shifted, on=keys, how="inner")

    merged = pd.concat([exact, fallback], ignore_index=True)
    print(f"[prepare] join with {name}: {len(exact)} exact + {len(fallback)} "
          f"fallback (rep+1) = {len(merged)} matched out of {len(base)} "
          f"label rows ({len(base) - len(merged)} dropped, unmatched)")
    return merged


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", default="data")
    ap.add_argument("--skip-download", action="store_true")
    args = ap.parse_args()

    data_dir = Path(args.data_dir)
    zip_path = (data_dir / "features.zip") if args.skip_download else download_features(data_dir)
    feat_dir = extract_features(zip_path, data_dir)

    labels = pd.read_csv(feat_dir / "labels.csv")
    labels["Class"] = normalize_class(labels["Class"])
    labels = labels.drop_duplicates(subset=["ID", "Class", "Repetition"])

    bio = aggregate_windowed(feat_dir / "bio_features_60s.csv", "bio")
    eeg = aggregate_windowed(feat_dir / "eeg_features_5s.csv", "eeg")
    aus = aggregate_aus(feat_dir / "aus_data.csv")

    merged = join_with_fallback(labels, bio, "bio_features_60s")
    merged = join_with_fallback(merged, eeg, "eeg_features_5s")
    merged = join_with_fallback(merged, aus, "aus_data")

    out_path = data_dir / "merged_trials.csv"
    merged.to_csv(out_path, index=False)
    print(f"[prepare] wrote {len(merged)} trials x {merged.shape[1]} columns -> {out_path}")


if __name__ == "__main__":
    main()
