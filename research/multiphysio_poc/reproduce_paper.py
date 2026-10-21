"""
Reproduces the MultiPhysio-HRC paper's baselines (Bussolan et al., Robotics
2025 / arXiv 2510.00703, Tables I-II) from the released features, then tests
whether they can be beaten under the same protocol.

The authors' repo (github.com/automation-robotics-machines/MultiPhysio-HRC)
only publishes feature extraction. The baseline notebooks/scripts its README
lists are not in it, so the protocol below is reconstructed from the paper:

- targets: STAI-Y1 (stress) and NASA-TLX (cognitive load) from labels.csv
- samples: one row per feature window, labelled with its trial's score:
  physio = 60 s windows (bio_features_60s.csv, paper n=250 features),
  EEG = 5 s windows (eeg_features_5s.csv, n=88), voice = one row per speech
  segment (speech_features.csv, n=439)
- features AND labels min-max normalized within each subject (the subject's
  own min/max, test subject included, as the paper states)
- 3 classes per subject: Low < mu - sd/2 <= Medium <= mu + sd/2 < High, where
  mu/sd are that subject's mean/std score over all its tasks
- RandomForest / AdaBoost / XGBoost with out-of-the-box hyperparameters
- Leave-One-Subject-Out; metric per held-out subject, then mean +/- std over
  subjects. Regression: RMSE on the normalized label. Classification: "F1";
  the averaging is unstated, so macro-F1 is reported (weighted-F1 also kept).

Unstated in the paper, chosen here: inf/NaN features imputed with the
training-fold median; random_state=0; every labelled trial included.

On top of the reproduction, same rows / same metric:
  - reference baselines: predict the training mean / majority class, and
    task identity alone (one-hot task name -> same model). This shows how
    much of each score is just "which task was this".
  - trial smoothing: average a model's window predictions over each test
    trial (labels are per trial anyway). Uses only test inputs, no labels.
  - late fusion: average the trial-smoothed predictions of the physio, EEG
    and voice models.
  - early fusion: one model on per-trial feature means of physio + EEG +
    voice + facial AUs, its trial prediction broadcast back to the rows.
Fusion rows are scored on the physio windows, the paper's headline modality.

Needs xgboost (see README "Reproducing the dataset paper").
"""
import os

os.environ.setdefault("OMP_NUM_THREADS", "1")

import argparse
import json
import multiprocessing as mp
import pickle
import sys
import time
import warnings
from pathlib import Path

import numpy as np
import pandas as pd
from sklearn.ensemble import (AdaBoostClassifier, AdaBoostRegressor, RandomForestClassifier,
                              RandomForestRegressor)
from sklearn.metrics import f1_score
from xgboost import XGBClassifier, XGBRegressor

from prepare_dataset import normalize_class

TARGETS = {"STAI": "stress", "NASA": "cognitive load"}
MODALITIES = {  # name -> (file, metadata columns that aren't features)
    "physio": ("bio_features_60s.csv", ["ID", "Class", "Repetition", "Window"]),
    "eeg": ("eeg_features_5s.csv", ["ID", "Class", "Repetition", "Window"]),
    "voice": ("speech_features.csv", ["ID", "Class", "Repetition", "SpokenSegment#", "Filename",
                                      "Start_Timestamp", "End_Timestamp", "Phrase"]),
}
MODELS = ("RF", "AB", "XGB")

# Paper Tables I-II (arXiv 2510.00703v1), mean over LOSO subjects.
PAPER = {
    ("STAI", "reg"): {"physio": (0.20, 0.20, 0.23), "eeg": (0.32, 0.30, 0.32), "voice": (0.32, 0.33, 0.34)},
    ("NASA", "reg"): {"physio": (0.19, 0.19, 0.20), "eeg": (0.31, 0.29, 0.32), "voice": (0.32, 0.32, 0.33)},
    ("STAI", "cls"): {"physio": (0.30, 0.33, 0.329), "eeg": (0.37, 0.34, 0.37), "voice": (0.35, 0.34, 0.36)},
    ("NASA", "cls"): {"physio": (0.39, 0.38, 0.38), "eeg": (0.39, 0.38, 0.40), "voice": (0.41, 0.37, 0.38)},
}

DATA = {}  # filled in main() before the worker pool forks; read-only in workers


# ------------------------------------------------------------------ data

def load_labels(feat_dir):
    """One row per trial with per-subject normalized scores and classes."""
    lab = pd.read_csv(feat_dir / "labels.csv")
    lab["Class"] = normalize_class(lab["Class"])
    lab = lab.drop_duplicates(subset=["ID", "Class", "Repetition"]).reset_index(drop=True)
    for t in TARGETS:
        g = lab.groupby("ID")[t]
        lo, hi = g.transform("min"), g.transform("max")
        lab[f"{t}_norm"] = ((lab[t] - lo) / (hi - lo).replace(0, np.nan)).fillna(0.0)
        mu, sd = g.transform("mean"), g.transform("std").fillna(0.0)
        lab[f"{t}_cls"] = np.where(lab[t] < mu - sd / 2, 0, np.where(lab[t] > mu + sd / 2, 2, 1))
    return lab


def attach_trials(feat, lab):
    """Label-driven join, same convention as prepare_dataset.join_with_fallback:
    a labels.csv trial (ID, Class, rep) takes the feature rows with the same
    rep, else rep-1 (feature files index single-run tasks from 0, labels from 1).
    Exact matches are claimed first so no feature rows get two labels."""
    keys = set(feat[["ID", "Class", "Repetition"]].drop_duplicates().itertuples(index=False, name=None))
    claimed, pairs = set(), []
    trial_keys = list(lab[["ID", "Class", "Repetition"]].itertuples(index=False, name=None))
    for t, k in enumerate(trial_keys):
        if k in keys:
            claimed.add(k)
            pairs.append(k + (t,))
    exact = {p[3] for p in pairs}
    for t, (id_, cls, rep) in enumerate(trial_keys):
        k = (id_, cls, rep - 1)
        if t not in exact and k in keys and k not in claimed:
            claimed.add(k)
            pairs.append(k + (t,))
    m = pd.DataFrame(pairs, columns=["ID", "Class", "Repetition", "trial"])
    return feat.merge(m, on=["ID", "Class", "Repetition"], how="inner")


def subject_minmax(df, cols):
    """Min-max each feature within each subject (paper protocol); constant -> 0."""
    g = df.groupby("ID")[cols]
    lo, hi = g.transform("min"), g.transform("max")
    out = (df[cols] - lo) / (hi - lo).where(hi > lo)
    return out.where(hi > lo, 0.0).where(df[cols].notna())


def load_modality(feat_dir, name, lab):
    path, meta = MODALITIES[name]
    df = pd.read_csv(feat_dir / path)
    df["Class"] = normalize_class(df["Class"])
    cols = [c for c in df.columns if c not in meta]
    df[cols] = df[cols].apply(pd.to_numeric, errors="coerce").replace([np.inf, -np.inf], np.nan)
    std = df[cols].std()
    cols = [c for c in cols if df[c].notna().any() and std[c] > 1e-12]
    df = attach_trials(df[["ID", "Class", "Repetition"] + cols], lab)
    X = subject_minmax(df, cols).to_numpy(np.float32)
    return dict(X=X, cols=cols, trial=df["trial"].to_numpy(), ids=df["ID"].to_numpy(),
                task=df["Class"].to_numpy(), raw=df[["ID", "trial"] + cols])


def trial_table(mods, lab, merged_path):
    """Early-fusion input: per-trial means of each modality's (raw) window
    features, plus facial AU mean/std per trial from merged_trials.csv, all
    min-max normalized within subject. NaN where a modality is missing."""
    blocks = []
    for name, m in mods.items():
        agg = m["raw"].groupby("trial")[m["cols"]].mean()
        agg.columns = [f"{name}:{c}" for c in agg.columns]
        blocks.append(agg)
    merged = pd.read_csv(merged_path)
    au_cols = [c for c in merged.columns if c.startswith("AU_")]
    au = lab[["ID", "Class", "Repetition"]].reset_index().merge(
        merged[["ID", "Class", "Repetition"] + au_cols], on=["ID", "Class", "Repetition"], how="inner")
    blocks.append(au.set_index("index")[au_cols].rename(columns=lambda c: f"au:{c}"))
    tab = pd.concat(blocks, axis=1).reindex(range(len(lab)))
    tab = tab[tab.notna().any(axis=1)]
    cols = list(tab.columns)
    tab["ID"] = lab.loc[tab.index, "ID"].to_numpy()
    X = subject_minmax(tab, cols).to_numpy(np.float32)
    return dict(X=X, cols=cols, trial=tab.index.to_numpy(), ids=tab["ID"].to_numpy(),
                task=lab.loc[tab.index, "Class"].to_numpy())


# ------------------------------------------------------------------ models / metrics

def make_model(name, kind):
    if name == "RF":
        return (RandomForestRegressor if kind == "reg" else RandomForestClassifier)(random_state=0, n_jobs=1)
    if name == "AB":
        return (AdaBoostRegressor if kind == "reg" else AdaBoostClassifier)(random_state=0)
    if kind == "reg":
        return XGBRegressor(random_state=0, n_jobs=1)
    return XGBClassifier(random_state=0, n_jobs=1)


def impute(X_tr, X_te):
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        med = np.nanmedian(X_tr, axis=0)
    med = np.nan_to_num(med)
    return np.where(np.isnan(X_tr), med, X_tr), np.where(np.isnan(X_te), med, X_te)


def fit_predict(model_name, kind, X_tr, y_tr, X_te):
    """Regression -> predicted value per row; classification -> [n, 3] probs."""
    X_tr, X_te = impute(X_tr, X_te)
    if kind == "reg":
        return make_model(model_name, kind).fit(X_tr, y_tr).predict(X_te).astype(np.float32)
    present = np.unique(y_tr)  # xgboost needs contiguous labels
    remap = {c: i for i, c in enumerate(present)}
    model = make_model(model_name, kind).fit(X_tr, np.vectorize(remap.get)(y_tr))
    proba = np.zeros((len(X_te), 3), dtype=np.float32)
    proba[:, present] = model.predict_proba(X_te)
    return proba


def smooth_by_trial(pred, trial):
    """Average predictions (values or probabilities) over each trial's rows."""
    df = pd.DataFrame(pred.reshape(len(pred), -1))
    return df.groupby(trial).transform("mean").to_numpy(np.float32).reshape(pred.shape)


def subject_score(kind, y, pred):
    if kind == "reg":
        return float(np.sqrt(np.mean((pred - y) ** 2))), None
    p = pred.argmax(axis=1)
    return f1_score(y, p, average="macro"), f1_score(y, p, average="weighted")


# ------------------------------------------------------------------ jobs

def labels_for(source, target, kind):
    lab = DATA["lab"]
    col = f"{target}_norm" if kind == "reg" else f"{target}_cls"
    return lab[col].to_numpy()[DATA[source]["trial"]]


def run_job(job):
    """One LOSO fold for one (rows source, feature set, target, kind, model).
    Returns the held-out subject's predictions (raw and trial-smoothed)."""
    source, features, target, kind, model_name, subject = job
    d = DATA[source]
    y = labels_for(source, target, kind)
    te = d["ids"] == subject
    tr = ~te
    if features == "task":
        tasks = np.unique(d["task"])
        X = (d["task"][:, None] == tasks[None, :]).astype(np.float32)
    else:
        X = d["X"]
    if kind == "reg":
        base = {"constant": np.full(te.sum(), y[tr].mean(), dtype=np.float32)}
    else:
        maj = np.zeros((te.sum(), 3), dtype=np.float32)
        maj[:, np.bincount(y[tr], minlength=3).argmax()] = 1.0
        base = {"constant": maj}
    pred = fit_predict(model_name, kind, X[tr], y[tr], X[te]) if model_name != "none" else base["constant"]
    return job, dict(trial=d["trial"][te], y=y[te], pred=pred, smooth=smooth_by_trial(pred, d["trial"][te]))


# ------------------------------------------------------------------ main

def per_subject(kind, rows):
    """rows: list of (subject, y, pred) -> (mean, std) over subjects of the metric (+ weighted F1 mean)."""
    s = [subject_score(kind, y, p) for _, y, p in rows]
    main = np.array([a for a, _ in s])
    extra = np.array([b for _, b in s]) if kind == "cls" else None
    return main.mean(), main.std(), (extra.mean() if extra is not None else None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", default="data")
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    ap.add_argument("--out", default=None, help="optional JSON dump of every table cell")
    ap.add_argument("--max-subjects", type=int, default=None,
                    help="only hold out the first N subjects per source (quick smoke test, not comparable)")
    ap.add_argument("--cache", default="results/reproduce_paper_fits.pkl",
                    help="every finished LOSO fit is appended here; a rerun skips fits already in it "
                         "(the full run takes ~2 h, mostly the default RF regressor on EEG/voice)")
    args = ap.parse_args()
    sys.stdout.reconfigure(line_buffering=True)

    feat_dir = Path(args.data_dir) / "raw" / "features"
    lab = load_labels(feat_dir)
    DATA["lab"] = lab
    for name in MODALITIES:
        DATA[name] = load_modality(feat_dir, name, lab)
        d = DATA[name]
        print(f"{name}: {len(d['cols'])} features (paper n="
              f"{ {'physio': 250, 'eeg': 88, 'voice': 439}[name] }), {len(d['X'])} rows, "
              f"{len(np.unique(d['trial']))} trials, {len(np.unique(d['ids']))} subjects")
    DATA["trialfusion"] = trial_table({m: DATA[m] for m in MODALITIES}, lab, Path(args.data_dir) / "merged_trials.csv")
    print(f"early-fusion trial table: {len(DATA['trialfusion']['cols'])} features, "
          f"{len(DATA['trialfusion']['X'])} trials")
    for t in TARGETS:
        print(f"{t} class balance (low/med/high, per-subject mu +/- sd/2): "
              f"{np.bincount(lab[f'{t}_cls'], minlength=3).tolist()}")

    jobs = []
    for source in MODALITIES:
        subjects = np.unique(DATA[source]["ids"])[:args.max_subjects]
        for target in TARGETS:
            for kind in ("reg", "cls"):
                for s in subjects:
                    jobs.append((source, "features", target, kind, "none", s))
                    for m in MODELS:
                        jobs.append((source, "features", target, kind, m, s))
                    jobs.append((source, "task", target, kind, "RF", s))
    for target in TARGETS:
        for kind in ("reg", "cls"):
            for s in np.unique(DATA["trialfusion"]["ids"])[:args.max_subjects]:
                for m in ("RF", "XGB"):
                    jobs.append(("trialfusion", "features", target, kind, m, s))
    # biggest jobs first so the pool doesn't finish on a long tail
    cost = {"eeg": 40, "voice": 30, "physio": 3, "trialfusion": 1}
    jobs.sort(key=lambda j: -(cost[j[0]] * (3 if j[4] == "RF" and j[1] == "features" else 1)))
    results = load_cache(args.cache)
    todo = [j for j in jobs if j not in results]
    print(f"{len(jobs)} LOSO fits, {len(jobs) - len(todo)} cached in {args.cache}, "
          f"{len(todo)} to run on {args.workers} workers")
    t0 = time.time()
    if todo:
        Path(args.cache).parent.mkdir(parents=True, exist_ok=True)
        with mp.get_context("fork").Pool(args.workers) as pool, open(args.cache, "ab") as cache:
            for i, (job, res) in enumerate(pool.imap_unordered(run_job, todo, chunksize=2), 1):
                results[job] = res
                pickle.dump((job, res), cache)
                cache.flush()
                if i % 200 == 0 or i == len(todo):
                    print(f"  {i}/{len(todo)} fits done ({time.time() - t0:.0f}s)")

    report({j: results[j] for j in jobs}, args)


def load_cache(path):
    """{job: result} from the append-only pickle stream written by main();
    a truncated last record (killed mid-write) is ignored."""
    results = {}
    if Path(path).exists():
        with open(path, "rb") as f:
            while True:
                try:
                    job, res = pickle.load(f)
                except (EOFError, pickle.UnpicklingError):
                    break
                results[job] = res
    return results


def collect(results, source, features, target, kind, model, smoothed):
    rows = []
    for (src, feat, tg, kd, m, s), r in results.items():
        if (src, feat, tg, kd, m) == (source, features, target, kind, model):
            rows.append((s, r["y"], r["smooth"] if smoothed else r["pred"]))
    return rows


def physio_rows_from_trials(target, kind, trial_pred):
    """Score per-trial predictions on the physio windows (paper headline rows)."""
    d, y = DATA["physio"], labels_for("physio", target, kind)
    rows = []
    for s in np.unique(d["ids"]):
        idx = np.where(d["ids"] == s)[0]
        keep = [i for i in idx if d["trial"][i] in trial_pred]
        if keep:
            rows.append((s, y[keep], np.stack([trial_pred[d["trial"][i]] for i in keep])))
    return rows


def report(results, args):
    dump = {}
    fmt = lambda m, s: f"{m:.3f} +/- {s:.3f}"
    for target, desc in TARGETS.items():
        for kind in ("reg", "cls"):
            metric = "RMSE (lower is better)" if kind == "reg" else "macro-F1 (higher is better)"
            print(f"\n{'=' * 104}\n{target} ({desc}) - {'regression' if kind == 'reg' else '3-class'}: "
                  f"{metric}, LOSO mean +/- std over subjects")
            print(f"  {'rows / model':34s}{'paper':>9s}{'reproduced':>20s}{'+ trial smoothing':>22s}"
                  + (f"{'weighted-F1':>13s}" if kind == "cls" else ""))
            for source in MODALITIES:
                for mi, model in enumerate(MODELS):
                    raw = per_subject(kind, collect(results, source, "features", target, kind, model, False))
                    sm = per_subject(kind, collect(results, source, "features", target, kind, model, True))
                    paper = PAPER[(target, kind)][source][mi]
                    print(f"  {source + ' / ' + model:34s}{paper:9.3f}{fmt(*raw[:2]):>20s}{fmt(*sm[:2]):>22s}"
                          + (f"{raw[2]:13.3f}" if kind == "cls" else ""))
                    dump[f"{target}/{kind}/{source}/{model}"] = dict(paper=paper, raw=raw[:2], smoothed=sm[:2])
                const = per_subject(kind, collect(results, source, "features", target, kind, "none", False))
                task = per_subject(kind, collect(results, source, "task", target, kind, "RF", False))
                print(f"  {source + ' / train mean or majority':34s}{'':9s}{fmt(*const[:2]):>20s}")
                print(f"  {source + ' / task identity only (RF)':34s}{'':9s}{fmt(*task[:2]):>20s}")
                dump[f"{target}/{kind}/{source}/constant"] = const[:2]
                dump[f"{target}/{kind}/{source}/task_identity"] = task[:2]

            print(f"  -- fusion, scored on the physio windows (compare with physio rows above) --")
            for model in MODELS:
                # late fusion: mean over modalities of each trial's smoothed prediction
                per_mod = {}
                for (src, feat, tg, kd, m, s), r in results.items():
                    if feat == "features" and m == model and tg == target and kd == kind and src in MODALITIES:
                        for tr_id, p in zip(r["trial"], r["smooth"]):
                            per_mod.setdefault(tr_id, {})[src] = p
                trial_pred = {t: np.mean(list(v.values()), axis=0) for t, v in per_mod.items()}
                late = per_subject(kind, physio_rows_from_trials(target, kind, trial_pred))
                print(f"  {'late fusion physio+EEG+voice / ' + model:34s}{'':9s}{'':>20s}{fmt(*late[:2]):>22s}")
                dump[f"{target}/{kind}/late_fusion/{model}"] = late[:2]
            for model in ("RF", "XGB"):
                trial_pred = {}
                for (src, feat, tg, kd, m, s), r in results.items():
                    if src == "trialfusion" and m == model and tg == target and kd == kind:
                        trial_pred.update(zip(r["trial"], r["pred"]))
                early = per_subject(kind, physio_rows_from_trials(target, kind, trial_pred))
                print(f"  {'early fusion +AU (trial) / ' + model:34s}{'':9s}{fmt(*early[:2]):>20s}")
                dump[f"{target}/{kind}/early_fusion/{model}"] = early[:2]

    if args.out:
        Path(args.out).write_text(json.dumps(dump, indent=1, default=float))
        print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
