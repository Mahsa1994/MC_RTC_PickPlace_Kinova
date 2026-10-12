"""
Step 2 follow-up: does a deep multimodal model make a better teacher?

The dataset authors (Bussolan et al. 2025) conclude that the physiological
signals carry information about stress/cognitive load, but that high
accuracy "underscores the need for advanced machine learning approaches and
multi-modal fusion techniques". This script compares, on the exact same
folds/seeds/labels/protocol as train_kd_classify.py's Step 2 teacher sweep:

  1. reference: the previous best teacher - MLP 512-256-128 on the OLD
     teacher features (raw EMG+EEG trial means + centered AUs).
  2. fixed features: the old features silently lacked ALL heart-rate (HRV),
     skin-conductance (EDA) and respiration features. dataset.
     _usable_columns() drops any column with a single NaN, and those are
     missing in ~0.4% of trials. Here they are kept: winsorized, standardized
     on the train fold, missing values imputed to the train mean.
  3. fixed features + within-trial std over windows: a cheap "temporal
     variability" summary, the bar the deep temporal model has to clear.
  4. deep multimodal fusion: one encoder per modality (bio, EEG, AU),
     modality dropout, fused head - on trial-level mean+std features.
  5. deep temporal fusion: per-window bio (60 s windows, ~3.5/trial) and EEG
     (5 s windows, ~45/trial) sequences through residual dilated 1D convs +
     attention pooling, fused with AUs. Sees within-trial dynamics that every
     trial-mean model above throws away.

Physio inputs are not subject-centered (raw was better for the teacher, see
README); AUs are the same centered features the student gets.

Run `python prepare_dataset.py` first (needs data/raw/features/*.csv and
data/merged_trials.csv).
"""
import os

os.environ.setdefault("OMP_NUM_THREADS", "1")
os.environ.setdefault("MKL_NUM_THREADS", "1")

import argparse
import multiprocessing as mp
import warnings
from pathlib import Path

import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import torch.nn.functional as F
from sklearn.metrics import f1_score
from sklearn.model_selection import GroupKFold
from sklearn.preprocessing import OneHotEncoder

from dataset import TARGET_COLS, _prepare, load_folds
from prepare_dataset import normalize_class
from train_distill import tertile_bins
from train_kd_classify import (N_CLASSES, aggregate, cv_ensemble, fit_hgb, fit_logreg, fit_mlp, print_rows,
                               scores, teacher_es_splits)

BIG_MLP = dict(hidden=(512, 256, 128), dropout=0.5)  # best Step 2 MLP in train_kd_classify.py
REFERENCE = "teacher: mlp512-256-128 on old features (EMG+EEG + AU)"
DATA = {}  # filled in main() before the worker pool forks; read-only in workers


# ------------------------------------------------------------------ data

def load_windows(path):
    """Per-window feature table -> {(ID, Class, Repetition): [n_windows, F]}
    in window order. Same unsupervised cleaning as dataset.py, but at window
    level and WITHOUT dropping columns that have a few NaNs: inf -> NaN, drop
    all-NaN/constant columns, winsorize each column to [0.5, 99.5]
    percentiles."""
    df = pd.read_csv(path)
    df["Class"] = normalize_class(df["Class"])
    feat = [c for c in df.columns if c not in ("ID", "Class", "Repetition", "Window")]
    df[feat] = df[feat].replace([np.inf, -np.inf], np.nan)
    std = df[feat].std()
    feat = [c for c in feat if df[c].notna().any() and std[c] > 1e-9]
    df[feat] = df[feat].clip(df[feat].quantile(0.005), df[feat].quantile(0.995), axis=1)
    groups = {key: g.sort_values("Window")[feat].to_numpy(np.float32)
              for key, g in df.groupby(["ID", "Class", "Repetition"])}
    return groups, feat


def trial_windows(groups, df):
    """Windows for each merged-table row: exact (ID, Class, Repetition), else
    Repetition-1 - the same 0- vs 1-indexed fallback prepare_dataset.
    join_with_fallback() used to build the merged table."""
    out = []
    for id_, cls, rep in df[["ID", "Class", "Repetition"]].itertuples(index=False):
        w = groups.get((id_, cls, rep))
        if w is None:
            w = groups.get((id_, cls, rep - 1))
        if w is None:
            raise KeyError(f"no windows for {(id_, cls, rep)}")
        out.append(w)
    return out


def pad(seqs):
    X = np.full((len(seqs), max(len(s) for s in seqs), seqs[0].shape[1]), np.nan, dtype=np.float32)
    for i, s in enumerate(seqs):
        X[i, :len(s)] = s
    return X, np.array([len(s) for s in seqs])


def trial_mean_std(seqs):
    """Trial-level summaries: NaN-aware mean over windows, and std over
    windows (NaN for single-window trials, imputed later)."""
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)  # all-NaN columns in a trial -> NaN, imputed later
        mean = np.stack([np.nanmean(s, axis=0) for s in seqs])
        std = np.stack([np.nanstd(s, axis=0) if len(s) > 1 else np.full(s.shape[1], np.nan) for s in seqs])
    return mean.astype(np.float32), std.astype(np.float32)


def standardize(train, *others):
    """Z-score with train-only NaN-aware stats, then impute NaN to 0 (the
    train mean). Works on [N, F] or padded [N, L, F] arrays (stats over all
    real windows)."""
    flat = train.reshape(-1, train.shape[-1])
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        mu, sd = np.nanmean(flat, axis=0), np.nanstd(flat, axis=0)
    mu, sd = np.nan_to_num(mu), np.where(np.isfinite(sd) & (sd > 1e-9), sd, 1.0)
    return [np.nan_to_num((a - mu) / sd).astype(np.float32) for a in (train,) + others]


class Bundle:
    """Dict of row-aligned arrays that cv_ensemble() can index like one
    array. A key `k` with a companion `k_len` is a padded sequence."""

    def __init__(self, **arrays):
        self.a = arrays

    def __len__(self):
        return len(next(iter(self.a.values())))

    def __getitem__(self, idx):
        return Bundle(**{k: v[idx] for k, v in self.a.items()})


# ------------------------------------------------------------------ deep models

class VecEncoder(nn.Module):
    def __init__(self, in_dim, d, dropout):
        super().__init__()
        self.net = nn.Sequential(nn.Linear(in_dim, 2 * d), nn.GELU(), nn.Dropout(dropout),
                                 nn.Linear(2 * d, d), nn.GELU())
        self.out_dim = d

    def forward(self, x, mask=None):
        return self.net(x)


class SeqEncoder(nn.Module):
    """Per-window projection, two residual dilated 1D-conv blocks (patterns
    across neighbouring windows: ~5-15 s for EEG, minutes for bio), then
    attention pooling + masked mean pooling over the trial."""

    def __init__(self, in_dim, d, dropout):
        super().__init__()
        self.proj = nn.Sequential(nn.Linear(in_dim, d), nn.GELU(), nn.Dropout(dropout))
        self.convs = nn.ModuleList([nn.Conv1d(d, d, 3, padding=dil, dilation=dil) for dil in (1, 2)])
        self.norms = nn.ModuleList([nn.LayerNorm(d) for _ in self.convs])
        self.drop = nn.Dropout(dropout)
        self.score = nn.Linear(d, 1)
        self.out_dim = 2 * d

    def forward(self, x, mask):
        m = mask.unsqueeze(-1).float()
        h = self.proj(x) * m
        for conv, norm in zip(self.convs, self.norms):
            c = conv(h.transpose(1, 2)).transpose(1, 2)
            h = norm(h + self.drop(F.gelu(c))) * m
        w = torch.softmax(self.score(h).squeeze(-1).masked_fill(~mask, -1e9), dim=1).unsqueeze(-1)
        mean = h.sum(1) / m.sum(1).clamp(min=1.0)
        return torch.cat([(w * h).sum(1), mean], dim=1)


class FusionNet(nn.Module):
    """One encoder per modality -> concat -> head. Modality dropout zeroes a
    whole modality's embedding for a random fraction of training samples,
    so the head can't lean on any single signal - the standard regularizer
    for multimodal fusion on small datasets."""

    def __init__(self, encoders, hidden, dropout, modality_dropout):
        super().__init__()
        self.encoders = nn.ModuleDict(encoders)
        self.head = nn.Sequential(nn.Linear(sum(e.out_dim for e in encoders.values()), hidden), nn.GELU(),
                                  nn.Dropout(dropout), nn.Linear(hidden, N_CLASSES))
        self.p = modality_dropout

    def forward(self, batch):
        zs = []
        for name, enc in self.encoders.items():
            z = enc(batch[name], batch.get(name + "_mask"))
            if self.training and self.p > 0:
                z = z * (torch.rand(z.shape[0], 1) > self.p).float() / (1 - self.p)
            zs.append(z)
        return self.head(torch.cat(zs, dim=1))


def make_batch(X, idx, args, rng=None):
    """Tensors for rows idx of Bundle X. Training (rng given): Gaussian input
    noise on every feature, and each sequence randomly cropped to a
    contiguous 50-100% chunk, capped at --max-train-len windows (temporal
    augmentation - the trial label holds for any chunk of the trial)."""
    batch = {}
    for k, v in X.a.items():
        if k.endswith("_len"):
            continue
        if k + "_len" in X.a:
            lens = X.a[k + "_len"][idx]
            starts, crop = np.zeros(len(idx), dtype=int), lens.copy()
            if rng is not None:
                crop = np.maximum(1, np.round(lens * rng.uniform(args.crop_min, 1.0, len(idx)))).astype(int)
                crop = np.minimum(crop, args.max_train_len)
                starts = (rng.uniform(size=len(idx)) * (lens - crop + 1)).astype(int)
            L = int(crop.max())
            out = np.zeros((len(idx), L, v.shape[2]), dtype=np.float32)
            mask = np.zeros((len(idx), L), dtype=bool)
            for i, (row, s, c) in enumerate(zip(idx, starts, crop)):
                out[i, :c] = v[row, s:s + c]
                mask[i, :c] = True
            t = torch.from_numpy(out)
            batch[k + "_mask"] = torch.from_numpy(mask)
        else:
            t = torch.from_numpy(v[idx])
        if rng is not None and args.deep_input_noise > 0:
            t = t + args.deep_input_noise * torch.randn_like(t)
        batch[k] = t
    return batch


def predict_deep(model, X, args):
    model.eval()
    out = []
    with torch.no_grad():
        for idx in np.array_split(np.arange(len(X)), max(1, len(X) // 128)):
            out.append(F.softmax(model(make_batch(X, idx, args)), dim=1).numpy())
    return np.concatenate(out)


def fit_deep(build, X_fit, y_fit, X_es, y_es, seed, args):
    """Mini-batch AdamW, early stopping on held-out macro-F1 (same criterion
    as the MLP teachers)."""
    torch.manual_seed(seed)
    rng = np.random.default_rng(seed)
    model = build()
    opt = torch.optim.AdamW(model.parameters(), lr=args.deep_lr, weight_decay=args.deep_weight_decay)
    best, best_state, since_best = -np.inf, None, 0
    for _ in range(args.deep_max_epochs):
        model.train()
        for idx in np.array_split(rng.permutation(len(y_fit)), max(1, len(y_fit) // args.batch_size)):
            loss = F.cross_entropy(model(make_batch(X_fit, idx, args, rng)), torch.from_numpy(y_fit[idx]).long())
            opt.zero_grad()
            loss.backward()
            opt.step()
        f1 = f1_score(y_es, predict_deep(model, X_es, args).argmax(axis=1), average="macro")
        if f1 > best + 1e-4:
            best, since_best = f1, 0
            best_state = {k: v.clone() for k, v in model.state_dict().items()}
        else:
            since_best += 1
            if since_best >= args.deep_patience:
                break
    model.load_state_dict(best_state)
    return lambda X: predict_deep(model, X, args)


# ------------------------------------------------------------------ one job

def run_job(job):
    target_i, fold_i, seed, args = job
    torch.set_num_threads(1)
    D, fd = DATA, DATA["folds"][fold_i]
    tr, te = fd["tr"], fd["te"]
    y_tr, y_te = tertile_bins(D["y"][tr, target_i], D["y"][te, target_i])
    ids_tr = D["ids"][tr]
    inner = list(GroupKFold(n_splits=args.inner, shuffle=True, random_state=seed).split(tr, groups=ids_tr))
    es = teacher_es_splits(inner, ids_tr, seed)
    out = {}

    # references
    majority = np.zeros((len(y_te), N_CLASSES), dtype=np.float32)
    majority[:, np.bincount(y_tr).argmax()] = 1.0
    out["ref: majority class"] = scores(y_te, majority)
    enc = OneHotEncoder(handle_unknown="ignore", sparse_output=False).fit(D["task"][tr].reshape(-1, 1))
    out["ref: task identity only (LR)"] = scores(y_te, fit_logreg(
        enc.transform(D["task"][tr].reshape(-1, 1)), y_tr)(enc.transform(D["task"][te].reshape(-1, 1))))
    au_tr, au_te = fd["au_tr"], fd["au_te"]
    _, test, _ = cv_ensemble(lambda k, itr, ies: fit_mlp(au_tr[itr], y_tr[itr], au_tr[ies], y_tr[ies], (64, 32),
                                                         0.3, seed * 100 + 50 + k, args), au_tr, au_te, inner)
    out["ref: student MLP 64-32 on AUs (the bar)"] = scores(y_te, test)

    # flat (trial-level) feature sets
    bio_m_tr, bio_m_te = standardize(D["bio_mean"][tr], D["bio_mean"][te])
    eeg_m_tr, eeg_m_te = standardize(D["eeg_mean"][tr], D["eeg_mean"][te])
    bio_s_tr, bio_s_te = standardize(D["bio_std"][tr], D["bio_std"][te])
    eeg_s_tr, eeg_s_te = standardize(D["eeg_std"][tr], D["eeg_std"][te])
    flat = {
        "old features (EMG+EEG + AU)": (fd["old_tr"], fd["old_te"]),
        "full physio": (np.hstack([bio_m_tr, eeg_m_tr]), np.hstack([bio_m_te, eeg_m_te])),
        "full physio + AU": (np.hstack([bio_m_tr, eeg_m_tr, au_tr]), np.hstack([bio_m_te, eeg_m_te, au_te])),
        "full physio mean+std + AU": (np.hstack([bio_m_tr, bio_s_tr, eeg_m_tr, eeg_s_tr, au_tr]),
                                      np.hstack([bio_m_te, bio_s_te, eeg_m_te, eeg_s_te, au_te])),
    }
    plan = [("mlp512-256-128", "old features (EMG+EEG + AU)"),
            ("logreg", "full physio"), ("gboost", "full physio"), ("mlp512-256-128", "full physio"),
            ("logreg", "full physio + AU"), ("gboost", "full physio + AU"), ("mlp512-256-128", "full physio + AU"),
            ("gboost", "full physio mean+std + AU"), ("mlp512-256-128", "full physio mean+std + AU")]
    for model_name, inp in plan:
        X_tr, X_te = flat[inp]
        if model_name == "logreg":
            fit = lambda k, itr, ies, X=X_tr: fit_logreg(X[itr], y_tr[itr], C=0.1)
        elif model_name == "gboost":
            fit = lambda k, itr, ies, X=X_tr: fit_hgb(X[itr], y_tr[itr], seed * 100 + k)
        else:
            fit = lambda k, itr, ies, X=X_tr: fit_mlp(X[es[k][0]], y_tr[es[k][0]], X[es[k][1]], y_tr[es[k][1]],
                                                      BIG_MLP["hidden"], BIG_MLP["dropout"], seed * 100 + k, args,
                                                      input_noise=args.teacher_input_noise)
        _, test, _ = cv_ensemble(fit, X_tr, X_te, inner)
        out[f"teacher: {model_name} on {inp}"] = scores(y_te, test)

    # deep models
    d, drop = args.deep_dim, args.deep_dropout

    def deep(build, X_tr, X_te):
        fit = lambda k, itr, ies: fit_deep(build, X_tr[es[k][0]], y_tr[es[k][0]], X_tr[es[k][1]], y_tr[es[k][1]],
                                           seed * 100 + k, args)
        return scores(y_te, cv_ensemble(fit, X_tr, X_te, inner)[1])

    if "fusion" in args.deep_models:
        Xf_tr = Bundle(bio=np.hstack([bio_m_tr, bio_s_tr]), eeg=np.hstack([eeg_m_tr, eeg_s_tr]), au=au_tr)
        Xf_te = Bundle(bio=np.hstack([bio_m_te, bio_s_te]), eeg=np.hstack([eeg_m_te, eeg_s_te]), au=au_te)
        build = lambda: FusionNet({k: VecEncoder(v.shape[1], d, drop) for k, v in Xf_tr.a.items()},
                                  args.deep_hidden, drop, args.modality_dropout)
        out["teacher: deep fusion (bio | EEG | AU encoders), mean+std"] = deep(build, Xf_tr, Xf_te)

    if "temporal" in args.deep_models:
        bio_tr, bio_te = standardize(D["bio_seq"][tr], D["bio_seq"][te])
        eeg_tr, eeg_te = standardize(D["eeg_seq"][tr], D["eeg_seq"][te])
        Xs_tr = Bundle(bio=bio_tr, bio_len=D["bio_len"][tr], eeg=eeg_tr, eeg_len=D["eeg_len"][tr], au=au_tr)
        Xs_te = Bundle(bio=bio_te, bio_len=D["bio_len"][te], eeg=eeg_te, eeg_len=D["eeg_len"][te], au=au_te)
        build = lambda: FusionNet({"bio": SeqEncoder(bio_tr.shape[2], d, drop),
                                   "eeg": SeqEncoder(eeg_tr.shape[2], d, drop),
                                   "au": VecEncoder(au_tr.shape[1], d, drop)},
                                  args.deep_hidden, drop, args.modality_dropout)
        out["teacher: deep temporal (bio+EEG window sequences | AU)"] = deep(build, Xs_tr, Xs_te)

    return (TARGET_COLS[target_i], fold_i, seed), out, {}


# ------------------------------------------------------------------ main

def build_data(data_dir, n_splits):
    merged = Path(data_dir) / "merged_trials.csv"
    df, _, _ = _prepare(merged, subject_center=False)
    feat_dir = Path(data_dir) / "raw" / "features"
    bio_groups, bio_feat = load_windows(feat_dir / "bio_features_60s.csv")
    eeg_groups, eeg_feat = load_windows(feat_dir / "eeg_features_5s.csv")
    bio_seqs, eeg_seqs = trial_windows(bio_groups, df), trial_windows(eeg_groups, df)
    print(f"bio: {len(bio_feat)} window features (HRV/EDA/RSP/EMG...), {np.mean([len(s) for s in bio_seqs]):.1f} "
          f"windows/trial; EEG: {len(eeg_feat)} features, {np.mean([len(s) for s in eeg_seqs]):.1f} windows/trial")

    data = dict(y=df[TARGET_COLS].to_numpy(np.float32), ids=df["ID"].to_numpy(), task=df["Class"].to_numpy())
    data["bio_mean"], data["bio_std"] = trial_mean_std(bio_seqs)
    data["eeg_mean"], data["eeg_std"] = trial_mean_std(eeg_seqs)
    data["bio_seq"], data["bio_len"] = pad(bio_seqs)
    data["eeg_seq"], data["eeg_len"] = pad(eeg_seqs)

    # Same outer folds as dataset.load_folds() (deterministic GroupKFold on
    # the same rows); old features and centered AUs come straight from it.
    data["folds"] = []
    splits = GroupKFold(n_splits=n_splits).split(df, groups=df["ID"])
    for (tr, te), (tr_c, va_c, _), (tr_r, va_r, meta) in zip(splits, load_folds(merged, n_splits, subject_center=True),
                                                             load_folds(merged, n_splits, subject_center=False)):
        assert ((data["ids"][tr] == tr_c.ids).all() and (data["ids"][te] == va_c.ids).all()
                and np.allclose(meta["y_scaler"].transform(data["y"][te]), va_r.y, atol=1e-5)), "fold misalignment"
        data["folds"].append(dict(tr=tr, te=te, au_tr=tr_c.student_X, au_te=va_c.student_X,
                                  old_tr=np.hstack([tr_r.teacher_X, tr_c.student_X]),
                                  old_te=np.hstack([va_r.teacher_X, va_c.student_X])))
    return data


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", default="data")
    ap.add_argument("--targets", nargs="+", default=list(TARGET_COLS), choices=TARGET_COLS)
    ap.add_argument("--cv", type=int, default=5)
    ap.add_argument("--inner", type=int, default=5)
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2])
    ap.add_argument("--deep-models", nargs="+", default=["fusion", "temporal"], choices=["fusion", "temporal"])
    # MLP settings: identical to train_kd_classify.py defaults
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--weight-decay", type=float, default=1e-2)
    ap.add_argument("--max-epochs", type=int, default=1000)
    ap.add_argument("--patience", type=int, default=100)
    ap.add_argument("--es-metric", choices=["ce", "f1"], default="f1")
    ap.add_argument("--teacher-input-noise", type=float, default=0.5)
    ap.add_argument("--temperature", type=float, default=2.0)  # unused here, fit_mlp reads it
    # deep model settings
    ap.add_argument("--deep-dim", type=int, default=64)
    ap.add_argument("--deep-hidden", type=int, default=128)
    ap.add_argument("--deep-dropout", type=float, default=0.3)
    ap.add_argument("--modality-dropout", type=float, default=0.2)
    ap.add_argument("--deep-input-noise", type=float, default=0.5)
    ap.add_argument("--deep-lr", type=float, default=1e-3)
    ap.add_argument("--deep-weight-decay", type=float, default=1e-2)
    ap.add_argument("--deep-max-epochs", type=int, default=150)
    ap.add_argument("--deep-patience", type=int, default=25)
    ap.add_argument("--batch-size", type=int, default=32)
    ap.add_argument("--crop-min", type=float, default=0.5)
    ap.add_argument("--max-train-len", type=int, default=128)
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    args = ap.parse_args()

    DATA.update(build_data(args.data_dir, args.cv))
    jobs = [(TARGET_COLS.index(t), f, s, args) for t in args.targets for f in range(args.cv) for s in args.seeds]
    print(f"{len(jobs)} jobs on {args.workers} workers")
    with mp.get_context("fork").Pool(args.workers) as pool:
        results = []
        for i, res in enumerate(pool.imap_unordered(run_job, jobs), 1):
            results.append(res)
            print(f"  done {i}/{len(jobs)}: {res[0]}", flush=True)

    for t in args.targets:
        agg = aggregate(results, t, args.cv)
        print(f"\n{'=' * 100}\n{t}: Step 2 teacher comparison, 3-class macro-F1, {args.cv}-fold subject-wise CV x "
              f"{len(args.seeds)} seeds")
        refs = [l for l in agg if l.startswith("ref:")]
        print_rows(agg, refs)
        teachers = sorted([l for l in agg if l.startswith("teacher:")], key=lambda l: -agg[l][0].mean())
        print()
        print_rows(agg, teachers, baseline=REFERENCE, delta_header="dF1 vs old best")


if __name__ == "__main__":
    main()
