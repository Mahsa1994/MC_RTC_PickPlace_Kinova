"""
Teacher/student distillation proof of concept on MultiPhysio-HRC.

Teacher: contact-based physiological features (HRV/EDA from bio_features_60s
+ EEG band power from eeg_features_5s) - "privileged", training-time only,
matches what the FRQNT proposal's HR+GSR sensors give.
Student: facial Action Unit features only - fully contactless, matches the
real-time-deployable modality for the Kinova pick-and-place task.
Target: SAM Valence/Arousal/Dominance - the same PAD framework the proposal
already uses for ground truth.

Trains three models to show what distillation actually buys you:
  1. teacher            - upper bound, uses privileged features
  2. student_no_distill - lower bound, AUs only, label loss only
  3. student_distill    - AUs only, label loss + matches the teacher's prediction

Run `python prepare_dataset.py` first to build data/merged_trials.csv.
"""
import argparse
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

from dataset import load_splits


def mlp(in_dim, out_dim=3, hidden=64):
    return nn.Sequential(
        nn.Linear(in_dim, hidden), nn.ReLU(),
        nn.Linear(hidden, hidden), nn.ReLU(),
        nn.Linear(hidden, out_dim),
    )


def train_model(model, X_tr, y_tr, X_va, y_va, extra_targets_tr=None, kd_weight=0.0,
                 epochs=300, lr=1e-3, weight_decay=1e-4):
    opt = torch.optim.Adam(model.parameters(), lr=lr, weight_decay=weight_decay)
    X_tr_t, y_tr_t = torch.tensor(X_tr), torch.tensor(y_tr)
    X_va_t, y_va_t = torch.tensor(X_va), torch.tensor(y_va)
    extra_t = torch.tensor(extra_targets_tr) if extra_targets_tr is not None else None

    best_val, best_state = float("inf"), None
    for _ in range(epochs):
        model.train()
        opt.zero_grad()
        pred = model(X_tr_t)
        loss = nn.functional.mse_loss(pred, y_tr_t)
        if extra_t is not None and kd_weight > 0:
            loss = loss + kd_weight * nn.functional.mse_loss(pred, extra_t)
        loss.backward()
        opt.step()

        model.eval()
        with torch.no_grad():
            val_loss = nn.functional.mse_loss(model(X_va_t), y_va_t).item()
        if val_loss < best_val:
            best_val = val_loss
            best_state = {k: v.clone() for k, v in model.state_dict().items()}

    model.load_state_dict(best_state)
    return model, best_val


def evaluate(model, X, y):
    model.eval()
    with torch.no_grad():
        pred = model(torch.tensor(X)).numpy()
    mae = np.mean(np.abs(pred - y), axis=0)
    ss_res = np.sum((y - pred) ** 2, axis=0)
    ss_tot = np.sum((y - y.mean(axis=0)) ** 2, axis=0)
    r2 = 1 - ss_res / np.clip(ss_tot, 1e-9, None)
    return mae, r2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", default="data")
    ap.add_argument("--epochs", type=int, default=300)
    ap.add_argument("--kd-weight", type=float, default=1.0)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    merged_path = Path(args.data_dir) / "merged_trials.csv"
    train, val, meta = load_splits(merged_path, seed=args.seed)

    print(f"train: {train.teacher_X.shape[0]} trials, val: {val.teacher_X.shape[0]} trials "
          f"(subject-wise split, {len(set(train.ids))} vs {len(set(val.ids))} participants)")
    print(f"teacher_dim={meta['teacher_dim']} (bio+eeg), student_dim={meta['student_dim']} (AUs)")

    # 1. Teacher (privileged features: HRV/EDA/EEG)
    teacher = mlp(meta["teacher_dim"])
    teacher, _ = train_model(teacher, train.teacher_X, train.y, val.teacher_X, val.y, epochs=args.epochs)
    teacher_mae, teacher_r2 = evaluate(teacher, val.teacher_X, val.y)

    with torch.no_grad():
        teacher_train_pred = teacher(torch.tensor(train.teacher_X)).numpy()

    # 2. Student, no distillation (AUs only, label loss only)
    student_plain = mlp(meta["student_dim"])
    student_plain, _ = train_model(student_plain, train.student_X, train.y, val.student_X, val.y, epochs=args.epochs)
    plain_mae, plain_r2 = evaluate(student_plain, val.student_X, val.y)

    # 3. Student, with distillation (AUs only, label loss + teacher match)
    student_kd = mlp(meta["student_dim"])
    student_kd, _ = train_model(student_kd, train.student_X, train.y, val.student_X, val.y,
                                 extra_targets_tr=teacher_train_pred, kd_weight=args.kd_weight,
                                 epochs=args.epochs)
    kd_mae, kd_r2 = evaluate(student_kd, val.student_X, val.y)

    names = ["Valence", "Arousal", "Dominance"]
    print("\n=== Validation MAE (standardized units, lower is better) ===")
    print(f"{'':22s}" + "".join(f"{n:>12s}" for n in names))
    print(f"{'teacher (privileged)':22s}" + "".join(f"{v:12.3f}" for v in teacher_mae))
    print(f"{'student, no KD':22s}" + "".join(f"{v:12.3f}" for v in plain_mae))
    print(f"{'student, with KD':22s}" + "".join(f"{v:12.3f}" for v in kd_mae))

    print("\n=== Validation R^2 (higher is better) ===")
    print(f"{'':22s}" + "".join(f"{n:>12s}" for n in names))
    print(f"{'teacher (privileged)':22s}" + "".join(f"{v:12.3f}" for v in teacher_r2))
    print(f"{'student, no KD':22s}" + "".join(f"{v:12.3f}" for v in plain_r2))
    print(f"{'student, with KD':22s}" + "".join(f"{v:12.3f}" for v in kd_r2))


if __name__ == "__main__":
    main()
