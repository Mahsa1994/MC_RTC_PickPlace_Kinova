# MultiPhysio-HRC teacher/student proof of concept

Proof of concept for the personalization-via-distillation idea, run against
[MultiPhysio-HRC](https://automation-robotics-machines.github.io/MultiPhysio-HRC.github.io/)
(Bussolan et al., *Robotics* 2025, doi:10.3390/robotics14120184, CC-BY-4.0,
[Zenodo record](https://zenodo.org/records/18668043)) instead of the
project's own (not-yet-collected) trial data, to de-risk the ML pipeline
before spending any participant sessions on it.

**Teacher** (privileged, contact-based, training-time only): HRV/EDA
features (`bio_features_60s.csv`) + EEG band-power features
(`eeg_features_5s.csv`) - the closest analogue this dataset has to the
FRQNT proposal's HR+GSR sensors.
**Student** (contactless, real-time-deployable): facial Action Unit
features only (`aus_data.csv`), aggregated to mean+std per trial.
**Target**: SAM Valence/Arousal/Dominance (`labels.csv`) - the same PAD
framework the proposal already uses for ground truth.

## Running it

```
pip install -r requirements.txt
python prepare_dataset.py --data-dir ./data   # downloads features.zip (~410MB), builds data/merged_trials.csv
python train_distill.py --data-dir ./data
```

`prepare_dataset.py` only downloads `features.zip` (pre-extracted features),
**not** the 15.3GB raw `physiological_data.zip` - not needed for this PoC.
`--skip-download` reuses an already-downloaded `data/features.zip`.

## What this validated

The pipeline runs end-to-end on the real dataset: download -> per-trial
feature aggregation -> subject-wise train/val split -> train a teacher MLP
on bio+EEG features -> train a student MLP on AU features only, with and
without a distillation loss (student matches the teacher's prediction, not
just the label) -> report validation MAE/R² for all three.

Current numbers (300 epochs, seed 0, 546 train / 152 val trials, 40 vs 11
participants):

|                       | Valence R² | Arousal R² | Dominance R² |
|-----------------------|-----------:|-----------:|-------------:|
| teacher (privileged)  |     -0.002 |     -0.041 |       -0.059 |
| student, no KD        |      0.013 |      0.048 |        0.013 |
| student, with KD      |      0.021 |      0.030 |        0.036 |

**Read this as "the plumbing works," not "distillation helps."** R² near
zero across the board - including the *teacher* - means none of these
models beat predicting the mean Valence/Arousal/Dominance for every trial.
That's a data/modeling problem to fix before drawing any conclusion about
distillation, not a negative result about the idea itself.

## Known data quirks (handled in `prepare_dataset.py`)

- `labels.csv` splits `Class` and `Repetition` into separate columns, with
  `Repetition=1` for single-instance tasks (Stroop, N-back, MAT, Hanoi,
  meditation, VR). `bio_features_60s.csv`/`eeg_features_5s.csv` use the same
  split but `Repetition=0` for those same single-instance tasks - handled by
  retrying the join with `Repetition+1` (`join_with_fallback`).
- `aus_data.csv` instead encodes the repetition **inside** the `Class`
  string for multi-instance tasks (`"cobot-task-3"`, `"rest-2"`), and its
  own `Repetition` column is a different axis (repeated video segments
  within one trial, averaged over) - handled by
  `parse_aus_class_repetition()`.
- Even after fixing both of the above, ~14% of AU trials and ~2% of
  bio/eeg trials still don't find a match and get dropped (printed at
  prepare time) - likely genuine missing data (`participants_task_overview.csv`
  shows plenty of "-" cells per participant), not a further join bug, but
  worth spot-checking if you extend this.

## Why the numbers are weak right now, and what to try next

This was scoped as "get a real pipeline running on real data," not
"tune a model" - the weak numbers are the expected state of an untuned
first pass, not evidence against the approach. In rough priority order:

1. **Per-participant normalization.** The proposal itself already says
   individual comfort/physiological baselines vary a lot - right now every
   feature is globally standardized, so a chunk of the variance an MLP has
   to explain is just "which person is this" rather than "how do they feel
   right now." Z-scoring each participant's features against their own
   baseline (or adding participant ID as a fixed/random effect) is the
   single most likely fix, and it's also a direct, concrete first step
   toward the personalization-via-distillation direction you want to
   pursue next - this PoC is a natural base to build that on.
2. **Model/data ratio.** 91 teacher features and 546 training trials from
   only 40 participants is a lot of dimensionality for a 2-layer MLP with no
   regularization search - try stronger weight decay, fewer hidden units, or
   a linear/ridge baseline first to get a sane reference point before
   trusting any MLP number.
3. **Regression vs. classification.** Self-reported Valence/Arousal/
   Dominance on a small ordinal scale is noisy; a coarser low/mid/high
   classification target may show a cleaner signal than exact regression
   before you conclude the features themselves are uninformative.
4. **Check label variance per participant.** If some participants barely
   move on the SAM scale across tasks, that's little for any model to learn
   from regardless of features.

None of this needs new data collection - it's all reruns of
`train_distill.py` / `dataset.py` against the same `data/merged_trials.csv`.

## Relationship to your own trial data

This dataset is a *disassembly* task with voice commands and no adaptive
robot behavior - it's useful for de-risking the perception/distillation
pipeline, but it can't substitute for your own trials, since your actual
research question (does adapting motion to inferred comfort improve
comfort) needs a robot that's actually closing the loop, which no existing
dataset has. Once the `pick_and_place` branch's compliant states are
collecting real trial data, point `dataset.py` at that instead - the
teacher/student/distillation code doesn't change, only what feeds it does.
