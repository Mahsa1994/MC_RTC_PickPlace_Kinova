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
python train_distill.py --data-dir ./data --cv 5 --classify
```

`prepare_dataset.py` only downloads `features.zip` (pre-extracted features),
**not** the 15.3GB raw `physiological_data.zip` - not needed for this PoC.
`--skip-download` reuses an already-downloaded `data/features.zip`.

`train_distill.py` flags worth knowing about:
- `--cv N`: subject-wise `GroupKFold` with `N` folds, mean±std across folds
  (recommended - a single split only holds out ~11 participants, too noisy
  to trust on its own). Omit for a quick single-split run.
- `--classify`: also reports tertile (low/mid/high) classification
  macro-F1 alongside the regression metrics.
- `--no-subject-center`: disable per-participant feature centering (on by
  default) to A/B against it.
- `--hidden`, `--weight-decay`: MLP width and L2 regularization.

`python pca_analysis.py --data-dir ./data` runs a PCA sanity check (see
"PCA feature-selection check" below): explained-variance curve, top feature
loadings per component, and full-dim vs. PCA-reduced performance via the
same CV harness.

## What this validated

The pipeline runs end-to-end on the real dataset: download -> per-trial
feature aggregation -> subject-wise train/val split -> train a teacher MLP
on bio+EEG features -> train a student MLP on AU features only, with and
without a distillation loss (student matches the teacher's prediction, not
just the label) -> report validation MAE/R² for all three.

## Tuning pass (2026-08-19)

Went through the "why the numbers are weak" list below in priority order:
one numerical bug found along the way, one item confirmed to help, one
confirmed but small, one promising lead, and one ruled out.

- **Bug found and fixed: an EEG feature was corrupting training.**
  `eeg_ThF3/AlP3` (a theta/alpha band-power ratio) had one participant with
  a value of `3.7e11` against a normal range of `1-3` - almost certainly a
  near-zero-denominator artifact, not a real reading. Uncaught, it dominated
  that participant's per-trial mean and blew up `StandardScaler`'s variance,
  which wrecked MLP training through the shared hidden layers (validation R²
  as low as **-966,054**, not just on that feature). Fixed with
  `dataset._winsorize()`: clip every teacher/student feature to its
  [0.5, 99.5] percentile range before centering/scaling. Worth remembering
  if you add more ratio-style features later.
- **Per-participant normalization (item 1) helps, confirmed with proper
  evaluation.** A single train/val split only holds out ~11 participants,
  too noisy to trust a before/after comparison on - `dataset.load_folds()`
  now does subject-wise 5-fold `GroupKFold` instead, and `train_distill.py
  --cv 5` reports mean±std across folds. With that, subject-centering
  (`dataset._subject_center()`, on by default) gives a consistent R² lift,
  most visible on the noise-free ridge baseline (teacher ridge Dominance R²:
  -0.011 centered vs -0.257 uncentered). It's on by default now
  (`--no-subject-center` to disable for comparison).
- **Model/data ratio (item 2): tried, small win.** Added a `RidgeCV` linear
  baseline alongside the MLP (`ridge_baseline()`) - it matches or beats the
  MLP most of the time, confirming the MLP wasn't buying much over a linear
  model at this train-set size. Swept hidden width and weight decay
  (`--hidden`, `--weight-decay`); `hidden=32, weight_decay=0.1` (new
  defaults, was `hidden=64, weight_decay=1e-4`) edged out other combinations
  but the gap was within fold-to-fold noise - regularization was not the
  bottleneck here.
- **Regression vs. classification (item 3): promising, biggest lift so
  far.** `train_distill.py --classify` bins Valence/Arousal/Dominance into
  train-quantile tertiles (low/mid/high) and reports macro-F1 via
  `LogisticRegression` against a majority-class dummy baseline over the same
  5 folds. Arousal in particular shows real signal: **0.545 macro-F1 vs.
  0.402 majority-class baseline** (student, AUs only). Valence and Dominance
  also beat the baseline but by less. This is the first result in this PoC
  that's clearly above chance - the near-zero regression R² was undersold by
  the metric, not just the model.
- **Label variance per trial (item 4): not the bottleneck.** Checked
  within-participant SAM std (mean ≈0.75) against between-participant std of
  per-participant means (≈0.6-0.68, all three targets) - within-participant
  variance is *larger*, so participants do move meaningfully across trials
  and there's real signal for a within-person model to learn from. No
  participant has near-zero variance on all three targets. Ruled out as an
  explanation for the weak regression numbers.

Current CV numbers (5-fold subject-wise `GroupKFold`, `hidden=32`,
`weight_decay=0.1`, subject-centered, `python train_distill.py --cv 5
--classify`):

|                       | Valence R² | Arousal R² | Dominance R² |
|-----------------------|-----------:|-----------:|-------------:|
| teacher, MLP          |     -0.048 |      0.019 |         0.011|
| teacher, ridge        |     -0.062 |      0.008 |        -0.011|
| student, no KD, MLP   |     -0.024 |      0.062 |         0.012|
| student, no KD, ridge |     -0.029 |      0.049 |         0.008|
| student, with KD, MLP |     -0.024 |      0.052 |         0.018|

|          | teacher F1 | student F1 | majority-class F1 |
|----------|-----------:|-----------:|-------------------:|
| Valence  |      0.448 |      0.459 |               0.405 |
| Arousal  |      0.532 |      0.545 |               0.402 |
| Dominance|      0.351 |      0.305 |               0.241 |

**Read this as "the plumbing works and a couple of real levers are
identified," not "distillation helps."** Regression R² is still near zero
for Valence in particular - subject centering and the winsorization fix
were correctness/robustness improvements, not the fix that gets R² solidly
positive. The clearest positive result is that coarse classification beats
chance meaningfully (Arousal especially); that's the strongest lead to
pull on next, not further regression tuning.

## PCA feature-selection check (2026-08-19)

Follow-up on "Feature selection / dimensionality" above, via
`pca_analysis.py` (`python pca_analysis.py --data-dir ./data`): does a
small number of components capture the ridge-recoverable signal, and do
the top components load on features that make physiological sense (a
sanity check that winsorization actually fixed the outlier problem, rather
than just hiding it)?

- **Variance is spread out, not concentrated.** Teacher needs 28/91
  components for 90% cumulative variance, student needs 18/40 - real but
  moderate redundancy (correlated electrodes/AUs), not a case where 3-4
  components explain everything. This means the earlier "ridge matches the
  MLP" result is better read as "signal is weak and diffuse across many
  features" than "only a handful of features matter."
- **The loadings check out - this is the reassuring result.** Teacher PC1
  (20% of variance) is a broad Beta/Gamma EEG factor spread across
  electrodes - a plausible global cortical-activation signal, not one
  runaway feature. PC2 (10%) is cleanly all `bio_EMG_*` features together -
  a coherent muscle-tension factor. Student PC3 (13%) loads on `AU_mean_
  Upper Lip Raiser` + `Lip Corner Puller` + `Cheek Raiser` together - AU25 +
  AU12 + AU6, the textbook "Duchenne smile" co-activation pattern from the
  facial-action-coding literature. None of the top components are
  dominated by a single feature or an artifact-looking value - the
  winsorization fix (see "Tuning pass") appears to have actually fixed the
  outlier problem rather than just capping its visible symptom.
- **But PCA compression costs the strongest signal.** Reducing to the
  90%-variance components changes ridge regression R² only within noise,
  but *hurts* tertile classification specifically on Arousal - the PoC's
  best result so far drops from 0.532→0.454 F1 (teacher) and 0.545→0.492
  F1 (student). Unsupervised, variance-based compression is discarding
  some lower-variance direction that carries Arousal-specific class
  information. **Takeaway: don't use PCA as a dimensionality-reduction
  step for this task** - if a smaller feature set is wanted (e.g. for a
  lighter deployed student model), a supervised method (Lasso/L1, or
  PLS/LDA which use the label) is more likely to keep what
  classification needs than PCA will.

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

## Why the numbers are still weak, and what to try next

All four items from the original untuned-first-pass list above have now
been tried (see "Tuning pass" above) - none of them was a silver bullet,
but they weren't meant to be conclusive on their own; they were meant to
rule things out and surface leads. Two things stand out as worth pulling on
next, in priority order:

1. **Follow the classification signal.** `--classify`'s tertile macro-F1
   clearly beating majority-class (Arousal: 0.545 vs. 0.402) is the
   strongest positive result in this PoC so far. Worth digging into: is it
   specifically the *extremes* (low vs. high) that are separable while the
   middle tertile is the noisy part? A 2-class low-vs-high split, or a
   per-class F1/confusion-matrix breakdown (not just macro-F1), would tell
   you whether to reframe the eventual proposal target as coarse comfort
   bands rather than continuous PAD regression.
2. **The teacher isn't a solid privileged signal here.** Even after fixing
   the EEG outlier bug, teacher regression R² is still ~0 (Valence still
   negative) - the HRV/EDA/EEG features in *this* dataset may just be weak
   predictors of self-reported PAD for a disassembly task, which caps how
   much distillation can help regardless of student-side tuning. Worth
   checking against the project's own eventual HR+GSR data once collected,
   since this dataset's teacher signal quality doesn't necessarily transfer.
3. **Feature selection: try supervised selection, not PCA.** PCA (see
   "PCA feature-selection check" above) confirmed the features themselves
   are physiologically sensible - not the problem - but its unsupervised
   variance-based compression actively hurts the best classification
   result (Arousal). Untried: L1 (Lasso) or PLS to see which *specific*
   features a supervised method keeps for Arousal classification - useful
   both to shrink the student model and as context for which real
   sensors/features are worth the trouble to collect for the actual
   proposal.

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
