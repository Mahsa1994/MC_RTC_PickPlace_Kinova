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
python train_kd_classify.py --data-dir ./data --targets Arousal Valence Dominance \
    --seeds 0 1 2 3 4 --extra-kd-teachers "gboost on physio_raw"   # ~35 min on 18 cores
```

`prepare_dataset.py` only downloads `features.zip` (pre-extracted features),
**not** the 15.3GB raw `physiological_data.zip` - not needed for this PoC.
`--skip-download` reuses an already-downloaded `data/features.zip`.

`train_distill.py` flags worth knowing about:
- `--cv N`: subject-wise `GroupKFold` with `N` folds, mean±std across folds
  (recommended - a single split only holds out ~11 participants, too noisy
  to trust on its own). Omit for a quick single-split run.
- `--classify`: also reports tertile (low/mid/high) classification
  macro-F1 alongside the regression metrics. (Before 2026-10-09 this was
  silently 2-class for Valence/Arousal - see "Binning bug" below.)
- `--no-subject-center`: disable per-participant feature centering (on by
  default) to A/B against it.
- `--hidden`, `--weight-decay`: MLP width and L2 regularization.

`python pca_analysis.py --data-dir ./data` runs a PCA sanity check (see
"PCA feature-selection check" below): explained-variance curve, top feature
loadings per component, and full-dim vs. PCA-reduced performance via the
same CV harness.

`train_kd_classify.py` is the classification-based teacher/student
pipeline from the 2026-10 supervisor review (see "KD classification pass"
below): visual-only baseline, teacher capacity/input sweep, 2-layer
student, distillation-weight sweep. Every MLP is a 5-member inner-CV
ensemble, and the outer test fold is never used for early stopping or
selection. Flags worth knowing about:
- `--targets`, `--seeds`: which of V/A/D to run, and how many repeats
  (per-fold numbers are averaged over seeds).
- `--teacher-inputs`, `--teacher-archs`, `--no-hgb`: shrink the teacher
  sweep for quick runs.
- `--kd-lambdas`, `--soft-modes`, `--temperature`, `--extra-kd-teachers`:
  the Step 4 distillation sweep.
- `--es-metric`, `--teacher-input-noise`: early-stopping criterion and
  input-noise regularization (defaults `f1` / `0.5`, chosen by the sweep
  described below).
- `--out FILE.json`: dump every per-fold/per-seed score.

`python train_deep_teacher.py --data-dir ./data` (~15 min) is the Step 2
follow-up from the dataset authors' "advanced ML / multimodal fusion"
suggestion: same folds/seeds/protocol, comparing the previous best teacher
against teachers on the full bio feature set (HRV/EDA/RSP included - see
"Deep teacher" below for why they were missing), a per-modality deep fusion
net, and a deep temporal model over the per-window bio/EEG sequences. Reads
`data/raw/features/{bio_features_60s,eeg_features_5s}.csv` directly.
`--deep-models fusion|temporal` and `--targets` shrink it for quick runs.

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
  the metric, not just the model. **Corrected 2026-10-09: these Valence and
  Arousal numbers were 2-class, not 3-class, and the "lift" was mostly a
  metric artifact - see "Binning bug" below.**
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

(Superseded - see "Binning bug" below for the corrected 3-class table.)

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
- **But PCA compression costs the strongest signal.** *(Corrected
  2026-10-09: this bullet's numbers came from the 2-class binning bug. With
  real 3-class bins the PCA effect on Arousal is within noise - student
  0.392 full vs 0.409 PCA, teacher 0.326 vs 0.302 - so "PCA hurts Arousal"
  no longer holds. The PCA-vs-supervised-selection advice stands on general
  grounds, but this check doesn't demonstrate it.)* Reducing to the
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

## KD classification pass (2026-10-09)

Applied the 4-step plan from the supervisor review, as 3-class (low/mid/high)
classification: (1) visual-only baseline, (2) a bigger, better-trained
teacher (highest priority), (3) a 2-layer 64-32 ReLU + softmax student, (4) a
high distillation penalty. Code: `train_kd_classify.py`. Numbers below are
from the run command in "Running it": 5 outer folds x 5 seeds, mean+/-std
across folds.

### Binning bug (found first, invalidates the earlier classification numbers)

`tertile_bins()` took train-quantile cut points, but SAM ratings are integers
on 1-5, so the cut points tie: for Arousal q1 = q2 = 2, and
`np.digitize(y, [2, 2])` returns **two** classes (Arousal = 1 vs >= 2). Same
for Valence (< 4 vs >= 4); only Dominance was really 3-class. So the "0.545
Arousal F1 vs 0.402 majority" headline was a 2-class result, and on accuracy
the teacher (0.638) was actually *below* the majority-class rate (0.676).
Fixed: cut points now sit between observed levels, chosen on train for the
most equal three-way split (Arousal becomes 1 / 2 / 3-5, roughly
226/287/185 trials). Corrected `train_distill.py --classify` (logistic
regression):

|          | teacher F1 | student F1 | majority-class F1 |
|----------|-----------:|-----------:|-------------------:|
| Valence  |      0.318 |      0.342 |               0.193 |
| Arousal  |      0.326 |      0.392 |               0.193 |
| Dominance|      0.316 |      0.363 |               0.198 |

With 3 classes, macro-F1 for always predicting the most common class is
~0.19, but its *accuracy* is ~0.41, so read both columns below.

### Protocol changes in `train_kd_classify.py`

- **The outer test fold is never used for training decisions.**
  `train_distill.py`'s `train_model()` early-stops on the fold it reports.
  Here each outer-train set is split again by participant (5 inner folds):
  every MLP is trained once per inner fold, early-stopped on that fold's
  held-out participants, and the 5 members' softmax outputs are averaged.
- **Teacher selection and "OOF" soft labels are clean.** Every outer-train
  trial gets an out-of-fold teacher prediction from a teacher that never saw
  that participant. That prediction is used to pick the teacher per fold and
  as one of the two soft-label sources for Step 4. MLP teachers early-stop on
  a separate participant slice, because early-stopping on the same set
  inflated their OOF F1 from a real ~0.35 to ~0.43 and biased selection
  toward MLPs.
- **Teacher training:** early stopping on held-out macro-F1 instead of
  cross-entropy, plus Gaussian input noise (std 0.5). See Step 2.

### Model specifications

Shared by every neural model in `train_kd_classify.py`:

- **Task:** 3-class (low/mid/high) classification. The final layer gives 3
  logits, and softmax turns them into class probabilities (cross-entropy
  applies softmax internally during training).
- **Inputs:** winsorized to the [0.5, 99.5] percentile range per feature,
  then z-scored with train-fold statistics. AUs are always subject-centered.
- **Hidden layers:** each is `Linear -> ReLU -> Dropout`.
- **Optimization:** full-batch AdamW (lr 1e-3, weight decay 1e-2), at most
  1000 epochs. Training stops after 100 epochs without improvement in
  macro-F1 on held-out participants, and the best epoch is kept.
- **Ensembling:** every reported MLP number is a 5-member ensemble, one
  member per inner fold, with softmax outputs averaged.
- **Evaluation:** 5 outer subject-wise folds x 5 seeds.

Roughly 450 training trials per fold against 1.4k-232k parameters is heavily
over-parameterized, which is why dropout, input noise and early stopping
matter so much here.

**Baseline before this pass (`train_distill.py`)**
- MLP: `Linear(in -> 32) -> ReLU -> Dropout(0.3) -> Linear(32 -> 3)`, trained
  as *regression* (MSE on V/A/D). Adam (lr 1e-3, L2 0.1), 300 full-batch
  epochs. 1,411 parameters on AUs, 3,043 on physio.
- Its classification numbers came from sklearn `LogisticRegression` (L2,
  C=1.0, lbfgs), not from the MLP.

**Step 1 + 3: student (input: 40 AU features = 20 AUs x per-trial mean/std)**

| model | layers | params | dropout | other |
|---|---|---:|---:|---|
| student MLP 64-32 (new, used for KD) | `Linear(40->64) -> ReLU -> Dropout -> Linear(64->32) -> ReLU -> Dropout -> Linear(32->3) -> softmax` | 4,803 | 0.3 | no input noise; trains on 80% of outer-train participants, early-stops on the other 20% |
| student MLP 32 (old arch) | `Linear(40->32) -> ReLU -> Dropout -> Linear(32->3) -> softmax` | 1,411 | 0.3 | same |
| student logistic regression (reference) | linear, softmax | 123 | - | sklearn, L2, C=1.0 |

**Step 2: teachers**

Four input sets:
- `physio`: 91 features (76 EEG band-power + 15 EMG), subject-centered.
- `physio_raw`: the same 91, not centered.
- `physio+AU` / `physio_raw+AU`: 131 features (the 91 plus the 40 AUs).

| model | hidden layers | dropout | params (91 / 131 inputs) |
|---|---|---:|---:|
| mlp32 | 32 | 0.3 | 3,043 / 4,323 |
| mlp128-64 | 128 -> 64 | 0.3 | 20,227 / 25,347 |
| mlp256-128-64 | 256 -> 128 -> 64 | 0.4 | 64,899 / 75,139 |
| mlp512-256-128 | 512 -> 256 -> 128 | 0.5 | 211,715 / 232,195 |
| logistic regression | - | - | L2, C=0.1 |
| gradient boosting | sklearn `HistGradientBoostingClassifier` | - | 200 trees, learning rate 0.05, max depth 3 |

- Teacher MLPs add Gaussian input noise (std 0.5) during training.
- They train on 64% of the outer-train participants and early-stop on a
  separate 16%; the remaining 20% gets clean out-of-fold predictions.
- The teacher used for KD is picked per fold by out-of-fold macro-F1.
- Best Arousal teacher: **mlp512-256-128 on physio_raw+AU (232,195
  parameters).**

**Step 4: distillation**
- Student: the 64-32 MLP above (4,803 parameters), same training settings.
- Loss: `(CE(y, s) + lambda * T^2 * KL(teacher_T || softmax(s / T))) / (1 + lambda)`,
  with T = 2 and lambda in {1, 3, 10, inf} (inf = pure distillation, no
  hard-label term).
- Teacher soft labels come from one of two sources:
  - **in-sample:** the teacher member that trained on those trials;
  - **OOF:** the teacher member that never saw that participant.
- Teachers distilled:
  - **best:** selected per fold;
  - **small:** mlp32 on `physio`, 3,043 parameters;
  - **fixed:** gradient boosting on `physio_raw`.

### Results

Arousal (the supervisor's target):

| model | macro-F1 | accuracy |
|---|---:|---:|
| majority class | 0.193 +/- 0.029 | 0.412 |
| task identity only (logreg on one-hot task name) | 0.440 +/- 0.044 | 0.450 |
| student logreg on AUs (old reference) | 0.392 +/- 0.026 | 0.407 |
| **Step 1/3:** student MLP 32 (old arch), no teacher | 0.413 +/- 0.056 | 0.440 |
| **Step 1/3:** student MLP 64-32, no teacher | 0.411 +/- 0.047 | 0.432 |
| **Step 2:** old-style teacher (MLP 32, centered bio+EEG) | 0.354 +/- 0.051 | 0.370 |
| **Step 2:** best teacher (MLP 512-256-128, raw bio+EEG + AUs) | 0.418 +/- 0.041 | 0.436 |
| **Step 4:** best distilled student (best teacher, lambda=1) | 0.417 +/- 0.051 | 0.442 |

Step 4, change in Arousal macro-F1 vs the no-teacher 64-32 student (loss =
(CE + lambda * T^2 * KL) / (1 + lambda), T = 2; "in-sample" = soft labels
from a teacher trained on those trials, "OOF" = from a teacher that never
saw that participant):

| teacher, soft labels | lambda=1 | 3 | 10 | inf (pure KD) |
|---|---:|---:|---:|---:|
| best teacher, in-sample | +0.006 (4/5 folds up) | -0.004 | -0.009 | -0.016 |
| best teacher, OOF | -0.002 | -0.018 | -0.024 | -0.028 |
| small teacher, in-sample | -0.001 | -0.014 | -0.036 | -0.051 |
| small teacher, OOF | -0.008 | -0.032 | -0.071 | -0.088 |

Valence and Dominance (macro-F1):

| model | Valence | Dominance |
|---|---:|---:|
| majority class | 0.193 | 0.198 |
| task identity only | 0.303 | 0.388 |
| student logreg on AUs | **0.342** | **0.363** |
| student MLP 64-32, no teacher | 0.327 | 0.324 |
| best teacher on the test folds | 0.378 (gboost, raw bio+EEG) | 0.350 (MLP 256-128-64, bio+EEG+AU) |
| best distilled student | 0.327 (+0.000) | 0.325 (+0.000) |

### What each step showed

1. **Visual-only baseline:** the AUs carry a little signal on their own:
   Arousal 0.41 F1, but only 0.43-0.44 accuracy vs 0.41 for the majority
   class. This is the bar a distilled student has to clear.
2. **Bigger teacher:** capacity alone did *not* help at first. Training
   curves show every MLP reaching its best held-out cross-entropy within
   10-50 epochs at ~1.0, barely below chance (ln 3 = 1.10), then memorizing
   the training set; bigger nets just memorize faster. Three changes made the
   teacher better (Arousal, 3-seed sweep): early stopping on F1 rather than
   cross-entropy (+0.05 to +0.10 for the 128-64 and larger MLPs on physio
   and raw-physio+AU inputs, ~0 on centered physio+AU), input noise on top
   of that (+0.02 to +0.04 on centered inputs, ~0 on raw), and raw (not
   per-person-centered) physio plus AUs as input. Raw physio keeps each person's absolute level, and ~1/3 of
   Arousal variance lies between participants. Together these took the
   Arousal teacher from 0.33-0.35 to 0.42 F1, and bigger teachers now edge
   out small ones (0.418 vs 0.396 for MLP 32 on the same input), as the
   supervisor expected, though within ~1 std. **But the best teacher only
   matches the visual-only student** on Arousal. Only on Valence does a
   teacher clearly beat the student, and that teacher is gradient boosting
   on raw physio, not an MLP.
3. **2-layer student:** no measurable difference from the old single 32-unit
   layer on any target. On Valence and Dominance, plain logistic regression
   on AUs beats both MLPs. The MLPs almost never predict the "high"
   Dominance class (~19% of trials): per-class F1 0.05 vs 0.22 for logreg.
4. **High distillation penalty: hurts here.** The student gets steadily worse
   as lambda grows, and the drop is larger the weaker the teacher. The best
   case is lambda=1 (+0.006, within noise). Even the Valence teacher that
   *does* beat the student transfers nothing (+0.000 at best, down to -0.06).
   Reasons: (a) for Arousal/Dominance the teacher isn't better than the
   student, so there's nothing to transfer; (b) the Valence teacher's edge
   comes from per-person absolute physio levels, which the student's
   per-person-centered AUs cannot represent, and distillation can only teach
   a function of the student's own input; (c) OOF soft labels were *worse*
   than in-sample ones, contrary to expectation. A teacher at ~0.35-0.40
   out-of-fold F1 produces soft labels that are mostly noise, while
   in-sample soft labels are close to smoothed hard labels, which is
   harmless but adds nothing.

**Bottom line:** in this dataset the limit is the signal, not the model.
Task identity alone predicts Arousal and Dominance better than any sensor
model, so the labels mostly reflect which task was being done. The physio
features add little beyond that. A high distillation weight only helps
when the teacher is clearly better than the student *on information the
student can see*, and here it isn't.

### Next steps

1. **Make the teacher's advantage learnable by the student.** Either give
   the student per-person calibration too (matches the personalization
   direction), or define the target per person ("higher arousal than this
   person's usual"), so that teacher and student both work on within-person
   variation.
2. **A richer contactless student input** (camera rPPG heart rate/HRV,
   thermal) is the most direct fix for (b) above: it gives the student
   access to some of what the physio teacher sees.
3. **Control for task identity:** check whether AUs/physio carry Arousal
   signal *beyond* the task label (e.g. predict within-task residuals), which
   is what the real pick-and-place study needs, since there the task is
   fixed.
4. **Handle class imbalance** (class weights or balanced sampling) for
   Dominance "high" and Valence "high", which the MLPs mostly ignore.

## Deep teacher (Step 2 follow-up, 2026-10-09)

The dataset authors conclude that the physiological signals are informative
but that higher accuracy "underscores the need for advanced machine learning
approaches and multi-modal fusion techniques". `train_deep_teacher.py` tests
that for the teacher only (Step 2). It uses the same folds, labels and
protocol as `train_kd_classify.py`, with 3 seeds.

**Feature bug found first: the teacher never had heart-rate or
skin-conductance features.** `dataset._usable_columns()` drops any column
with even one NaN. HRV features are missing in only ~0.4% of trials, but
that is enough. **243 of the 264 bio features were dropped**: all 137 EDA,
72 of 74 HRV, and all respiration (RSP/RRV/RAV) features. Every "physio"
teacher above was really EEG + 15 EMG features. `train_deep_teacher.py`
keeps all of them instead: window-level winsorizing, train-fold
standardization, and missing values imputed to the train mean.
`dataset.py` is unchanged, so `train_kd_classify.py` numbers still use the
old features.

**Models compared** (deep model sizes are for these inputs):

| model | input | params |
|---|---|---:|
| reference: mlp512-256-128 (previous best) | old features: EMG+EEG + AU (131) | 232,195 |
| mlp512-256-128 / logreg / gboost | full physio (bio 248 + EEG 88 trial means = 336), +AU (376), or + within-trial std (712) | 337k / 358k / 530k for the MLP |
| **deep fusion** | per-trial mean+std: bio 496, EEG 176, AU 40 | 141,379 |
| **deep temporal** | per-window sequences: bio 248 x ~3.6 windows, EEG 88 x ~48.6 windows, AU 40 | 126,661 |

**Deep fusion:**
- One encoder per modality: `Linear(in->128) -> GELU -> Dropout(0.3) ->
  Linear(128->64) -> GELU`.
- The three 64-d outputs are concatenated (192), then
  `Linear(192->128) -> GELU -> Dropout(0.3) -> Linear(128->3) -> softmax`.
- Modality dropout 0.2: a whole modality's embedding is zeroed for 20% of
  training samples.

**Deep temporal:**
- Bio and EEG branches: `Linear(F->64) -> GELU -> Dropout(0.3)`, then 2
  residual blocks of `Conv1d(64->64, k=3, dilation 1 then 2) -> GELU ->
  Dropout -> +skip -> LayerNorm`. The receptive field is 7 windows: ~35 s of
  EEG, ~7 min of bio.
- Each branch pools over time two ways, attention pooling and masked mean
  pooling, giving 128 dims per branch.
- The AU encoder (as in fusion) gives 64 dims. Concatenated (320), then
  `Linear(320->128) -> GELU -> Dropout(0.3) -> Linear(128->3) -> softmax`.
- Modality dropout 0.2.

**Deep training:**
- Mini-batches of 32; AdamW (lr 1e-3, weight decay 1e-2); at most 150
  epochs, stopping after 25 without improvement in held-out macro-F1.
- Augmentation: Gaussian input noise (std 0.5), and each training sequence
  randomly cropped to a contiguous 50-100% chunk, at most 128 windows.
- 5-member inner-CV ensemble, early stopping on a separate participant
  slice (same as the MLP teachers).

**Results (macro-F1, 5 folds x 3 seeds):**

| teacher | Valence | Arousal | Dominance |
|---|---:|---:|---:|
| student MLP 64-32 on AUs (the bar) | 0.333 | 0.408 | 0.325 |
| reference: mlp512-256-128, old features + AU | 0.347 | 0.411 | 0.340 |
| mlp512-256-128, full physio + AU | 0.342 | **0.412** | **0.343** |
| mlp512-256-128, full physio mean+std + AU | **0.355** | 0.392 | 0.340 |
| gboost, full physio + AU | 0.351 | 0.393 | 0.322 |
| deep fusion (bio / EEG / AU encoders) | 0.330 | 0.391 | 0.330 |
| deep temporal (bio+EEG sequences / AU) | 0.313 | 0.397 | 0.324 |
| *physio only (no AU):* logreg, full physio | 0.336 | 0.403 | 0.320 |
| *physio only:* best old-feature physio-only teacher (5-seed run above) | 0.378 (gboost) | 0.368 (logreg) | 0.325 (MLP 128-64) |

**What this shows:**
- **The deep models do not beat the simple MLP teacher on any target.** The
  temporal model is the worst on Valence (-0.034 vs the reference) and
  Dominance. With ~450 training trials from ~40 participants per fold,
  there isn't enough data to learn within-trial dynamics. Adding even simple
  within-trial std features doesn't help consistently either. The labels
  are one questionnaire per trial, so whatever moment-to-moment structure
  exists isn't what the labels reflect.
- **Restoring HRV/EDA/respiration helps the physio-only teacher on Arousal**
  (logreg 0.368 -> 0.403), consistent with the authors' point that these
  signals carry information. But once AUs are in the input it changes
  nothing (0.411 -> 0.412), and on Valence the old-feature gboost (0.378)
  stays ahead.
- Net: architecture is not the bottleneck at this data size. The ceiling is
  still the task-identity baseline (Arousal 0.440, Dominance 0.388).
  Distillation results (Step 4) would not change with a deep teacher, so
  they weren't rerun.

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

*(Written 2026-08-19, before the binning bug was found. Item 1's premise -
a strong Arousal classification signal - didn't survive the fix. See "KD
classification pass" above for the current picture and next steps.)*

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
