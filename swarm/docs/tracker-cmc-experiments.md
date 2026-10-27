# Tracker and CMC experiments (2026-10-26)

How well X keeps each vehicle's ID when the drone flies with its camera fixed,
and what it took to improve it: the detector, ByteTrack's matching, and camera
motion compensation (CMC). The method and tools are in
[MOT experiments](features/mot-experiments.md). This page is the record of
the first round of experiments and what they showed. To repeat them yourself, see
[step by step: run it yourself on the Radxa](features/mot-experiments.md#step-by-step-run-it-yourself-on-the-radxa).

**Outcome.** The rknn-sim profile now runs **VisDrone + ByteTrack + CMC**:
`yolov8n_visdrone` at confidence 0.2, ByteTrack with active tracks matched
first and corrected IoU thresholds, and CMC without the inlier gate
(A5X_RPI_Release `2b48943`).

On the Radxa it runs at 25 frames/s. Against the same VisDrone detector with
the tracker as it was, ID switches fell from 34 to 25 at the junction and
from 89 to 24 while the drone manoeuvres, with IDF1 0.53 / 0.43 / 0.38
(convoy / junction / drone motion); see [section 7](#7-visdrone--bytetrack--cmc-on-the-radxa).

How it got there:
- **Detector, the bottleneck:** the COCO `yolov8n` found 6-14% of the cars.
  `yolov8n_visdrone` at confidence 0.2 finds 52-67%, with 86-95% of its boxes
  correct, at 25-30 ms per frame.
- **Tracker:** X's ByteTrack had a bug that gave a car a new ID after a single
  missed detection, its IoU thresholds were far too strict, and stale lost
  tracks could take a partly hidden car's detection.
- **CMC:** it more than halves the IDs per car while the drone turns.

## Set-up

**Recordings** (sim, `--ground-truth`, `record: true` missions; 1920×1080
GimbalCam at about 16 frames/s):

| Recording | Frames | Traffic | Drone and camera | Car size in the frame (median box) |
|---|---|---|---|---|
| `mot_convoy` | 3387 | 5 identical cars nose to tail, 12 m/s, about 10 m gaps | Flies beside the road at 9 m/s, camera fixed 90° right, 34° down | 88×42 px |
| `mot_junction` | 2396 | Two streams of 4 identical cars interleaving through a four-way junction, 10 m/s | Holds 70 m off the junction, then crosses slowly; camera on a fixed compass heading | 33×15 px |
| `mot_drone_motion` | 2533 | 8 identical cars, 8 m/s | Hovers beside the road, yaws ±45° (about 40°/s), dashes to 15 m/s and stops, climbs 37 → 57 m; camera fixed to the nose | 88×42 px |

Each recording also has two parked cars (`Car1`, `Car2`) at the frame edge.
They are scored with the rest but left out of the "IDs per car" column below,
which counts the scenario's moving cars only.

**Replays:**
- **Detector runs:** `X/mot_replay.py` on the Radxa (RK3588, rknn-sim
  profile), every frame in order.
- **Tracker-only comparisons:** the same script with `--detections`, which
  feeds a Radxa run's saved detections, on the laptop or the Radxa.
- **Oracle runs:** `--oracle` feeds the true boxes as detections, optionally
  spoiled with 3 px Gaussian jitter on each edge and 10% of boxes dropped
  (seed 0), to test the tracker without a detector.

**Scoring:** `swarm/mot_eval.py`, IoU 0.5 for a match, true boxes narrower
than 8 px ignored.
- **ID switches:** a car matched to a different track than before.
- **IDs per car:** distinct track IDs a car was given (1 is perfect).
- **IDF1:** how much of each car's time was spent under its single best ID.
- **Detector recall and precision:** per box, from the raw detections, so
  detector misses are separated from tracker mistakes.

One run per setting. The sim, the drone's flight and the traffic repeat
closely (repeated mission runs flew 0.3-0.8 m apart), so differences of a few
switches are within run-to-run noise.

## 1. The ByteTrack bug: a missed frame cost a new ID

**Symptom.** Oracle replay of `mot_convoy` with lightly spoiled true boxes
(3 px jitter, 10% dropped): each moving car got 43 IDs on average.

**Cause** (`X/utils/trackers/mot.py`, `ByteTrack.update`):
- A track that missed a frame went "lost".
- Stage 1 matched high-confidence detections against active tracks only.
- Lost tracks were offered only low-confidence detections, in stage 2.

So when the car was detected again at normal confidence, its lost track was
never a candidate, and ByteTrack started a new one.

**Fix:** as in ByteTrack, stage 1 matches high-confidence detections against
active and lost tracks, stage 2 lets low-confidence detections rescue the
active tracks left over, and a rematched lost track is confirmed again.
`tests/test_mot_replay.py` covers it; its dropout tests fail on the old code.

| `mot_convoy`, oracle | ID switches | IDs per car | IDF1 | MOTA |
|---|---|---|---|---|
| Clean true boxes (before and after: identical) | 15 | 1.8 | 0.77 | 0.98 |
| Spoiled boxes, before the fix | 396 | 43.2 | 0.07 | 0.61 |
| Spoiled boxes, after the fix | 101 | 3.2 | 0.72 | 0.77 |

With X's real detections from the Radxa (COCO `yolov8n`, confidence 0.4), the
same detections through the old and the fixed tracker:

| Recording | ID switches before → after | IDs per car before → after | IDF1 before → after |
|---|---|---|---|
| convoy | 16 → 4 | 3.6 → 1.4 | 0.087 → 0.101 |
| junction | 9 → 2 | 0.75 → 0.62 | 0.046 → 0.069 |
| drone motion | 27 → 16 | 2.5 → 2.5 | 0.057 → 0.100 |

An "IDs per car" below 1 means some cars were never tracked at all.

## 2. The detector: cars too small for the COCO model

At confidence 0.4 the COCO `yolov8n` found 6-14% of the cars, and IDF1 stayed
around 0.1 whatever the tracker did:
- **Too few pixels.** The whole 1920×1080 frame is shrunk to the model's
  640 px input, so a 88×42 px car becomes about 29×14 px, and a junction car
  about 11×5 px.
- **Wrong labels.** It also called cars bench, fork or airplane.

Detector variants, all with the fixed tracker:

| Detector (Radxa) | Recall | Precision | ms/frame (detector) | IDF1 convoy / junction / drone motion | IDs per car convoy / junction / drone motion |
|---|---|---|---|---|---|
| `yolov8n`, conf 0.4 (as before) | 6-14% | 71-95% | 36 | 0.10 / 0.07 / 0.10 | 1.4 / 0.6 / 2.5 |
| `yolov8n`, conf 0.2 | 12-28% | 44-85% | 36 | 0.13 / 0.08 / 0.12 | 1.6 / 0.6 / 2.5 |
| `yolov8n`, conf 0.2 + SAHI tiles | 59-77% | 44-57% | 99 | 0.67 / 0.24 / 0.31 | 1.0 / 7.5 / 10.5 |
| **`yolov8n_visdrone`, conf 0.2** | **52-67%** | **86-95%** | **30** | 0.53 / 0.36 / 0.22 | 2.2 / 4.75 / 11.0 |
| `yolov8n_visdrone`, conf 0.2 + SAHI | 89-93% | 50-76% | 78 | 0.61 / 0.43 / 0.42 | 1.2 / 18.6 / 12.5 |

(Ranges are over the three recordings; ms/frame is the detector alone, from
the replay's timing.)

- **Lowering the confidence alone** roughly doubles recall, but from a very
  low base.
- **VisDrone** (an aerial-view model: car, van, truck, bus, ...) sees most cars
  with few false boxes, fast enough for live X (`INPUT_FPS` 20).
- **SAHI tiling** (the detector on 640 px tiles at full resolution) finds the
  most cars, but:
  - false and duplicate boxes, presumably where tiles overlap, raise false
    positives from single digits to 200-1100 per run
  - those boxes multiply ID switches, especially at the junction
  - it is too slow for live X: see [Timing](#4-timing-on-the-radxa)

  It was left off.

The tracker results above used the old IoU thresholds and no CMC; section 3
changes those.

## 3. ByteTrack thresholds and CMC

**Thresholds.** X's ByteTrack treats `BYTETRACK_MATCH_THRESH` and
`BYTETRACK_R1_MATCH_THRESH` as minimum IoUs, but the configs held upstream
ByteTrack's IoU-distance values:
- stage 2 (low-confidence detections) needed IoU 0.8
- stage 1 needed IoU 0.5

They are now IoU 0.5 and 0.2, which is upstream's intent.

**CMC** (`OBJECT_TRACKER_CMC`) estimates how the whole image moved between
frames (sparse optical flow, RANSAC affine fit) and shifts every track's
predicted position by it before matching.

Same VisDrone detections from the Radxa, re-tracked with
`mot_replay.py --detections`:

| Recording | Tracker | ID switches | IDs per car | IDF1 | MOTA |
|---|---|---|---|---|---|
| convoy | old thresholds | 11 | 2.2 | 0.53 | 0.55 |
| convoy | corrected thresholds | 9 | 2.2 | 0.53 | 0.57 |
| convoy | corrected + CMC | 8 | 2.2 | 0.53 | 0.57 |
| junction | old thresholds | 34 | 4.75 | 0.36 | 0.39 |
| junction | corrected thresholds | **25** | **3.62** | **0.43** | 0.43 |
| junction | corrected + CMC | 38 | 3.75 | 0.43 | 0.43 |
| drone motion | old thresholds | 89 | 11.0 | 0.22 | 0.33 |
| drone motion | corrected thresholds | 71 | 8.75 | 0.30 | 0.44 |
| drone motion | corrected + CMC | **28** | **3.75** | **0.39** | **0.48** |

- **Corrected thresholds** help everywhere, most at the junction and while the
  drone moves.
- **CMC while the drone turns** cuts ID switches by 61% and IDs per car from
  8.75 to 3.75. The camera-motion cause in `events.csv` falls from 54 to 34
  switches.
- **CMC with a steady camera** (convoy) changes nothing.
- **CMC at the junction** adds switches tagged crowding (6 → 18). Many cars
  move through the view while the camera is nearly still, and they pull the
  image-motion estimate.

The oracle runs (spoiled true boxes) agree: CMC took the drone-motion run from
16.2 to 8.6 IDs per vehicle, and the junction from 32.7 to 37.8.

These CMC results predate two later CMC changes (`042ac40`):
- **It ignores detected vehicles:** no motion points are taken from inside
  this frame's detections, meant for the junction case.
- **It rejects weak fits:** a fit with too few inliers is not applied.

Section 5 measures them: ignoring the cars did not help the junction, and the
inlier gate hurt at low frame rates. The gate is now off in rknn-sim.

## 4. Timing on the Radxa

`mot_replay.py` times each stage per frame (`timing.csv`). `mot_drone_motion`,
2533 frames, VisDrone at conf 0.2, CMC on:

| ms per frame | No SAHI: mean | p50 | p95 | max | SAHI: mean | p50 | p95 | max |
|---|---|---|---|---|---|---|---|---|
| Video decode (replay only) | 5.4 | 4.8 | 10.1 | 70.5 | 5.9 | 5.4 | 9.3 | 69.6 |
| Detector | 25.3 | 25.0 | 31.1 | 80.2 | 78.7 | 78.3 | 87.4 | 200.7 |
| CMC | 8.0 | 6.7 | 15.2 | 28.4 | 9.7 | 11.1 | 14.7 | 20.7 |
| Rest of ByteTrack | 0.3 | 0.1 | 1.1 | 3.8 | 1.3 | 1.0 | 4.3 | 9.4 |
| **Total** | **39.1** | 38.4 | 52.5 | 126.6 | **95.6** | 94.9 | 109.0 | 251.6 |
| Frames/s | **25.6** | | | | **10.5** | | | |

- **CMC costs about 8 ms per frame.** On saved detections, CMC off vs on took
  19.1 s vs 38.5 s for the whole recording.
- **Without SAHI,** live X's detection loop (detector, CMC, tracker) takes
  about 34 ms per frame, about 29 frames/s. That is above `INPUT_FPS` 20, so
  no frames are skipped and each result is about 34 ms old.
- **With SAHI** it takes about 90 ms, about 11 frames/s. Live X would track
  only about every other frame, each result about 90 ms old. ByteTrack's
  `MAX_LOST` (in frames) would then cover twice as long.
- **Caveat:** live, stream decoding and output encoding run in parallel and
  compete for the CPU, so the live margin is smaller than the replay shows.

## 5. Is CMC good enough? Scored against the true camera motion

**Question.** Should CMC be improved, for example by fusing gimbal and
attitude telemetry? Two parts:
- **Accuracy:** how far off is each CMC estimate?
- **Headroom:** how many ID switches would a perfect CMC save?

**Method** (`swarm/cmc_eval.py`, A5X_RPI_Release `mot_replay.py`):
- **True camera motion.** Computed from the recorded camera pose, field of
  view and vehicle positions. For each pair of processed frames, a car's
  3D position at the first frame is projected through both cameras: the
  difference is where the camera motion alone moved it.
- **Scoring each estimate.** Errors are measured at the true cars, which is
  where the tracker needs them. "IoU<0.2" is the share of car pairs whose
  CMC error alone pushes a box's overlap with itself below 0.2, ByteTrack's
  stage-1 minimum.
- **Pose lag.** In these recordings the camera pose for frame k belongs to
  the image of frame k+1: the sim's image metadata runs a frame ahead of the
  pixels (`sim_time` is right). The recorder now writes each frame the pose
  its pixels were rendered from, and marks it `camera_pose_aligned` in
  `meta.json`. `cmc_eval.py` and `mot_eval.py` shift older recordings by
  themselves. Checked on a new `mot_drone_motion` recording: 2.2 / 7.3 px
  (p50 / p90) unshifted, against 11.9 / 54.1 px shifted. With the pose read one row earlier, projected
  vehicles land within 0.6-3.8 px (median) of their true boxes, against
  1.5-9.5 px without, and 55 px at p90 while the drone yaws.
- **Perfect CMC.** `cmc_eval.py` writes the true warp per frame
  (`cmc_truth.csv`), and `mot_replay.py --cmc-warps` tracks with it instead
  of estimating. Also a rotation-only warp: what perfect, synchronised gimbal
  and attitude telemetry could give, ignoring the drone's own movement.
- **Detections.** The same Radxa VisDrone detections (conf 0.2) for every
  setting, tracked on the laptop.
- **Rates.** Every frame (about 16 frames/s, as live X without SAHI), and
  `--rate 8` (as live X with SAHI or a slower detector).

**ID switches** (one deterministic replay each; a few switches either way is
noise):

| CMC | Convoy, every frame | Junction | Drone motion | Convoy, 8 Hz | Junction | Drone motion |
|---|---|---|---|---|---|---|
| Off | 9 | **25** | 71 | 6 | 17 | 108 |
| Vision (current: targets masked, inlier gate 12 / 50%) | 8 | 39 | 27 | 14 | 18 | 40 |
| Vision, no inlier gate (4 / 0%) | 8 | 39 | 26 | 14 | 18 | 25 |
| **Perfect** (true warp) | 8 | 30 | 28 | 4 | 17 | **19** |
| Rotation only (perfect telemetry) | 8 | **24** | 29 | 3 | 18 | 23 |

**Estimate accuracy**, drone motion (error at the true cars, px p50 / p90;
share of car pairs with IoU<0.2):

| Camera turning | Every frame: no CMC | Vision | Perfect | 8 Hz: no CMC | Vision | Vision, no gate | Perfect |
|---|---|---|---|---|---|---|---|
| 0-5 deg/s | 3.4 / 11.1 | 0.6 / 1.7 | 0.2 / 0.6 | 9.5 / 35.6 | 1.4 / 4.7 (1.7%) | 1.4 / 4.5 (0.6%) | 0.6 / 1.7 |
| 15-30 deg/s | 32.8 / 53.0 (10.8%) | 2.4 / 4.7 (0%) | 2.6 / 4.8 | 94 / 138 (96%) | 4.8 / 70 (10.6%) | 4.4 / 16.3 (1.4%) | 6.4 / 10.5 |
| over 30 deg/s | 47.7 / 78.6 (31.5%) | 3.2 / 6.8 (0.4%) | 3.7 / 6.5 | 133 / 175 (100%) | 7.1 / 145 (11.2%) | 5.6 / 23.1 (0%) | 9.6 / 13.8 |
| Estimate rejected or failed, over 30 deg/s | | 15% | | | 63% | 5% | |

**What it shows:**
- **At every frame, vision CMC is as good as perfect.** Drone motion 27 vs
  28 switches, convoy 8 vs 8. While the camera turns faster than 30 deg/s, its
  error at the cars is 6.8 px at p90, the same as the true warp's. There is
  nothing left for better CMC, or telemetry, to win at this rate.
- **The inlier gate (`CMC_MIN_INLIERS` 12, `CMC_MIN_INLIER_RATIO` 0.5)
  hurts at 8 Hz.** It rejects 63% of estimates in fast turns, the frames
  that need them most, and drone-motion switches rise from 25 to 40. At
  every frame it makes no difference.
- **At 8 Hz, vision has headroom.**
  - Drone motion: 25 switches without the gate, against 19 for perfect CMC.
  - Convoy: 14, against 4 perfect and 6 with CMC off. The ground moves a
    steady ~30 px between processed frames. Vision's estimate of it
    scatters (12-43 px, p90 error 9.7 px, perfect 1.1). With CMC off, the
    Kalman velocity simply absorbs the steady pan.
- **Masking the detected cars did not fix the junction:** 39 switches,
  against 38 before. Even perfect CMC (30) is worse there than none (25);
  rotation-only gives 24. The camera is nearly still, and it looks at the
  ground at a shallow angle, so near and far ground move differently. One
  rotation + scale + shift warp fits the whole ground (p90 3.3 px) but not
  the distant cars: in the 0-5 deg/s frames it is wrong at the cars, which
  then lose their match.
- **Rotation only is close to perfect everywhere:** drone motion 29 vs 28
  and 23 vs 19; convoy and junction as good as the best. The drone's own
  movement matters little at these ranges. So timely, synchronised gimbal and
  attitude telemetry could replace or check the vision estimate. These sim
  numbers are its upper bound; X's live gimbal telemetry (10 Hz, no frame
  timestamps) would do worse.

**Answer.** At live X's frame rate without SAHI, CMC does not need improving
for a turning drone. The gaps are:
1. The inlier gate: lower or drop it.
2. A nearly still camera with many moving cars (the junction): any
   translation correction there costs IDs.
3. Lower frame rates, where vision's estimate gets noisy.

## 6. ByteTrack: what the remaining switches are, and the matching order

**Question.** With good detections and good CMC, why do IDs still outnumber
vehicles, and does ByteTrack need better frame-edge and size-jump handling?

**Method.**
- **Laptop replays** of ByteTrack (`mot_replay.py`); the tracker is
  deterministic, so these counts hold on the Radxa too.
- **Three inputs per recording:**
  - true boxes with perfect CMC (`--oracle --cmc-warps cmc_truth.csv`): the
    tracker's own mistakes only
  - true boxes with X's CMC
  - the Radxa's VisDrone detections (conf 0.2) with X's CMC
- **Settings:** CMC without the inlier gate, as rknn-sim now uses.
- **Diagnosis:** individual switches traced frame by frame (true box, track
  predictions, overlaps).

**What the switches were** (true boxes, perfect CMC, drone motion and
junction):
- **Out of view, not cut off.** Most "frame-edge" switches are cars that the
  yaw swing turned out of view for about 2.4 s and back.
  - Their track expired (`MAX_LOST` 30 frames is about 1.9 s here) and they
    came back with a new ID.
  - Keeping the track longer doesn't help: the car drives about 20 m (about
    300 px) while unseen, so its old prediction is nowhere near where it
    reappears.
  - `MAX_LOST` 60, 90 and 150 changed nothing with true boxes, and 1 switch
    with real detections.
- **Stolen by a stale lost track.** When part of a car was hidden, the
  visible sliver landed on an old *lost* track lying at that spot. The
  earlier fix let lost tracks compete in stage 1 on equal terms with active
  ones, so the stale track won (IoU 0.76 against 0.24 for the car's own
  track). The car flipped to that ID and back a frame or two later.
- **Out of view for a long time:** the parked cars, unseen for about 80 s.
- **The junction's turn home:** a cluster while the camera turns at
  90-110°/s.

**Changes tried** (ID switches; ✓ kept):

| Tracker | Junction, real detections | Drone motion, real detections | Drone motion, true boxes + perfect CMC |
|---|---|---|---|
| Before (lost tracks compete in stage 1) | 39 | 26 | 17 |
| ✓ **Matching order:** active tracks first, lost tracks only for the detections left | **26** | **22** | **15** |
| + containment: a detection mostly inside an active track's prediction is that car, partly hidden | 26 | 22 | 16 |
| + frame edge and size jumps: predictions clipped to the frame; a cut-off or suddenly shrunk box updates the position only, keeping the size | 28 | 22 | 18 |

The convoy stayed at 8 throughout. Junction IDF1 on real detections went
0.425 → 0.434.

Containment and the edge / size-jump handling did not help, because the
"edge" and "size-jump" switches were really the two causes above. They were
dropped.

**Kept** (A5X_RPI_Release `utils/trackers/mot.py`): stage 1 matches
high-confidence detections against active tracks first, then the leftovers
against lost tracks. Also, `mot_replay.py` reports track boxes clipped to the
frame, as the overlay draws them.

**Left, needing more than ByteTrack:**
- **Out-of-view cars:** a motion model that keeps predicting a car while it
  is unseen, with distance-based rather than overlap-based matching. With
  identical cars, position and timing are all there is.
- **The long-unseen parked cars:** re-identification.

## 7. VisDrone + ByteTrack + CMC on the Radxa

The final configuration, run end to end on the RK3588:
- **Code:** X as committed (A5X_RPI_Release `2b48943`), deployed to
  `~/Code/X_sim`.
- **Settings:** the rknn-sim profile as it now stands (`yolov8n_visdrone`,
  `ATR_CONFIDENCE` 0.2, no SAHI; ByteTrack with active tracks first and the
  corrected thresholds; `OBJECT_TRACKER_CMC: true` without the inlier gate).
- **Run:** `mot_replay.py` over every frame of the three recordings.

| Recording | Detector and tracker | ID switches | IDs per car | IDF1 | MOTA |
|---|---|---|---|---|---|
| Convoy | COCO yolov8n, conf 0.4 (the starting point) | 4 | 1.4 | 0.10 | 0.04 |
| | VisDrone, conf 0.2; ByteTrack as before, no CMC | 11 | 2.2 | 0.53 | 0.55 |
| | **VisDrone + ByteTrack + CMC** | **8** | 2.2 | 0.53 | 0.57 |
| Junction | COCO yolov8n, conf 0.4 | 2 | 0.6 | 0.07 | 0.04 |
| | VisDrone, conf 0.2; ByteTrack as before, no CMC | 34 | 4.75 | 0.36 | 0.39 |
| | **VisDrone + ByteTrack + CMC** | **25** | **3.75** | **0.43** | **0.43** |
| Drone motion | COCO yolov8n, conf 0.4 | 16 | 2.5 | 0.10 | 0.08 |
| | VisDrone, conf 0.2; ByteTrack as before, no CMC | 89 | 11.0 | 0.22 | 0.33 |
| | **VisDrone + ByteTrack + CMC** | **24** | **3.75** | **0.38** | **0.48** |

IDs per car counts the moving cars only; 1 is perfect.

- **The COCO rows have few switches only because the detector saw few cars**
  (6-14%): an untracked car can't change ID. IDF1 is the fair comparison.
- **The detector is the same in both VisDrone rows:** 52-67% recall, 86-95%
  precision. The difference is ByteTrack's fixes and CMC.
- **The Radxa matches the laptop.** The laptop, tracking the Radxa's saved
  detections, predicted 8 / 26 / 22 switches; the board gave 8 / 25 / 24.
  CMC's estimates differ slightly with the board's OpenCV build.

**Time per frame on the Radxa** (mean, the three recordings):

| Detector | CMC | Rest of ByteTrack | Video decode (replay only) | Total | Frames/s |
|---|---|---|---|---|---|
| 25.0-25.5 ms | 8.2-8.8 ms | 0.4-0.5 ms | 5.3-5.5 ms | 39.4-39.9 ms (p95 52-54) | 25.0-25.4 |

Live X decodes video in its own thread, so its detection loop takes about
34 ms per frame (about 29 frames/s), above its 20 frames/s input: no frames
skipped.

**What the switches left are** (likely causes from `events.csv`; a switch can
have several):

| Recording | Detector gap | Camera motion | Frame edge | Crowding | Other |
|---|---|---|---|---|---|
| Convoy | 15 | 0 | 12 | 3 | 37 |
| Junction | 17 | 3 | 11 | 5 | 26 |
| Drone motion | 31 | 37 | 14 | 4 | 28 |

These include fragmentations as well as ID switches.
- **Detector gaps:** the detector missing a car for a frame or more.
- **Camera motion:** mostly cars the yaw turns swing out of view for seconds,
  which return with new IDs: beyond ByteTrack (section 6).

The levers left are detector recall and a way to carry cars through seconds
out of view.

## 8. Repeat runs, live X, and CMC at a nearly still camera

**Live flights.** Six more flights in the sim (each mission twice), with X
running live on the Radxa as Drone1's companion:
- **Settings:** the rknn-sim profile, logging every frame (`MOT_LOG_DIR`).
- **Recording:** the sim recorded each flight, so each mission now has three
  recordings.
- **Scoring:** `live_align.py` lined X's log up with each recording; the
  shift (clocks plus stream delay) was 0.15-0.18 s on every flight.
- **Stand-down:** X stood down live as designed. It logged "the autopilot is
  flying a mission and holds the gimbal: standing down" at each start, and
  "gimbal control resumed" after.

**Repeat runs: replay on the Radxa, VisDrone + ByteTrack + CMC, three
recordings per mission.**

| Mission | ID switches (each recording) | Mean ± sd | IDs per car | IDF1 | MOTA | Detector recall |
|---|---|---|---|---|---|---|
| Convoy | 8, 18, 17 | 14.3 ± 5.5 | 3.5 ± 1.1 | 0.47 ± 0.06 | 0.52 ± 0.04 | 0.63 ± 0.04 |
| Junction | 25, 14, 14 | 17.7 ± 6.4 | 2.9 ± 0.7 | 0.45 ± 0.02 | 0.44 ± 0.01 | 0.53 ± 0.00 |
| Drone motion | 24, 27, 23 | 24.7 ± 2.1 | 3.6 ± 0.2 | 0.38 ± 0.01 | 0.48 ± 0.02 | 0.59 ± 0.01 |

Flights of the same mission differ by up to ±6 ID switches. Differences of a
few switches between settings measured on one recording (sections 1-7) are
within that noise. The large effects hold: the tracker fixes and CMC took drone
motion from 89 to 24.

**Live X against replay, same flights:**

| Flight | ID switches replay → live | IDs per car | IDF1 | Detector recall | MOTP (box overlap) |
|---|---|---|---|---|---|
| Convoy 1 | 18 → 26 | 4.2 → 5.2 | 0.45 → 0.41 | 0.61 → 0.61 | 0.86 → 0.84 |
| Convoy 2 | 17 → 28 | 4.0 → 5.2 | 0.42 → 0.41 | 0.60 → 0.61 | 0.85 → 0.83 |
| Junction 1 | 14 → 20 | 2.5 → 2.75 | 0.46 → 0.43 | 0.53 → 0.47 | 0.82 → 0.80 |
| Junction 2 | 14 → 18 | 2.5 → 2.9 | 0.47 → 0.43 | 0.52 → 0.46 | 0.82 → 0.80 |
| Drone motion 1 | 27 → 31 | 3.6 → 4.6 | 0.39 → 0.34 | 0.59 → 0.53 | 0.87 → 0.84 |
| Drone motion 2 | 23 → 37 | 3.4 → 5.25 | 0.37 → 0.31 | 0.57 → 0.50 | 0.87 → 0.84 |

- **Rate.** Live X processed 25.4-25.7 frames/s (detector and tracker
  36.4 ms per frame on average). That is above the 16 frames/s the sim
  rendered (with ground truth on), so it kept up.
- **Repeats.** The video stream repeats the newest frame to hold 30
  frames/s, so about 40% of X's frames were repeats.
  - **Not the cause.** Re-tracking X's live detections with and without the
    repeats gave 21 and 22 switches.
- **The compressed stream is the cause.** Live X reads H.264 video, where the
  replays read a near-lossless file:
  - its boxes are less precise (MOTP down 0.02-0.03 on every flight)
  - it detects about 6 points fewer cars at the junction and in drone motion
  - that costs 4-14 ID switches per flight
- **What follows.** Replays overstate live tracking quality by about that
  much. A higher stream bitrate (the sim's `RtspStreamer`) would close part
  of the gap.

**CMC at a nearly still camera.** The concern from section 3: any CMC
correction at the junction's nearly still camera seemed to cost IDs.
- **What was tried.** A minimum-motion setting that applies no correction
  below 1, 2, 4 or 8 px.
- **Result.** It changed nothing: real detections 26-28 switches (26
  without), true boxes 36-38 (36).
- **Why.** With the current tracker CMC no longer costs IDs at the junction:
  CMC off gives 25 switches against 26 with it on real detections. With true
  boxes it gives 156 against 36. The earlier cost (25 vs 38-39) was with the
  old matching order.
- **Outcome.** The option was not kept, and CMC stays on.

## Conclusions

1. **The detector matters most.** The VisDrone model at confidence 0.2 found 4-9×
   more of these cars than the COCO model at 0.4, at the same speed.
2. **The ByteTrack bug and the threshold values made every detector gap an ID
   jump.** Both are fixed.
3. **CMC is worth its 8 ms** for a moving drone: 61% fewer ID switches while it
   turns. At live X's frame rate it is already as good as a perfect estimate
   (section 5). Its cost is with a nearly still camera and many moving cars
   (the junction), where any translation correction loses IDs. Ignoring the
   detected cars did not change that.
4. **SAHI is not worth it live on the RK3588** with these models: it is three
   times slower and adds false boxes that cost IDs.
5. **CMC's inlier gate hurt at low frame rates.** It rejected most estimates in
   fast turns. It is now off in rknn-sim (the defaults: 4 inliers, no ratio).
6. **Giving active tracks first claim** on detections cut junction switches by
   a third (39 -> 26) on real detections. Most remaining switches need more
   than ByteTrack: cars out of view for seconds, and identical cars.
7. **VisDrone + ByteTrack + CMC, run on the Radxa, holds up:** 8 / 25 / 24 ID
   switches (convoy / junction / drone motion), down from 11 / 34 / 89 with
   the tracker as it was, at 25 frames/s. It is now the rknn-sim profile.
8. **Repeat runs and live X** (section 8): flights of one mission differ by up
   to ±6 ID switches, so only large differences between settings are real.
   Live X keeps up (about 25 frames/s), but the compressed video stream costs
   it 4-14 switches per flight against replays.

## Next

- **CMC** (section 5):
  - The inlier gate is now off in rknn-sim (A5X_RPI_Release; deploy it to the
    Radxa). Confirm with a run at 8 Hz.
  - A nearly still camera no longer needs special handling (section 8).
  - Only if X must run at low rates: smooth the estimate over time, or check
    it against gimbal telemetry.

- **Cars out of view for seconds** (section 6): keep predicting them while
  unseen and match on distance, or re-identification.
- **The junction's switches during the fast turn home** (90-110°/s).
- **Live image quality:** raise the sim's stream bitrate (or stream the
  near-lossless frames) and re-run the live flights, to separate compression
  from everything else.
- **More repeats per setting:** three recordings per mission show ±6 switches
  of spread; compare settings on all of them, not one.
- **SAHI false positives:** if SAHI is wanted later, raise its confidence and
  check the duplicate merge (`SAHI_MERGE_METHOD`, `SAHI_THRESH`).
