# Tracker and CMC experiments (2026-10-26)

How well X keeps each vehicle's ID when the drone flies with its camera fixed,
and what it took to improve it: the detector, ByteTrack's matching, and camera
motion compensation (CMC). The method and tools are in
[MOT experiments](features/mot-experiments.md). This page is the record of
the first round of experiments and what they showed. To repeat them yourself, see
[step by step: run it yourself on the Radxa](features/mot-experiments.md#step-by-step-run-it-yourself-on-the-radxa).

**Outcome.** On these recordings the detector was the bottleneck:
- **Detector:** the COCO `yolov8n` found 6-14% of the cars. `yolov8n_visdrone`
  at confidence 0.2 finds 52-67%, with 86-95% of its boxes correct, at
  25-30 ms per frame.
- **Tracker:** X's ByteTrack had a bug that gave a car a new ID after a single
  missed detection, and its IoU thresholds were set far too strict.
- **CMC:** it more than halves the IDs per car while the drone turns.

The rknn-sim profile now uses VisDrone, confidence 0.2, the corrected
thresholds and CMC (A5X_RPI_Release `042ac40`, `5cb3adf`).

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
  this frame's detections, which should help exactly the junction case.
- **It rejects weak fits:** a fit with too few inliers is not applied.

They are not measured yet.

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

## Conclusions

1. **The detector matters most.** The VisDrone model at confidence 0.2 found 4-9×
   more of these cars than the COCO model at 0.4, at the same speed.
2. **The ByteTrack bug and the threshold values made every detector gap an ID
   jump.** Both are fixed.
3. **CMC is worth its 8 ms** for a moving drone: 61% fewer ID switches while it
   turns, at a small cost when many cars move through a still view. The later
   CMC changes (ignoring detected vehicles) target that case.
4. **SAHI is not worth it live on the RK3588** with these models: it is three
   times slower and adds false boxes that cost IDs.

## Next

- **Measure the later CMC changes** (ignore detected vehicles, reject weak
  fits): deploy them to the Radxa first, then re-run.
- **Tune `BYTETRACK_MAX_LOST` and `MIN_HITS`** on the VisDrone detections,
  with `--rate` at live X's rate.
- **Look at the remaining causes in `events.csv`:** frame-edge (cars leaving
  and re-entering view) and crowding at the junction.
- **A live sim flight with X on the Radxa** using these settings, to confirm
  the replay results and the live frame rate.
- **Repeat each setting several times** to put error bars on the differences.
- **SAHI false positives:** if SAHI is wanted later, raise its confidence and
  check the duplicate merge (`SAHI_MERGE_METHOD`, `SAHI_THRESH`).
