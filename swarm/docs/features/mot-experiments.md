# MOT experiments: ID switches from a moving drone

How well X's detector and multi-object tracker (ByteTrack) keep each vehicle's
ID while the drone flies with its camera at a fixed angle, and what the drone
was doing whenever an ID jumped. The aim is to minimise ID switches.

The loop:

1. **Record:** fly a scripted mission in the sim past scripted traffic. Every
   camera frame is recorded with the drone's metadata and the true box of
   every vehicle.
2. **Replay:** run the recording through X's own detector and ByteTrack on the
   Radxa (`mot_replay.py`), every frame in order.
3. **Score:** `mot_eval.py` reports ID switches, IDs per vehicle and IDF1, and
   lists each switch with the drone's attitude, rates and speed at that
   moment, plus a likely cause.
4. **Tune:** re-run only the tracker on the recorded detections with other
   settings (seconds, no re-flying), and compare.

Missions do the flying ([missions](missions.md)); the gimbal steps hold the
camera at a fixed angle, and X keeps detecting and tracking (standing down
only stops its gimbal commands).

## 1. Record

Start the sim with ground truth (it renders a segmentation image for the true
boxes), then fly one of the experiment missions:

```bash
./run_cesium_sim.sh --qgc --ground-truth
```
```text
mission load mot_convoy
mission upload 1
mission start 1
```

| Mission | Traffic (`missions/scenarios/`) | What it stresses |
|---|---|---|
| `mot_convoy` | 5 identical cars nose to tail, about 10 m gaps, 12 m/s | The drone flies beside the road at 9 m/s, camera fixed sideways. The cars drift through the frame at 3 m/s relative, about 30 s each, all five together for about 10 s: close identical targets |
| `mot_junction` | Two streams of 4 identical cars interleaving through a four-way junction | The drone watches the junction with the camera on a fixed compass heading, then crosses past it. Boxes cross and overlap |
| `mot_drone_motion` | 8 identical cars, 8 m/s, about 105 m long | The drone yaws ±45° at about 40°/s, dashes to 15 m/s and stops hard, then climbs to 57 m, with the camera fixed to the nose, while the convoy is in view (about 60–87 s). Stresses camera motion compensation |

Each mission names its own traffic (`actors:`), respawned at `mission start`.
The cars wait in formation along their route (`start_at_m`, `stop_short_m`)
and set off on a timer (`start_delay`) chosen so they reach the drone's view
once it is in place.

The run is written to `runs/<mission>_<run>/`:

| File | Contents |
|---|---|
| `gimbal_Drone1.mp4` | Every rendered GimbalCam frame once, in order (1920×1080, near-lossless) |
| `frames_Drone1.csv` | Per frame: index, sim and wall time, the camera's world pose (NED, quaternion and Euler: the gimbal's actual attitude), field of view, zoom, PX4's gimbal angles, mode, mission item, box count |
| `boxes_Drone1.csv` | Per frame, each vehicle in view: its name (true identity), box, visible pixels, cut by the edge |
| `telemetry.csv` | The drone's true kinematics at about 50 Hz (sim time): position, lat/lon/height, roll/pitch/yaw, their rates, velocity, acceleration, mode, mission item |
| `actors.csv` | Every vehicle's position and heading at 10 Hz |
| `meta.json` | Camera, frame counts (and any dropped: should be 0), copies of the mission and traffic files |

With the ground-truth render the sim produces about 16 frames/s (sim time
runs at about real time); the CSVs carry each frame's true time.

## 2. Replay on the Radxa

Copy the run folder to the Radxa, then from X's directory:

```bash
A5X_PROFILE=rknn-sim python3 mot_replay.py ~/runs/mot_convoy_<run>
```

It reads the video frame by frame, resizes to `INPUT_WIDTH`×`INPUT_HEIGHT`,
and runs the detector and tracker exactly as `detection.py` builds them
(`ATR_MODEL`, `ATR_RUNTIME`, `ATR_CONFIDENCE`, `ATR_LABELS`, `OBJECT_TRACKER`,
`BYTETRACK_*`, `OBJECT_TRACKER_CMC`). Output goes to
`<run>/tracker_<time>/`: `tracks.csv`, `detections.csv`, `run_info.json`.

- `--rate HZ`: process only the frames a live X at HZ would. Live X tracks the
  newest frame each time its detector is free, and `BYTETRACK_MAX_LOST` and
  `MIN_HITS` count processed frames, not seconds, so match the live rate when
  comparing with flight behaviour.
- `--set KEY=VALUE`: override a setting, e.g. `--set BYTETRACK_MAX_LOST=60`.
- `--detections FILE`: skip the detector and reuse a previous replay's
  `detections.csv`. ByteTrack only, in seconds, on the Radxa or the laptop.
- `--oracle [--jitter PX --drop P]`: feed the true boxes (optionally spoiled)
  as detections, to test the tracker alone.

## 3. Score

On the laptop (the `projectairsim` env with scipy and matplotlib):

```bash
python mot_eval.py runs/mot_convoy_<run> runs/mot_convoy_<run>/tracker_*
```

Per tracker: ID switches, fragmentations (lost, then picked up again with the
same ID), IDs per vehicle (1 is perfect), IDF1, MOTA, MOTP, recall, false
positives, and the detector's own recall and precision, which separate
detector misses from tracker mistakes. With several trackers a comparison
table follows.

In each tracker folder:

- **`events.csv`:** one row per ID switch or fragmentation, with:
  - the drone's attitude, rates, speed, acceleration and height
  - how fast the camera was turning
  - the vehicle's box size, distance to the frame edge and to the nearest
    other vehicle
  - frames since it was last detected
  - a likely cause: `detector-gap`, `crowding`, `camera-motion`,
    `frame-edge` or `other`
- **`summary.json`:** the totals.
- **`timeline.png`:** each vehicle's track IDs over time (one colour per ID,
  red marks at switches) above the drone's yaw rate, speed and the camera's
  turn rate.

Only vehicles are scored by default (`--kinds`), true boxes narrower than
8 px are ignored (`--min-px`), and a match needs IoU 0.5 (`--iou`).

## Step by step: run it yourself on the Radxa

The same recordings and code as the
[2026-10-26 experiments](../tracker-cmc-experiments.md). The three
recordings are on the laptop in `swarm/runs/` (not in git) and already copied
to the Radxa:

| Recording | Folder (laptop `swarm/runs/`, Radxa `~/mot_runs/`) |
|---|---|
| convoy | `mot_convoy_20261026_110740_184801946` |
| junction | `mot_junction_20261026_112050_117478778` |
| drone motion | `mot_drone_motion_20261026_112704_725029113` |

**1. Put the current code on the Radxa** (laptop, A5X_RPI_Release on
`swarm-sim`, up to date):

```bash
cd ~/Documents/A5X_RPI_Release/X
./deploy.sh --profile rknn-sim --dir radxa@192.168.199.99:~/Code/X_sim/ --code-only
```

This copies the tracker, CMC, `mot_replay.py` and the rknn-sim
`config_cv.yml`: VisDrone model, confidence 0.2, corrected thresholds, CMC on.

**2. Replay a recording on the Radxa:**

```bash
ssh radxa@192.168.199.99
cd ~/Code/X_sim
A5X_PROFILE=rknn-sim venv/bin/python mot_replay.py ~/mot_runs/mot_convoy_20261026_110740_184801946 --tag mine
```

- **Time:** about 1.5-2 minutes per recording without SAHI.
- **At the end** it prints a per-stage timing table and the output folder,
  `~/mot_runs/<recording>/tracker_<date>_<time>_mine/`, which holds
  `tracks.csv`, `detections.csv`, `timing.csv` and `run_info.json`.

To reproduce a row of the experiments, add `--set` overrides (repeatable):

| Setting | Extra options |
|---|---|
| Current (VisDrone, 0.2, CMC on) | none |
| CMC off | `--set OBJECT_TRACKER_CMC=false` |
| With SAHI | `--set SAHI_ENABLED=true --set PARALLELISM_MODE=1` |
| COCO model at 0.4 | `--set ATR_MODEL=./model/yolov8n.rknn --set ATR_CONFIDENCE=0.4` |
| Old thresholds | `--set BYTETRACK_MATCH_THRESH=0.8 --set BYTETRACK_R1_MATCH_THRESH=0.5` |

To try tracker settings only, without re-running the detector (seconds
instead of minutes), feed a previous run's detections:

```bash
A5X_PROFILE=rknn-sim venv/bin/python mot_replay.py ~/mot_runs/<recording> \
    --detections ~/mot_runs/<recording>/tracker_<...>_mine/detections.csv \
    --set BYTETRACK_MAX_LOST=60 --tag lost60
```

The "before the fix" rows used X's tracker as of A5X_RPI_Release commit
`569f964`. Check out `X/utils/trackers/mot.py` from it to reproduce them.

**3. Copy the result back** (laptop), into the same recording's folder, which
has the true boxes and telemetry for scoring:

```bash
scp -r "radxa@192.168.199.99:mot_runs/mot_convoy_20261026_110740_184801946/tracker_*_mine" \
    ~/Documents/ProjectAirSim/swarm/runs/mot_convoy_20261026_110740_184801946/
```

**4. Score it** (laptop):

```bash
conda activate projectairsim
cd ~/Documents/ProjectAirSim/swarm
python mot_eval.py runs/mot_convoy_20261026_110740_184801946 \
    runs/mot_convoy_20261026_110740_184801946/tracker_*_mine
```

- **Printed:** ID switches, IDs per car, IDF1, MOTA, and the detector's recall
  and precision.
- **Written** into the tracker folder: `events.csv`, `summary.json`,
  `timeline.png`.
- **Comparing:** list several tracker folders to get a comparison table, e.g.
  next to `tracker_*_vd_newthresh_cmc` (the 2026-10-26 run with the current
  settings).

**A new recording instead.** Start the sim with
`./run_cesium_sim.sh --qgc --ground-truth` and fly a mission (`mission load
mot_convoy`, `mission upload 1`, `mission start 1`). Copy its video and frame
list to the Radxa:

```bash
ssh radxa@192.168.199.99 mkdir -p mot_runs/<run>
scp runs/<run>/gimbal_Drone1.mp4 runs/<run>/frames_Drone1.csv radxa@192.168.199.99:mot_runs/<run>/
```

Then follow steps 2-4. Copy `boxes_Drone1.csv` too only for `--oracle`
runs on the Radxa.

## Finding: a missed frame cost a new ID (fixed in X)

The oracle replay of `mot_convoy`, with the true boxes 3 px jittered and 10%
dropped (detector flicker), gave each car about 42 IDs. In X's ByteTrack a
track that missed one frame became "lost", and lost tracks could only be
matched again by **low-confidence** detections. The car's next normal
detection started a new track. ByteTrack proper matches high-confidence
detections against lost tracks too.

Fixed in `X/utils/trackers/mot.py` (with tests in `tests/test_mot_replay.py`):

| Same detections | ID switches | IDs per car | IDF1 |
|---|---|---|---|
| X's ByteTrack before | 396 | 57.6 | 0.07 |
| After the fix | 101 | 14.1 | 0.72 |

Clean true boxes give the same result either way (the remaining switches are
on two parked cars at the frame edge).

Also: X's ReID (`REID_ENABLED`) only annotates the overlay (`ReID→id`); it
never changes `track_id`, so it cannot reduce ID switches as it stands.

## Results on the Radxa (2026-10-26)

The full write-up, with every run, the causes and the timing, is
[Tracker and CMC experiments](../tracker-cmc-experiments.md).

X's own detectors on the three recordings (RK3588, every frame), scored
against the true boxes. Detector recall and precision are per box; IDs per car
counts the scenario's moving cars only.

| Detector | Recall | Precision | ms/frame | Convoy IDF1 | Junction IDF1 | Drone motion IDF1 |
|---|---|---|---|---|---|---|
| yolov8n (COCO), conf 0.4: as before | 6-14% | 71-95% | 36 | 0.10 | 0.07 | 0.10 |
| yolov8n, conf 0.2 | 12-28% | 44-85% | 36 | 0.13 | 0.08 | 0.12 |
| yolov8n, conf 0.2 + SAHI tiles | 59-77% | 44-56% | 100 | 0.67 | 0.24 | 0.31 |
| **yolov8n_visdrone, conf 0.2** | **52-67%** | **86-95%** | **30** | 0.53 | 0.36 | 0.22 |
| yolov8n_visdrone, conf 0.2 + SAHI | 89-93% | 50-76% | 78 | 0.61 | 0.43 | 0.42 |

- **The cars are small for a 640 px model:** about 88×42 px in the 1920×1080
  frame, and 33×15 px at the junction.
  - The COCO model barely saw them at 0.4.
  - The VisDrone model sees most of them with few false detections, fast
    enough for live use.
  - SAHI tiling finds more still, but at 10-13 frames/s and with many more
    false and duplicate boxes, which multiply ID switches.
- **ByteTrack on the VisDrone detections (laptop, `--detections`):**

| Tracker | Convoy IDs/car | Junction IDs/car | Drone motion IDs/car | Drone motion ID switches |
|---|---|---|---|---|
| Old IoU thresholds (stage 2 needed IoU 0.8) | 2.2 | 4.75 | 11.0 | 89 |
| Corrected thresholds (stage 1 IoU 0.2, stage 2 0.5) | 2.2 | 3.62 | 8.75 | 71 |
| Corrected + camera motion compensation | 2.2 | 3.75 | **3.75** | **28** |

The rknn-sim profile now uses `yolov8n_visdrone.rknn`, `ATR_CONFIDENCE`
0.2, the corrected thresholds and `OBJECT_TRACKER_CMC: true`.

## Not yet done

- **The Radxa runs:** the real RKNN detector over these recordings, then
  ByteTrack sweeps on its detections.
- **Live logging:** writing X's tracks during a live sim flight, for checking
  against a recording. It needs the Radxa's and laptop's clocks in sync.
