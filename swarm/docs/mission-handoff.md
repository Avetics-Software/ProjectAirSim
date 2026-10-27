# Mission feature handoff — 2026-10-23

> **Historical note** from the session that started #32, kept for its record of
> the debugging. The current state is in [missions](features/missions.md) and
> [MOT experiments](features/mot-experiments.md).

> **Update, later the same day:** the blocking items below are resolved and
> validated in the sim (see `docs/features/missions.md`, "Validation"):
> gimbal identity now comes from PX4's first autopilot heartbeat
> (`autopilot_system_id_` in `mavlink_api.cpp`, replacing the experimental
> `getTargetSystemId()` patch); the sim gimbal handles PX4's zero-rate
> mission angles and turns within its yaw travel; each gimbal step claims the
> gimbal for the autopilot; `look_at` is re-aimed at each waypoint;
> `MIS_MNT_YAW_CTL 1` keeps ROI off the aircraft heading. Still open: a live
> Radxa/X mission test and real drones. #32 stays open until the user says.

## User request and issue

Implement repeatable uploaded PX4 missions with aircraft heading and camera
pitch/yaw control. X must stand down during missions. **Do not close the issue
until it has been tested in the simulator.** The user asked to wrap up because
of the session usage limit, then requested this handoff.

Existing issue: https://github.com/Avetics-Software/ProjectAirSim/issues/32
It remains OPEN. No duplicate was created. #23 already excludes scripted
flights and retains saved search areas/targets/geofences.

## Local changes

Branch: `cesium-px4-gimbal`. Nothing committed or pushed. The previous Claude
session left uncommitted mission work; this session extended it. Preserve all
working-tree changes.

- `gcs/mission.py`: YAML validation/compiler, upload/start/stop/clear, run CSVs
  and trajectory/heading/gimbal comparisons, completion/interruption handling.
- `gcs/station.py`, `companion.py`, `dashboard.py`: integration, pre-start
  tracking/jog cancellation and manual gimbal input blocking during missions.
- `gcs/mavlink_link.py`: native item upload/encoding, mission-state feedback,
  5 Hz gimbal feedback request.
- `gcs/minimap.py`: camera-step labels.
- `gcs/swarm.py`, `safety.py`: varying-height mission ceiling metadata.
- `px4_gimbal_cesium.py`: seeded actor reset and upload invalidation on reload.
- `docs/features/missions.md`, `missions/example.yml`: guide and example.
- `tests/test_mission.py`: **13 tests passed**, including request/ack protocol,
  command encoding, lifecycle/restart, companion input and height/fence checks.

Test command (sandbox asyncio thread shutdown hangs; run outside sandbox):

```bash
~/miniconda3/envs/projectairsim/bin/python -B -m unittest discover -s tests -v
```

## Live simulator results: feature NOT accepted yet

Started the actual Unreal/Cesium GUI, dashboard and two PX4 instances. Native
mission uploads succeeded, both aircraft armed/took off, followed waypoints,
changed aircraft headings and returned/landed on command. The first mission
also exercised camera yaw lock and ROI. Hovering mission completion reached
HOLD, but one successful finish was logged as `mission left`; completion
classification should be refreshed after exiting the monitoring loop.

**Blocking bug: multi-drone gimbal identity/telemetry.** Original simulator code
hardcodes MAVLink system ID **1** in two places in
`../vehicle_apis/multirotor_api/src/mavlink_api.cpp`:

- `SetupGimbalConnection`, around line 1465: gimbal node source system.
- `SendGimbalDeviceInformation`, around line 2439: `msg.sysid`.

In the first flight, Drone1 had gimbal feedback and Drone2 logged `nan` for
both camera angles. Rendered Drone2 attitude did change, so missing telemetry
must not be confused with proof that its camera never moved.

**Current experimental native patch is unsuccessful and remains in the
working tree / built plugin.** It replaces the node's fixed system ID with
`mav_vehicle_->getTargetSystemId()` and information `msg.sysid` with
`gimbal_node_->getLocalSystemId()`. After rebuilding and restarting, BOTH
drones' mission logs had `nan` gimbal feedback. Do not treat this patch as
validated. The control UDP connection may not have learned its target yet:
`getTargetSystemId()` starts at -1, which can become system ID 255.

Next investigate taking the system ID from the **HIL connection**
(`connection_->getTargetSystemId()`) or recording the actual PX4 heartbeat's
source ID, with proper startup/lifetime handling. `ConnectVehicle()` waits
for `send_params_`, set by the first PX4 heartbeat on HIL, before calling
`SetupGimbalConnection`. `connection_info_.vehicle_sysid` is the simulator
endpoint ID (default 135), NOT the actual PX4 ID; do not substitute it.

First flight also appeared to leave Drone1's final camera at neutral after
ROI was cleared, despite a later explicit -45 degree camera step. Recheck
using rendered attitude and correct telemetry after fixing identity. Do not
assume all trailing camera steps execute correctly. PX4 automatically
neutralises/releases gimbal control around mode changes. No speculative
compiler changes for this were applied.

PX4's `MIS_MNT_YAW_CTL` defaults can make ROI rotate the aircraft as well as
the camera; investigate value 1 for independent camera ROI versus aircraft
heading. No parameter change was made for this yet.

## Reproduction and artifacts

```bash
LOG_DIR=/tmp/swarm-mission-validation PX4_KEEP_LOGS=1000000 \
  ./run_cesium_sim.sh --qgc --drones 2
```

Large keep count preserves existing PX4 flight logs. The session inherited
an older Claude simulator, stopped it, and launched its own fresh instances.
The latest simulator was commanded `all rtl` before wrapping up.

Test mission files (temporary; copy into repo if needed):

- `/tmp/swarm-mission-validation.yml`: two drones, headings, body yaw,
  compass/yaw lock, ROI/clear, and final camera dwell.
- `/tmp/swarm-camera-acceptance.yml`: same core camera/heading test without ROI.

First flight (original native plugin):
`runs/acceptance_20261023_171834_769403563.csv`

Second flight (unsuccessful experimental native identity patch):
`runs/camera_acceptance_20261023_172823_575959658.csv`

Logs: `/tmp/swarm-mission-validation/{sim.log,px4.log,px4_1.log}`.
Rendered camera/drone attitude logs in the same directory:
`attitude_Drone1_171925.csv`, `attitude_Drone2_171925.csv`,
`attitude_Drone1_172823.csv`, `attitude_Drone2_172823.csv`.
Logs may be buffered until the attitude recording finishes.

Build commands already succeeded (repeat after correcting native patch):

```bash
cd /home/mh/Documents/ProjectAirSim
cmake --build build/linux64/UnrealEngine-5.7/Release --target multirotor_api -j 4
/home/mh/UnrealEngine-5.7/Engine/Build/BatchFiles/Linux/Build.sh \
  BlocksEditor Linux Development \
  -project=/home/mh/Documents/ProjectAirSim/unreal/Blocks/Blocks.uproject \
  -NoHotReloadFromIDE -MaxParallelActions=4
```

Native source/build locations are outside the writable `swarm/` root and
required sandbox approval. The experimental edit script is
`/tmp/swarm-fix-gimbal-identity.py`; it is not idempotent after its first run.

## X stand-down and GitHub access

The sibling `/home/mh/Documents/A5X_RPI_Release` already has uncommitted
Claude changes implementing `MAVLINK_STAND_DOWN_IN_MISSION` in
`X/utils/gimbal_api/mavlink_mount.py`, enabled in the `rknn-sim` profile.
This session did not edit that repository. A stand-in heartbeat check verified
mission mode suppresses commands/reclaims and HOLD resumes control. **No live
Radxa/X test was performed.** An async question asked whether a Radxa running
the updated build was available; no answer had arrived at handoff.

GitHub connector reads work but issue edits returned HTTP 403, resource not
accessible by integration. Local `gh issue view 32 --repo
Avetics-Software/ProjectAirSim` works outside the network sandbox. No GitHub
issue update was completed in this session. Update #32 with actual validation
results once the flight test passes, and only then consider closing it.
