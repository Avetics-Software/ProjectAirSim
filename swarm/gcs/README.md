# Swarm ground station

The dashboard, swarm control, deconfliction and companion-computer link from
the Cesium sim, for real PX4 or ArduPilot drones. It reaches the drones only through MAVLink,
their video streams and their companions' UDP, so the sim and real drones run
the same code:

- **Sim:** `./run_cesium_sim.sh --qgc --drones 3` (settings in `swarm/config.yml`); see
  [docs/simulation.md](../docs/simulation.md).
- **Real drones:** `./run_real.sh` (reads `swarm/config.yml`, runs preflight
  checks first; `--config FILE` for another), or `python -m gcs --fleet FILE`

Nothing here imports Project AirSim. To use it on a field laptop, copy this
folder (and, if any drone is not a PX4 multirotor, `aircraft/models.yml` in an
`aircraft/` folder beside `gcs/`: only each model's autopilot and capabilities
are read), then:

```bash
pip install -r gcs/requirements.txt    # plus python3-tk on Debian/Ubuntu
python -m gcs --fleet my_fleet.yml     # from the folder that contains gcs/
```

## The fleet file

`fleet.example.yml` shows the drone keys; the settings sections (`swarm:`,
`deconfliction:`, `safety:`, `controls:`) are the same as in the sim's
`../config.yml`, which lists every key with its default. In short:

```yaml
mavlink: udpin:0.0.0.0:14550     # the link carrying every drone (radio, mesh, mavlink-router)
gcs_sysid: 245                   # our MAVLink system id; not 255, so QGC can share the link
drones:
  - name: Drone1
    sysid: 1                     # its autopilot's MAV_SYS_ID (ArduPilot: SYSID_THISMAV)
    model: quad_px4              # its aircraft (aircraft/models.yml); the default
    video: rtsp://192.168.144.25:8554/main
    companion: 192.168.199.99    # Radxa running X
  - name: Drone2
    sysid: 2
    mavlink: udpin:0.0.0.0:14560 # a drone can have its own link instead
```

Every drone needs a unique system id on its autopilot. `model:` tells the ground
station which autopilot it speaks to and what the aircraft can do (a
fixed-wing circles instead of hovering and has no heading of its own); see
[aircraft](../docs/features/aircraft.md).

## What it does on real drones

The same dashboard and commands as the sim (see the sim guide's *Swarm* and
*Camera and gimbal* sections):

- **Status and map:** each drone's mode, arming, height, heading and speed,
  and its position on the minimap, from its MAVLink telemetry. `LINK LOST`
  shows once a drone's position is 3 s old. The map's origin is `home:` from
  the fleet file, or the first drone's first position fix. OpenStreetMap roads
  around it are loaded in the background and cached in `osm_cache/` next to
  this folder.
- **Flying:** take off, go to a point, heading, hold, land, return, and polygon
  area searches uploaded as PX4 missions, from the control bar or by typing
  commands (`help` lists them). The commands go to PX4 like QGC's would, so QGC
  can watch and take over at any time.
- **Repeatable missions:** upload YAML plans with aircraft heading and camera
  pitch/yaw or ROI, start them across the swarm, and compare run logs. X stands
  down with the matching companion configuration. See [missions](../docs/features/missions.md).
- **Deconfliction:** height layers, the crossing check and the separation
  monitor, as in the sim.
- **Safety:** each drone's battery, GPS and link on its status line, alerts,
  `HOLD ALL` / `RTL ALL` (Ctrl+H holds all), low-battery and lost-link
  responses, and a geofence uploaded to every PX4. The drawer (F2) has the
  detail; thresholds go in the fleet file's `safety:` section. See
  [docs/features/safety.md](../docs/features/safety.md).
- **Cameras:** the selected drone's `video` in the GimbalCam pane and
  `chase_video` (if any) in the ChaseCam pane. Without a companion, W/A/S/D
  steer the gimbal through PX4's gimbal manager and the zoom keys send
  `MAV_CMD_SET_CAMERA_ZOOM` to the drone's MAVLink camera (component 100).
- **Companions:** with `companion:`, the Tracker view, click-to-track and the
  gimbal keys go to X on that drone, as in the sim.

## Before flying it for real

- **Firmware.** Gimbal control goes through the autopilot's gimbal manager:
  PX4 with `MNT_MODE_IN=4`, `MNT_MODE_OUT=2`, or ArduPilot with a mount that
  speaks it (the sim's ArduPlane uses `MNT1_TYPE 7`, a self-stabilising PWM
  gimbal; a real mount's type, travel and stops are yours to check). A gimbal
  driven directly by the companion (e.g. a DS60) needs checking first. ArduPlane
  has been flown only in the sim.
- **Set each autopilot's own failsafes:** PX4's `NAV_DLL_ACT` / `COM_DL_LOSS_T`
  (link loss), ArduPilot's `FS_GCS_ENABL`, `FS_LONG_ACTN`, `FS_LONG_TIMEOUT`
  (and `MAV_GCS_SYSID`, the ground station's system id), the battery thresholds
  and `COM_LOW_BAT_ACT` (PX4) or `BATT_*` (ArduPilot), and upload the geofence. The ground station's responses and separation monitor come on top
  and only work while the link holds; the monitor was tuned on the sim's clean
  GPS (its margins widen with reported GPS inaccuracy). Treat them as advisory,
  not as the safety system.
- **Link bandwidth.** The monitor needs positions several times a second from
  every drone. A low-rate radio shared by many drones may not deliver that;
  watch the `position N/s` rate in `status`.

## Files

| File | What |
|---|---|
| `__main__.py` | `python -m gcs --fleet ...`: the ground station for real drones (`../run_real.sh` starts it) |
| `preflight.py` | The checks `run_real.sh` runs first: config, MAVLink link, heartbeats, video, companions |
| `station.py` | `GroundStation` (selection, console, map, status) and `DroneCamera` |
| `dashboard.py` | The Tk dashboard window |
| `mavlink_link.py` | `MavConnection` (a link shared by several drones) and `VehicleLink` (one drone) |
| `swarm.py`, `search_planner.py` | Swarm commands, area search, deconfliction |
| `mission.py` | Mission files: checking, conversion to mission items, upload, start, run logs and comparison; `python -m gcs.mission FILE` checks a file without the sim |
| `autopilot.py`, `airframe.py` | Where PX4 and ArduPilot differ on MAVLink, and what a drone's aircraft can do (hover or not) |
| `safety.py`, `safety_panel.py` | Health alerts and responses; the safety drawer (F2) |
| `sim_panel.py` | The sim settings drawer (F3): the camera (resolution, aperture, noise, distortion, vibration) and the time of day |
| `companion.py`, `video.py` | X on a companion computer; RTSP video readers |
| `minimap.py`, `road_routes.py` | The map and its OpenStreetMap roads |
| `fleet.py`, `fleet.example.yml` | The fleet file: drones and settings sections |
| `settings.py` | Swarm, deconfliction and control settings (sections of the fleet file) |

The sim launcher (`px4_gimbal_cesium.py`) builds a `GroundStation` with sim
cameras (`SimPayload`) and adds its own commands, map objects and status
lines through the station's hooks.
