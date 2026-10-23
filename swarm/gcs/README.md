# Swarm ground station

The dashboard, swarm control, deconfliction and companion-computer link from
the Cesium sim, for real PX4 drones. It reaches the drones only through MAVLink,
their video streams and their companions' UDP, so the sim and real drones run
the same code:

- **Sim:** `./run_cesium_sim.sh --qgc --drones 3` (settings in `swarm/config.yml`); see
  [docs/simulation.md](../docs/simulation.md).
- **Real drones:** `python -m gcs --fleet my_fleet.yml`

Nothing here imports Project AirSim. To use it on a field laptop, copy this
folder, then:

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
    sysid: 1                     # its PX4's MAV_SYS_ID
    video: rtsp://192.168.144.25:8554/main
    companion: 192.168.199.99    # Radxa running X
  - name: Drone2
    sysid: 2
    mavlink: udpin:0.0.0.0:14560 # a drone can have its own link instead
```

Every drone needs a unique `MAV_SYS_ID` on its PX4.

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

- **Firmware.** Gimbal control assumes PX4 with its gimbal manager
  (`MNT_MODE_IN=4`, `MNT_MODE_OUT=2`). An ArduPilot drone, or a gimbal driven
  directly by the companion (e.g. a DS60), needs checking first.
- **Set each PX4's own failsafes:** `NAV_DLL_ACT` / `COM_DL_LOSS_T` (link
  loss), the battery thresholds and `COM_LOW_BAT_ACT`, and upload the
  geofence. The ground station's responses and separation monitor come on top
  and only work while the link holds; the monitor was tuned on the sim's clean
  GPS (its margins widen with reported GPS inaccuracy). Treat them as advisory,
  not as the safety system.
- **Link bandwidth.** The monitor needs positions several times a second from
  every drone. A low-rate radio shared by many drones may not deliver that;
  watch the `position N/s` rate in `status`.

## Files

| File | What |
|---|---|
| `__main__.py` | `python -m gcs --fleet ...`: the ground station for real drones |
| `station.py` | `GroundStation` (selection, console, map, status) and `DroneCamera` |
| `dashboard.py` | The Tk dashboard window |
| `mavlink_link.py` | `MavConnection` (a link shared by several drones) and `VehicleLink` (one drone) |
| `swarm.py`, `search_planner.py` | Swarm commands, area search, deconfliction |
| `safety.py`, `safety_panel.py` | Health alerts and responses; the safety drawer (F2) |
| `companion.py`, `video.py` | X on a companion computer; RTSP video readers |
| `minimap.py`, `road_routes.py` | The map and its OpenStreetMap roads |
| `fleet.py`, `fleet.example.yml` | The fleet file: drones and settings sections |
| `settings.py` | Swarm, deconfliction and control settings (sections of the fleet file) |

The sim launcher (`px4_gimbal_cesium.py`) builds a `GroundStation` with sim
cameras (`SimPayload`) and adds its own commands, map objects and status
lines through the station's hooks.
