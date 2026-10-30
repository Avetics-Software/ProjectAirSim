# Ports

Every network port the swarm sim uses, per drone and per autopilot, so a
companion computer (a Pi or Radxa running X) or another tool can match them.
Drone numbers below are **N = 0 for Drone1**, 1 for Drone2, and so on. Each
drone's MAVLink system id is **N + 1**, whatever its autopilot.

The numbers come from `aircraft/models.yml` (`autopilots:` ports, stride and
instance offset), PX4's own `px4-rc.mavlink`, `ardupilot_sitl.py`,
`payload_mavlink.py` and `run_cesium_sim.sh`. If you change one there, change
it here.

## What a companion computer needs

This is the short version for the Pi. The rest of the page is the detail.

| Direction | Port | What |
|---|---|---|
| autopilot → Pi | UDP **15540** | MAVLink telemetry (X's `TELEMETRY_MAVLINK_PORT`) |
| autopilot → Pi | UDP **15541** | MAVLink for the gimbal (X's `GIMBAL_MAVLINK_PORT`) |
| Pi pulls from laptop | TCP **8554** | the sim camera: `rtsp://<laptop LAN IP>:8554/gimbal1` for Drone1, `/gimbal2` for Drone2, ... (X's `INPUT_RTSP`) |
| ground station → Pi | UDP **5556** | click-to-track, control and heartbeat (X's `GCS_INPUT_PORT`) |
| laptop pulls from Pi | TCP **8554** | X's annotated video, `rtsp://<Pi IP>:8554/tracker` (the Pi's own MediaMTX) |

- **X listens on both MAVLink ports** (`udpin` on `MAVLINK_URL`, 0.0.0.0) and
  replies to whichever address the autopilot sent from. So the Pi needs no
  autopilot ports configured: only its IP goes in the swarm config
  (`companion:` per drone, or `--pi-ip`).
- **Different ports on the Pi:** start the sim with `--pi-ports P1,P2` and set
  the same numbers in X's profile. `--pi-ports` applies to every drone's
  companion.
- **One Pi per drone.** Every drone sends to the same two ports on its
  companion, so two drones can't share one Pi at the default ports.
- X's profiles also name `GCS_OUTPUT_PORT 5557`; the ground station doesn't
  listen on it.

## Per drone, by autopilot

### PX4 (`quad_px4`)

PX4's instance is N. PX4 offsets every port by its instance.

| Link | PX4 side | Other side | Notes |
|---|---|---|---|
| Simulator (lockstep) | connects to TCP **4560 + N** | sim listens | `tcp-port` |
| QGC | UDP **18570 + N** | QGC listens on UDP 14550 | add one server per drone in QGC: `127.0.0.1:18570`, `18571`, ... |
| API / offboard | UDP 14580 + N | sim, UDP **14540 + N** | `control-port` |
| Payload (ground station + MAVLink camera) | UDP 14280 + N | swarm script listens on UDP **14030 + N** | `payload_mavlink.py` |
| Gimbal | UDP 13030 + N | sim, UDP **13280 + N** | `gimbal-port`; the sim acts as the gimbal device |
| Companion computer | UDP **14590 + 10N** and **14591 + 10N** | Pi, UDP 15540 and 15541 | one local port per `--pi-ports` entry; added only with a companion |

### ArduPilot (`x8_arduplane`, `quad_arducopter`)

ArduPilot's instance is **I = N + 5**, which keeps clear of another ArduPilot
on instance 0, such as the packaged Avetics sim. `-I` offsets ArduPilot's own
ports by 10 × I.

| Link | ArduPilot side | Other side | Notes |
|---|---|---|---|
| Simulator (JSON, lockstep) | sends to UDP **9002 + 10I** (9052 + 10N) | sim listens there | `local-host-udp-port` |
| SERIAL0 | TCP **5760 + 10I** (5810 + 10N), listening | unused | ArduPilot's console; attach MAVProxy here if needed |
| SERIAL1: payload (ground station + camera) | sends to UDP **14030 + N** | swarm script listens there | the same port as PX4's payload link |
| SERIAL2: QGC | sends to UDP **14550** | QGC listens there | no QGC server to add: ArduPilot drones appear by themselves |
| SERIAL5, SERIAL6: companion | send to Pi UDP **15540**, **15541** | Pi listens there | `SERIALn_PROTOCOL 2`; added only with a companion; the source port is whatever the OS picks |
| SITL internals | e.g. UDP 9005 + 10I (IRLock) | none | opened by ArduPilot SITL itself; nothing in the swarm uses them |

### Example: three drones

Drone1 `quad_px4`, Drone2 `quad_arducopter`, Drone3 `x8_arduplane`, each with a companion:

| | Drone1 (PX4, N = 0) | Drone2 (ArduCopter, N = 1, I = 6) | Drone3 (ArduPlane, N = 2, I = 7) |
|---|---|---|---|
| System id | 1 | 2 | 3 |
| Simulator | TCP 4560 | UDP 9062 | UDP 9072 |
| Payload link (swarm script) | UDP 14030 | UDP 14031 | UDP 14032 |
| QGC | server 127.0.0.1:18570 | to 14550 | to 14550 |
| Companion MAVLink | 14590/14591 → Pi 15540/15541 | → Pi 15540/15541 | → Pi 15540/15541 |
| Sim camera for its Pi | `rtsp://<laptop>:8554/gimbal1` | `.../gimbal2` | `.../gimbal3` |

## Shared by the whole sim

| Port | What |
|---|---|
| TCP **8989** | Project AirSim topics (images, poses), from Unreal |
| TCP **8990** | Project AirSim services (load scene, spawn objects, ...), from Unreal |
| TCP **8554** | MediaMTX RTSP on the laptop: `/gimbal1`, `/gimbal2`, ... (`mediamtx_rtsp.yml`); run_cesium_sim.sh reuses one already running |
| TCP **8889** | MediaMTX WebRTC, for watching in a browser: `http://127.0.0.1:8889/gimbal1` |
| UDP **14550** | QGC listens; ArduPilot drones send to it, PX4 drones are reached through their 18570 + N servers |

At start, run_cesium_sim.sh refuses to run if 8989, 8990 or any PX4 drone's
4560 + N is already in use, usually by a simulator or PX4 still running.

## Real drones (run_real.sh)

The sim's per-drone ports don't apply to real drones. The ground station
listens on one MAVLink link, `mavlink:` in the fleet file, as system id
`gcs_sysid` (245). `config.yml` uses `udpin:0.0.0.0:14560` so QGC can keep 14550;
`gcs/fleet.example.yml` shows 14550.
Each drone is told apart by its own system id. A drone's companion is reached
the same way as in the sim: UDP 5556 for commands and `rtsp://<Pi IP>:8554/tracker`
for X's video. See `gcs/fleet.example.yml`.
