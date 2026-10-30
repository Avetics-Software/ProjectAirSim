# From the sim to real drones

The ground station in `gcs/` doesn't know whether its drones are simulated.
It reaches them only through what a real drone offers too: MAVLink to PX4,
video streams, and the companion computers' UDP. So whatever is flown and
tuned in the Cesium sim at Seletar is the same code that flies the real swarm.

## The same fleet file for both

A fleet file lists the drones and how to reach each one
([`gcs/fleet.example.yml`](../../gcs/fleet.example.yml) has every key):

```yaml
mavlink: udpin:0.0.0.0:14550        # the link carrying every drone
drones:
  - name: Drone1
    sysid: 1
    video: rtsp://192.168.144.25:8554/main
    companion: 192.168.199.99
  - name: Drone2
    sysid: 2
    companion: 192.168.199.100
```

| | Sim | Real drones |
|---|---|---|
| Run | `./run_cesium_sim.sh --qgc` | `./run_real.sh` (preflight checks, then the ground station) |
| Config | `swarm/config.yml` (`--config FILE` for another) | the same file: fill in each drone's `sysid`, `video`, `companion` and the top-level `mavlink` |
| Settings | The same `swarm:`, `deconfliction:`, `safety:` and `controls:` sections | the same |
| Drones | One PX4 SITL per drone, named Drone1, Drone2, ... | Your PX4s; each needs its own `MAV_SYS_ID` |
| MAVLink | Each PX4 on its own UDP port (14030 + N) | One shared link (radio, mesh, mavlink-router), or one per drone |
| Video | The sim's GimbalCam over RTSP (`/gimbal1`, `/gimbal2`, ...) | Each drone's `video:` stream; `chase_video:` for a second camera |
| Companions | `companion:` (PX4 sends it MAVLink on 15540/15541) | `companion:` |
| Zoom | The sim's camera field of view | `MAV_CMD_SET_CAMERA_ZOOM` to the drone's MAVLink camera |
| Extra | Objects, moving actors, `goto` a new location | |

The sim reads the drones' companions and the settings sections; the MAVLink,
system id and video keys are for real drones.

## What is shared, and what isn't

Shared, in `gcs/`: the dashboard, swarm commands, area search,
deconfliction, the companion link, the map and status. All of it works from
MAVLink telemetry; in the sim the map's drone positions come from PX4's GPS
estimate, not the sim's ground truth, just as they would on real drones.

Sim only, around it:

- `px4_gimbal_cesium.py` loads the Cesium world, starts a PX4 per drone and
  builds the ground station with sim cameras (`SimPayload`).
- `payload_mavlink.py` pretends to be each drone's MAVLink camera, so QGC
  and X can find the sim's video and zoom it. A real drone's camera does this
  itself.
- Spawned objects, moving actors and the spotter, for something to search for.

## run_real.sh

```bash
./run_real.sh                 # config.yml next to it
./run_real.sh --check         # only the preflight checks
./run_real.sh --config field_day.yml
```

Before starting the ground station it checks, and prints one line per check:

- the config is valid for real drones (a MAVLink link, unique system ids)
- the MAVLink link can be opened (the UDP port is free, the radio is plugged in)
- each drone's PX4 is heard on it (listens 6 s, `--listen`)
- each drone's video stream answers, each companion answers ping and X's
  tracker video is up

A bad config, a busy port or a missing radio stops it; anything else is a
warning, so you can start before every drone is powered on. Each session's
log goes to `~/swarm_logs/gcs_<date>_<time>.log` (`LOG_DIR`). It uses the
`projectairsim` conda env's Python if present, else `python3` (`PYTHON`); a
field laptop needs `gcs/requirements.txt` and python3-tk.

## Sharing a link

On a real fleet, one radio or mesh usually carries every drone. `gcs/`
opens each link once and picks out each drone's messages by its system id.
Replies addressed to someone else (QGC's mission uploads, say) are ignored,
and the ground station identifies itself as system 245, not 255, so QGC can
share the link at the same time.

## Before flying it for real

See [gcs/README.md](../../gcs/README.md#before-flying-it-for-real):
gimbal control assumes PX4's gimbal manager; the separation monitor is
advisory and depends on the link; and it needs several positions per second
from every drone, so check the rate in `status` on your radio.
