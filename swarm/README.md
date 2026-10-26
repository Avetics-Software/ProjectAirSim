# Swarm

A ground station that plans, flies and watches a swarm of PX4 drones carrying
gimbal cameras and companion computers running the X vision pipeline. It
comes with a simulation to develop and test it in: Project AirSim over
photorealistic Cesium tiles of Singapore (Seletar by default), with moving
people and vehicles to search for.

The same ground station code flies the simulated swarm and real drones.

```bash
# Simulation: two drones, the first with a Radxa running X
./run_cesium_sim.sh --qgc --drones 2 --pi-ip 192.168.199.99

# Real drones, each with its own Radxa: fill in config.yml's drones and mavlink, then
./run_real.sh
```

## Why not just QGroundControl?

QGroundControl is built for one operator flying one aircraft at a time. It
can connect to several, but you still fly them one by one: select a drone,
plan its mission, upload it, switch to the next. This ground station tasks the
drones as one swarm.

| | QGroundControl | This ground station |
|---|---|---|
| Searching an area | One survey per drone; you work out the split | Draw the area once; it is [split between the drones](docs/features/area-search.md) and each gets a PX4 mission |
| Keeping drones apart | Nothing; each PX4 knows nothing of the others | [Height layers, a crossing check and a separation monitor](docs/features/deconfliction.md) |
| Vision and tracking | Shows video | [Click-to-track and gimbal control through X](docs/features/vision-tracking.md) on each drone's companion |
| Commanding many drones | Mostly one at a time | [`all takeoff`, `1,3 goto`, one-click buttons](docs/features/swarm-commands.md) |
| Safety across the swarm | Per-vehicle warnings, one vehicle at a time | [Every drone's health on one screen, alerts, hold-all / return-all, low-battery and lost-link responses, a geofence for all](docs/features/safety.md) |
| Testing | Separate from any simulation | [The same code flies the sim and real drones](docs/features/sim-to-real.md) |
| Extending | C++/Qt, a large codebase | [A small Python package](docs/features/architecture.md): detections, ReID and new search patterns go straight in |

QGroundControl keeps its place next to it: calibration, parameters, firmware,
flight logs, and flying a drone by hand. Both share the MAVLink link, and the
ground station never fights QGC for control.

## Documentation

- [Simulation guide](docs/simulation.md): running the sim, its options,
  objects and moving actors, cameras, the spotter, companions, ports
- [Ground station](gcs/README.md): running it against real drones, the fleet
  file, what to check before flying
- Features:
  [safety](docs/features/safety.md) ·
  [area search](docs/features/area-search.md) ·
  [deconfliction](docs/features/deconfliction.md) ·
  [vision and tracking](docs/features/vision-tracking.md) ·
  [swarm commands](docs/features/swarm-commands.md) ·
  [sim to real](docs/features/sim-to-real.md) ·
  [architecture](docs/features/architecture.md)
- [Roadmap](docs/roadmap.md): what is still missing, in priority order
- [Repeatable missions](docs/features/missions.md): uploaded flights with aircraft
  headings, camera pitch/yaw, and per-run logs
- [MOT experiments](docs/features/mot-experiments.md): record missions past
  scripted traffic, replay them through X's detector and ByteTrack, score ID
  switches; first results in [tracker and CMC experiments](docs/tracker-cmc-experiments.md)

## Layout

| Path | What |
|---|---|
| `gcs/` | The ground station. No Project AirSim imports; copy it to a field laptop |
| `run_cesium_sim.sh`, `px4_gimbal_cesium.py` | The simulation launcher and the script that runs the world, PX4s and the ground station |
| `run_real.sh` | Preflight checks, then the ground station for real drones |
| `px4_sitl.py` | Starts and stops one PX4 SITL per drone |
| `payload_mavlink.py`, `payload_video.py`, `camera_properties.*` | The simulated gimbal camera: MAVLink camera, RTSP video, lens and zoom |
| `spawned_objects.py`, `moving_actors.py`, `actor_editor.*`, `config_*.yml` | Objects, and people and vehicles moving along roads, to search for |
| `spotter_control.py`, `shared_frame.py` | The spotter: a person walked around the world, with a first-person view |
| `config.yml` | The drones (name, sysid, video, companion), the real drones' MAVLink link, and the ground station's settings (swarm, deconfliction, safety, controls); read by both launchers |
| `sim_config/` | Scene and robot configs (the `_custom` ones are generated each run) |
| `docs/` | This documentation |

The Unreal project and the Project AirSim client this builds on are in
`../unreal/` and `../client/python/projectairsim/`.

## Playback (to view the ground truth bounding boxes)

```bash
conda activate projectairsim
cd swarm
python gt_playback.py ../datasets/sim_gt/20261023_144412 --drone 2 --only-boxes
```
