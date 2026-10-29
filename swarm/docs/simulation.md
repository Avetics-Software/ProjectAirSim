# Simulation guide: PX4 and ArduPilot drones in a Cesium world

Flies PX4 quadrotors and ArduPilot fixed-wings (the Skywalker X8), each with a
gimbaled payload camera (GimbalCam), over real-world Google 3D tiles streamed
through Cesium for Unreal. The default spawn is Seletar Airport's runway,
Singapore. For what the project is and its other docs, see the
[README](../README.md). To fly your own scripted flight, start with
[Repeatable missions](features/missions.md#write-and-fly-your-own-mission).

## Prerequisites

- Unreal Engine 5.7 at `~/UnrealEngine-5.7`, with the Blocks editor built
  (Cesium for Unreal installed in `unreal/Blocks/Plugins/CesiumForUnreal`)
- PX4-Autopilot SITL built at `~/Documents/PX4-Autopilot`
- For the X8 (ArduPlane): ArduPilot's SITL built at `~/Documents/ardupilot`
  (`./waf configure --board sitl && ./waf plane`, ArduPlane 4.7); `ARDUPILOT_DIR`
  names another checkout
- Conda env `projectairsim` (Python client, pymavlink, PyYAML)
- For the spotter: UE's template Mannequin, copied in once with
  `./install_spotter_assets.sh` (then restart the simulator)
- A Cesium ion token in `~/.cesium_ion_token`:
  ```bash
  echo 'export CESIUM_ION_TOKEN=<token>' > ~/.cesium_ion_token && chmod 600 ~/.cesium_ion_token
  ```

## Running

```bash
cd ~/Documents/ProjectAirSim/swarm
./run_cesium_sim.sh [options]
```

`run_cesium_sim.sh` starts the simulator, a fresh PX4 SITL with the gimbal
enabled, an RTSP server if port 8554 is free, and `px4_gimbal_cesium.py` with
your options. Ctrl+C, or closing the simulator window, stops everything.

The sim starts paused until the Cesium ground under the drone has loaded
(about 10 s once tiles are cached; longer on a location's first visit), then
sets the drone on it.

| Command | What it does |
|---|---|
| `./run_cesium_sim.sh` | Scripted demo at Seletar: take off, climb to 40 m, sweep the gimbal, land |
| `./run_cesium_sim.sh --qgc` | Load the world and wait; fly and steer the gimbal from QGroundControl or the dashboard window |
| `./run_cesium_sim.sh --qgc --lat 1.3521 --lon 103.8198 --heading 90` | Same, spawning somewhere else |
| `./run_cesium_sim.sh --qgc --model x8_arduplane` | The Skywalker X8 fixed-wing (ArduPlane) instead of a quadrotor; see [aircraft](features/aircraft.md). Wait for `World ready` and its GPS (about 30 s), then `1 takeoff 40`, `1 goto LAT LON 40`, `1 hold`, `1 rtl`, or fly a [mission](features/missions.md) |
| `./run_cesium_sim.sh --qgc --config my_fleet.yml` | A mixed fleet: each drone's `model:` in the file (`quad_px4` or `x8_arduplane`) |
| `./run_cesium_sim.sh --qgc --spotter` | Also open the spotter window |
| `./run_cesium_sim.sh --px4-params my.params --pi-ip 192.168.199.73 --qgc` | Your PX4 parameters, plus MAVLink and video for the Raspberry Pi |

## Launcher options

These are handled by `run_cesium_sim.sh`; everything else goes to
`px4_gimbal_cesium.py`.

| Option | Default | Meaning |
|---|---|---|
| `--px4-params FILE` | | Extra PX4 parameters applied at boot, after the model's own (`px4_params.parm` for `quad_px4`). Accepts `NAME VALUE` lines (like ArduPilot `.parm` files, `#` comments allowed) or a QGC `.params` export. Repeatable; later files win |
| `--pi-ip IP` | | Also send MAVLink to a companion computer at `IP`, and announce the video at `rtsp://<this laptop's LAN IP>:8554/gimbal` |
| `--pi-ports P1,P2` | 15540,15541 | Companion-computer MAVLink UDP ports (as `--serial1/2` in the old ArduPilot scripts) |
| `--config FILE` | `config.yml` | The drones, their companions and the ground station's settings (see [The config file](#the-config-file)); `--drones`, `--pi-ip` and `--layer-step` override it. `--fleet` is the same. (see [Companions on several drones](#companions-on-several-drones)). Replaces `--drones` and `--pi-ip` |

The sim also sends the robot config's `parameters` (`NAV_RCL_ACT`,
`COM_OBL_RC_ACT`, ...) after PX4 connects (no RC in simulation). It sends them
as floats, which PX4 accepts for integer parameters only when the value is 0,
so set any other integer parameter in a params file instead.

Each drone's aircraft is its `model:` in the config file (default
`quad_px4`; `x8_arduplane` is the Skywalker X8 on ArduPlane), or
`px4_gimbal_cesium.py`'s `--model` for drones the file doesn't give one. See
[aircraft](features/aircraft.md) for the models and how to add one.

`px4_params.parm` (next to the launcher) is the `quad_px4` model's, applied
before any `--px4-params`. It:

- raises PX4's speed limits (18 m/s horizontal, 15 m/s cruise, faster climb
  and acceleration)
- sets the link-loss failsafe to return home after 10 s (`NAV_DLL_ACT 2`,
  `COM_DL_LOSS_T 10`)
- makes the simulated battery last 20 minutes armed, never below 35% on its
  own (`SIM_BAT_DRAIN`, `SIM_BAT_MIN_PCT`)
- makes a mission's `look_at` (ROI) turn the camera rather than the drone
  (`MIS_MNT_YAW_CTL 1`; see [missions](features/missions.md))

Edit it to taste. The ground station's safety panel (F2, and its Sim tab, F9)
can cut a drone's link and drain its battery to test the responses; see
[safety](features/safety.md).

## The config file

`config.yml` (next to the launcher) holds the ground station's settings, with
every key and its default written out:

| Section | What |
|---|---|
| `drones:` | One entry per drone, in order; `companion: IP` gives a drone a Radxa running X |
| `swarm:` | Height layers (`layer_step_m`), default height, line spacing, search spacing, arrival distance |
| `deconfliction:` | Separation distances, look-ahead, give-way climb and clearance |
| `safety:` | Battery, link, GPS and geofence thresholds and responses ([safety](features/safety.md)) |
| `controls:` | Gimbal step, zoom step and the companion's zoom pulse |

The command line's `--drones`, `--pi-ip` (Drone1's companion) and
`--layer-step` override it; `--config FILE` uses another file. Real drones use
the same format as their fleet file (`gcs/fleet.example.yml`). A misspelt key
is an error naming the valid ones. The start-up line says what was used:
`Config .../config.yml: 3 drones, companions Drone1 ..., height layers 5 m`.

Each time the world loads (start-up and `goto`), every drone's stored geofence
and mission are cleared: PX4 keeps both on disk across restarts.

## Script options

| Option | Default | Meaning |
|---|---|---|
| `--lat`, `--lon` | Seletar runway | Spawn position (degrees). Pick open ground: trees and roofs are solid |
| `--heading` | 33 at Seletar, else 0 | Spawn heading, degrees clockwise from north |
| `--ground-alt` | looked up online | Ground elevation at the spawn (m MSL). Pass it to skip the Open-Meteo lookup |
| `--spawn-height` | 15 | The drone starts this high above `--ground-alt` and settles onto the real surface |
| `--geoid` | 8.1 | Geoid height (m); about 8 across Singapore |
| `--drones N` | 1 | Fly a swarm of N PX4 drones (see [Swarm](#swarm)) |
| `--drone-spacing M` | 8 | Metres between the drones at the spawn, in a line to the right |
| `--layer-step M` | config (5) | Drone N flies (N-1) x M metres above the height asked for (0 = off) |
| `--objects FILE` | `config_spawned_objects.yml` | Objects to place in the world; `''` for none |
| `--actors FILE` | `config_moving_actors.yml` | People and vehicles moving along paths; `''` for none |
| `--camera-props FILE` | `camera_properties.json` | GimbalCam resolution, sensor and zoom lens |
| `--spotter` | off | Open the spotter window |
| `--qgc` | off | Manual mode instead of the scripted demo |
| `--rtsp-url URL` | `rtsp://127.0.0.1:8554/gimbal` | Where the GimbalCam video is published |
| `--rtsp-advertise-url URL` | same as `--rtsp-url` | Video address announced to QGC and the Pi (set by `--pi-ip`) |
| `--no-rtsp` | off | Don't publish video |
| `--ground-truth [DIR]` | off (`datasets/sim_gt/` in the ProjectAirSim checkout) | Save labelled GimbalCam frames from every drone (see [Labelled frames](#labelled-frames-for-detection-tracking-and-reid)) |
| `--gt-rate HZ` | 2 | Labelled frames saved per second per drone |
| `--tile-wait SEC` | 90 | Longest time to wait for the ground under the drone to load |
| `--scene FILE` | | Load a scene from `sim_config/` as-is instead of building one from `--lat/--lon` |

Run `python px4_gimbal_cesium.py --help` for the full list.

## Spawned objects

Objects are defined in `config_spawned_objects.yml`. Each has a position and
exactly one of `asset` (a model built into the sim) or `file` (your own
`.glb`/`.gltf`):

```yaml
objects:
  - name: Car1
    asset: SKM_SportsCar      # or: file: models/my_target.glb
    lat: 1.413730
    lon: 103.865200
    heading: 33               # degrees from north
    length: 4.5               # longest side in metres (or scale: [x, y, z])
    height: 0                 # metres above ground
    physics: false
```

Keep objects within a few hundred metres of the drone so the ground under them
has loaded.

While the sim runs, type these in the terminal:

| Command | Effect |
|---|---|
| `list` | Show objects spawned this session |
| `spawn ASSET LAT LON [HEADING] [LENGTH_M]` | Spawn a built-in asset on the ground |
| `delete NAME` / `delete all` | Remove objects |
| `reload` | Delete all and respawn from the YAML file (edit it first) |
| `assets [REGEX]` | List built-in assets |
| `goto LAT LON [HEADING]` | Move the drone to a new place without restarting Unreal (`--qgc` mode) |
| `gimbal PITCH YAW` | Point the gimbal (degrees; yaw relative to the drone) |
| `zoom PERCENT` | Set the zoom (0 = widest, 100 = longest focal length) |

## Moving people and vehicles

`config_moving_actors.yml` defines actors that walk or drive along paths. The
sim moves them every frame, keeps them on the ground, plays walk animations and
pauses them with the sim, so dozens can move at once. Each path is one of:

```yaml
actors:
  - name: Person1
    kind: person                    # person or vehicle: picks the defaults
    waypoints: [[1.413416, 103.865262], [1.413642, 103.865409]]
    mode: loop                      # loop, pingpong or once
  - name: Courier
    kind: vehicle
    route: {from: [1.41660, 103.86180], to: [1.406242, 103.859972]}
  - name: Traffic                   # Traffic_1 .. Traffic_4
    kind: vehicle
    count: 4
    speed: 12
    random_route: {length_m: 1500}
```

`route` and `random_route` follow OpenStreetMap roads or footpaths (people).
Vehicles obey one-way streets and drive in the kerb-side lane (left in
Singapore), placed using OSM's lane counts (3.5 m lanes). To go back, a
vehicle takes its own legal route from the end to the start (often the other
carriageway) instead of reversing over its path, and laps that round trip. A `route`'s `from` and `to` must be within 50 m of a road
or footpath. Vehicle `waypoints` are joined along the roads too (set
`follow_roads: false` for straight lines; people walk straight unless
`follow_roads: true`); a waypoint off the roads is reached in a straight line.
For vehicles, airport runways and taxiways count as roads for `waypoints` and
`route`, but random traffic stays off them. `start_delay` holds an actor still
for a while first; `start_at_m` / `stop_short_m` start it part-way along its
path and stop it short of the end, so several cars sharing one route wait and
stop in formation (a convoy) instead of stacked at its start. A
[mission](features/missions.md) can bring its own actors file (`actors:`),
respawned at each `mission start`. The map data is downloaded once into `osm_cache/` and needs
internet the first time. Preview routes without the sim:
`python road_routes.py --kind vehicle --random 1500 --count 5 --geojson routes.geojson`,
then drop the file onto https://geojson.io.

**Map editor.** Instead of typing coordinates, pick paths on a map in the
browser:

```bash
python actor_editor.py                    # edit config_moving_actors.yml
./run_cesium_sim.sh --qgc --edit-actors   # edit first; the sim starts on "Save & start"
python actor_editor.py --mission NAME     # a mission's drone steps and its own traffic, together
./run_cesium_sim.sh --qgc --edit-mission NAME
```

With `--mission`, the page has a second tab to plan the drones' flight on the
same map; see [Planning on the map](features/missions.md#planning-on-the-map).
The Traffic tab then edits the file the mission names (`actors:`).

Choose or add an actor on the left, pick a path type, then click the map:

- **Route:** click A, then B. The path follows roads (vehicles) or footpaths
  (people). Drag the pins to adjust.
- **Waypoints:** click points in order. Drag to move, right-click to remove.
  With "Follow roads between waypoints" (on by default for vehicles) the path
  follows the roads between points; untick it for straight lines. "Return to
  the first point" loops back to the start, like a mission that returns to its
  takeoff point.
- **Random route:** optionally click a start point; set the length, number of
  copies and seed.
- **Start along the path / Stop short of the end** (`start_at_m`, `stop_short_m`):
  use part of the path, so cars sharing one route wait and stop in formation.

The line drawn is exactly the path the sim will use. Save writes the YAML and
keeps the previous file as `.bak`. Comments inside the actor list aren't kept;
the explanation at the top of the file is. Pass `--lat/--lon` if the drone
spawns somewhere other than Seletar. In a running sim, type `actors reload`
after saving.

| Command | Effect |
|---|---|
| `actors [list]` | Show moving actors |
| `actors pause` / `actors resume` | Stop or restart all of them |
| `actors reload` | Delete and respawn from the YAML file |
| `actors delete NAME` | Remove one |

**Trees, bridges and occlusion.** The photorealistic world is one 3D surface:
where trees or a bridge overhang a road, their top is all there is. A hidden
bare-earth terrain (Cesium World Terrain, ion asset 1, needs no extra setup) is
loaded alongside it. Where the visible surface is more than 4 m above the road,
an actor goes *under* it at road level, so tree cover hides it from the drone
as it would in reality. The road height under cover is ramped from where the
cover starts to where the path comes out of it. Bridges (from OSM) keep
vehicles on the deck. Placed objects whose underside is high up, like a roof,
count as cover too; objects standing on the ground, like parked cars, don't.
The tiles cast Unreal shadows, so actors under trees or beside buildings are
shaded.

**Ground truth.** Where every actor is gets logged about 5 times a second to
`/tmp/projectairsim-run/ground_truth_<time>.csv`, as ground truth for detection,
occlusion and re-identification. The newest 5 logs are kept; geojson.io can
show the CSV on a map. Columns:

| Column | Meaning |
|---|---|
| `sim_time_s`, `name`, `kind` | When, which actor, person or vehicle |
| `lat`, `lon`, `alt_m`, `north_m`, `east_m`, `down_m`, `heading_deg` | Where, and its direction of travel |
| `in_view` | 1 if its centre is in Drone1's GimbalCam field of view (within 1.5 km) |
| `visible_fraction` | Share of 18 points over its body that are in view with a clear line of sight (0-1); trees, buildings and other actors block it |
| `covered` | 1 while it is under trees, a bridge or a roof |
| `camera_distance_m` | Distance from the GimbalCam |

The same check is available to scripts as
`world.get_object_visibility(robot, camera, names)`.

### Labelled frames for detection, tracking and ReID

`--ground-truth [DIR]` saves every drone's GimbalCam frames with a box around
each person and vehicle in view: YOLO labels for training a detector, and each
box tagged with the actor's name, which is the same across frames and across
drones, for scoring tracking and re-identification.

```bash
./run_cesium_sim.sh --qgc --drones 2 --ground-truth            # ProjectAirSim/datasets/sim_gt/<time>/
./run_cesium_sim.sh --qgc --ground-truth ~/data/gt --gt-rate 5
```

The boxes come from a segmentation image the GimbalCam renders beside its
picture, where each actor has an ID of its own. So a box fits what is actually
drawn: a car half behind a building gets a box around the visible half, a car
inside a building or fully under a roof gets none, and one cut by the frame edge
gets a box up to the edge. The segmentation zooms with the camera.

Labelled: the moving actors, and placed objects whose asset or model file name
says they are a vehicle or a person (`car`, `vehicle`, `truck`, `van`, `bus`;
`manny`, `quinn`, `mannequin`, `person`, `human`). Objects spawned or deleted
while it runs are picked up within 2 s. Type `gt` to see how much has been saved.

Output, in `DIR/<YYYYmmdd_HHMMSS>/`:

| File | Contents |
|---|---|
| `images/<drone>_s<scene>_<sim ns>.jpg` | The frame. `<scene>` counts `goto` reloads |
| `labels/<same name>.txt` | YOLO boxes, `class cx cy w h` normalised 0-1; class 0 person, 1 car |
| `classes.txt` | The class names, in order |
| `boxes.csv` | Every box: `sim_time`, `drone`, `frame`, `actor` (identity), `class`, `x1 y1 x2 y2` (pixels), visible `pixels`, `truncated` (cut by the frame edge) |
| `frames.csv` | Every frame: `sim_time`, `drone`, `frame`, size, box count, the camera's world position and orientation (NED, quaternion) and horizontal FOV |
| `ground_truth_<time>.csv` | Where every moving actor was (the log described above), one per scene load. With `--ground-truth` it goes here instead of `/tmp/projectairsim-run/`, and none are deleted |
| `mission_runs.csv` | When each ground-station mission run started and its track steps (sim time), to pick a run's frames out of the capture |

Frames with nobody in view are skipped, except one in ten (hard negatives).
For every frame of a mission run, with the drone's metadata and a video
instead of JPEGs, record the mission instead (`record: true`; see
[MOT experiments](features/mot-experiments.md)).
Images and labels share a name, the layout Ultralytics expects; list several
runs' `images/` folders in a `data.yaml`, and keep whole runs (flights or
places) for validation rather than random frames, which are near-copies of
their neighbours.

**Reviewing a capture** after the sim has stopped:

```bash
python gt_playback.py                                   # the newest capture
python gt_playback.py ../datasets/sim_gt/<time> --drone 2 --only-boxes
python gt_playback.py --only-boxes --save review.mp4    # to a video instead
```

Each frame is shown with its boxes and actor names (one colour per actor),
beside a north-up map of the actors' paths, where each actor was (filled when
boxed in this frame) and each drone camera's position and heading. Keys: space
play/pause, arrow keys (or a/d) step, `.` `,` jump 10, `1`-`9` one drone, `0`
all, `b` frames with boxes only, `+` `-` speed, `q` quit. Parked objects are
boxed but not on the map, which only has the moving actors.

It costs a second render per GimbalCam, so leave it off when not collecting.
Without it the scene doesn't initialise segmentation IDs at all, which also
saves the Cesium tiles' custom-depth pass.

## Swarm

```bash
./run_cesium_sim.sh --qgc --drones 3
```

starts Drone1 at the spawn point and Drone2, Drone3 in a line to its right
(`--drone-spacing`), each with its own PX4 instance, gimbal camera, MAVLink
camera and RTSP stream (`/gimbal`, `/gimbal2`, `/gimbal3`). Without `--qgc`,
the swarm flies a hands-off demo: takeoff, a search of a box ahead of the
spawn, return and land.

**From QGroundControl:** one manual UDP link (listen 14550) with a server per
drone: `127.0.0.1:18570`, `127.0.0.1:18571`, `127.0.0.1:18572`. QGC shows
Vehicle 1-3; pick one in the toolbar to fly it or upload its mission, or use
the Fly view's multi-vehicle list to command them all.

**From the terminal** (drones first: a number, a list like `1,3`, or `all`):

| Command | Effect |
|---|---|
| `all takeoff [ALT]` | Arm and climb to ALT m above home (default 30) |
| `2 goto LAT LON [ALT] [HEADING]` | Fly drone 2 there and hover; several drones form a line 15 m apart. With HEADING (degrees from north: 0 N, 90 E) it flies there with its heading unchanged and turns to face HEADING once it arrives (within 4 m); without, it keeps its heading. Use `-` for ALT to keep the height, e.g. `2 goto 1.4150 103.8660 - 180` |
| `2 heading DEG` | Turn drone 2 in place to face DEG from north |
| `1,3 hold` / `land` / `rtl` | Hover / land where they are / return home and land |
| `1,3 searcharea ALT SPACING LAT,LON;LAT,LON;...` | Grid-search a polygon: passes SPACING m apart are split between the drones and each gets a PX4 mission (QGC shows it) |
| `all search LAT1 LON1 LAT2 LON2 [ALT] [SPACING]` | The same for a box (default spacing 40) |
| `status` | Each drone's flight mode, armed state, height and position |
| `select N` | Show and steer drone N in the dashboard (or press 1-9 there) |
| `gimbal PITCH YAW [N]`, `zoom PERCENT [N]` | Point / zoom drone N's camera (default: the selected one) |

The commands go to each drone's PX4 over MAVLink, like a companion computer's,
so QGC shows what the drones are doing and can take over at any time. A new
command for a drone replaces what it was doing.

**From the dashboard:** the control bar under the dashboard does the same
without typing (see [Camera and gimbal](#camera-and-gimbal)): pick a drone,
click the minimap to set its target, and use *Go to target*, *Set heading*,
*Take off*, *Hold*, *Land* or *Return home*. Tick *Buttons command all
drones* to command the whole swarm.

**Area search from the dashboard:** turn on *Draw area*, click the corners
on the minimap, set the height, pass spacing and drones (`all` or e.g. `1,3`)
and press *Search area*. The passes run along the area's longest edge and are
split so each drone's share is about as long; concave areas work. Each drone's
remaining route is drawn on the minimap.

**Deconfliction.** Drones don't avoid each other on their own (PX4 knows
nothing about the other drones), so the swarm does it in three layers:

1. *Height layers:* drone N flies (N-1) x 5 m above the height asked for
   (`--layer-step`), so routes that cross pass at different heights.
2. *Crossing check:* every new route is compared with the other drones'
   routes. Places where they come within 6 m are reported in the messages and
   marked on the minimap: green if the heights keep them 3 m or more apart,
   red if not.
3. *Separation monitor:* from each drone's position and velocity, the swarm
   predicts how close every pair will get in the next 6 s. If two would pass
   within 6 m horizontally and 3 m vertically, the higher-numbered drone gives
   way: it stops and climbs 6 m, then resumes its mission or flight once the
   other is 10 m away. This also covers drones flown from QGC. PX4 reports
   positions about 4-5 times a second, which is why it predicts rather than
   waiting until they are close.

The dashboard, swarm control and deconfliction are the ground station in
`gcs/`, which flies real PX4 drones with the same code; see
[gcs/README.md](../gcs/README.md).

The dashboard shows the selected drone's cameras; the minimap shows all drones
(the selected one yellow, the others blue), where each is heading and its
remaining route, and a compass with the selected drone's heading. The status
panel lists each drone's mode and height. Every camera of the swarm renders, so frame rates drop
(about 12 fps per camera with 3 drones on an RTX 3070 laptop, against 22 with
one); drones 2 and up have a smaller chase camera and no down camera.

## Changing location

Start somewhere else with `--lat/--lon/--heading`, or move while running by
typing `goto LAT LON [HEADING]` in the terminal (`--qgc` mode). `goto` keeps
Unreal running, reloads the scene at the new spot, restarts PX4 for it (PX4 SITL
can only connect to one scene), waits for the ground there and respawns the
objects and spotter. It takes about 25 s once tiles are cached. Reconnect
QGroundControl afterwards.

Objects and spotter points are absolute lat/lon, so edit
`config_spawned_objects.yml` and `config_spotter_points.yml` for the new area.

## Camera properties and optical zoom

`camera_properties.json` uses the same format as the old ArduPilot package,
plus a `zoom` block:

```jsonc
"resolution": "1080p",                       // 360p, 480p, 720p, 1080p, 2k, 4k or WxH
"sensor": { "sensor_width_mm": 6.19, "sensor_height_mm": 3.48 },
"lens":   { "focal_length_mm": 5.2, "focus_distance_m": 50.0 },
"zoom":   { "focal_length_max_mm": 52.0,     // or "optical_zoom_max": 10
            "zoom_speed_percent_per_s": 30.0 }
```

The field of view is `2·atan(sensor_width / 2·focal_length)`, so zooming from
5.2 to 52 mm narrows a ZR10 from 61.5° to 6.8°, like the real 10x lens. Also
applied: `focus_distance_m` (depth of field), `ue_overrides.chromatic_aberration`
and the `ue_noise_*` image noise.

### Lens distortion, vibration and sensor noise

Three more defects are added to the frames in Python (`camera_effects.py`), so
the video looks like a real payload's. Each is off until enabled:

```jsonc
"lens":     { "distortion_enabled": true, "distortion_k1": -0.12, "distortion_k2": 0.02,
              "distortion_p1": 0.0, "distortion_p2": 0.0, "distortion_k3": 0.0 },
"platform": { "vibration_enabled": true, "vibration_amplitude_deg": 0.02,
              "vibration_frequency_hz": 30.0 },
"sensor":   { "noise_enabled": true, "noise_gain_db": 6.0, "noise_temperature_c": 25.0 }
```

- **Distortion** uses OpenCV's model, so coefficients from a real calibration go
  straight in (negative `k1` is barrel). The same coefficients apply at every zoom.
  Corners the distortion leaves empty are black.
- **Vibration** shakes the picture by `vibration_amplitude_deg` at
  `vibration_frequency_hz` sideways, and at a slightly different rate up and
  down. It is worked out in sim time, so a replay shakes the same way. The angle
  is converted to pixels with the current field of view, so zooming in shakes the
  picture more.
- **Noise** is signal-independent: a standard deviation of
  `10^(gain/20) · 2^((T-25)/8)` levels (8-bit), which is 2 at 6 dB and 25 °C. It
  is an indicative model, not a calibrated one.

### Changing the camera while the sim runs

`camera_properties.json` is what the camera starts with. In the terminal, for
every drone's GimbalCam at once:

| Command | What it does |
|---|---|
| `sim camera res 720p` (`1080p`, `4k`, `WxH`) | The render size of the scene and segmentation images; the RTSP stream restarts at the new size (players reconnect after a second or so). Recordings in progress stop writing frames, so start a new run. A later `goto` keeps the new size |
| `sim camera noise on` / `off` / `GAIN_DB` | The sensor noise above, on or off, or on at a gain |
| `sim camera aperture F` | The f-number, which changes the depth of field only (exposure stays automatic) |

Expect one odd frame at the switch (still the old size, or black). That makes a resolution or noise sweep one flight: change it, fly the mission
again, and score each run with `mot_eval.py`.

From a script, the client has `drone.set_resolution(camera, w, h)` and
`drone.set_aperture(camera, f)` (the service methods `SetResolution` and
`SetAperture` in `core_sim`). The engine's own image noise (`ue_noise_*`) can only
be set when the scene loads: turning it on later left about a third of the frames
black, so it isn't offered.

The defects are on the RTSP stream and the dashboard, and in the `--ground-truth` and
mission-run recordings. The segmentation those boxes come from is moved the same
way, so the boxes stay on the objects. Noise doesn't touch it. All three cost
about 5 ms per 1080p frame.

## Camera and gimbal

**Dashboard window** ("ProjectAirSim"): one window, tiled like tmux panes,
with a control bar underneath:

```
+----------------------------+-------------+
|                            | small camera|
|  main camera (GimbalCam    +-------------+
|  or ChaseCam)              |             |
+----------------------------+   Minimap   |
|  Status (drones, counts)   |             |
+----------------------------+-------------+
| Drone: [Drone1] [Drone2] ...   View: [GimbalCam] [ChaseCam]  [ ] all drones |
| Height [30]  Heading [  ]  Take off  Go to target  Set heading  Hold ...    |
| Search area: Draw area  Undo corner  Clear area  Spacing [40]  Drones [all] |
| Command [ ...the terminal's commands... ]                                   |
| messages                                                                    |
```

**Control bar:**

- *Drone* buttons (or keys 1-9) pick the drone whose cameras are shown and
  whose gimbal the keys steer. *View* puts the GimbalCam or the ChaseCam in the
  big pane (or press V, or click the small camera).
- Flight buttons command the selected drone, or every drone with *Buttons
  command all drones* ticked. *Height* is metres above home; *Heading* is
  degrees from north (0 N, 90 E, 180 S, 270 W), used by *Set heading* (turn in
  place now) and *Go to target* (turn on arrival; blank keeps the heading).
- Click the minimap to set the selected drone's target (red cross). Each drone
  keeps its own target: switch drones to see theirs. Once a drone is flying
  somewhere, a line runs from it to its destination.
- *Search area*: see [Swarm](#swarm).
- *Command*: any terminal command (↑/↓ for history); replies and drone
  messages appear below it. Debug messages go to `projectairsim_client.log`
  only (add `--verbose` to see them in the terminal too).

The **minimap** is north-up and centred on the selected drone (yellow arrow;
other drones blue). It shows OpenStreetMap roads, moving vehicles and people
(dots with a heading tick), static objects (grey squares), targets, routes,
predicted path crossings and the search area being drawn. The **compass** in
its bottom-right corner shows the selected drone's heading against North in
degrees, with a red tick at the heading it was told to face. Click the compass
to pick a heading for *Set heading* / *Go to target* (it shows as a hollow
pointer until sent). Positions refresh
every second. It starts at
2 m per pixel (about 1 km across) and zooms from 0.25 to 16 m per pixel. Roads
cover 1.5 km around the spawn point; objects show wherever they are.

Each pane zooms on its own: the mouse wheel acts on the pane under the pointer.

Click the cameras or map first so they have keyboard focus (typing in the
control bar's boxes doesn't steer the gimbal):

| Input | Action |
|---|---|
| W / S | Tilt the gimbal up / down |
| A / D | Pan left / right |
| Mouse wheel over the GimbalCam, + / - | Optical zoom along the lens's focal-length range |
| Mouse wheel over the ChaseCam | Digital zoom (1–8x) |
| V, or click the small camera | Swap the two cameras |
| 1-9 | Show and steer that drone |
| Click the minimap | Set the selected drone's target (or, in *Draw area* mode, add a corner) |
| C | Centre (level, straight ahead) |
| X | Look straight down |
| Mouse wheel over the minimap, [ / ] | Zoom the map out / in |
| L | Map labels on / off |

The spotter's control and POV windows stay separate.

**Video stream:** the GimbalCam is published once and served two ways:

- **WebRTC** (lowest latency, always the newest frame): open
  `http://127.0.0.1:8889/gimbal` in a browser.
- **RTSP**: `rtsp://127.0.0.1:8554/gimbal` (H.264 at the camera's resolution),
  for QGC, the Pi or `cv2.VideoCapture("rtsp://127.0.0.1:8554/gimbal")`. In
  QGC, turn on *Low Latency Mode* under Application Settings → Video.

**QGroundControl:** add a manual link under Application Settings → Comm Links:
type UDP, port 14550, server `127.0.0.1:18570`, then Connect. The script
appears to QGC as a MAVLink camera, so QGC picks up the video and its zoom and
shutter controls work. Snapshots are saved to `~/Pictures/projectairsim/`.

QGC and the dashboard both steer the gimbal through PX4. Whichever was
used last is in control.

### Gimbal stabilisation and yaw modes

The GimbalCam's mount keeps the camera level: whatever the drone's roll and
pitch, the camera holds the pitch it was given (measured: drone tilting up to
38 deg in hard acceleration and braking, camera roll 0.0 and pitch exactly as
commanded throughout).

How the gimbal moves is set by the actuator's `gimbal-settings` in
`sim_config/robot_quadrotor_px4_gimbal_payload.jsonc` (leave the block out for
an ideal gimbal that reaches every angle at once):

| Setting | Default here | Meaning |
|---|---|---|
| `max-pitch-rate-deg`, `max-yaw-rate-deg`, `max-roll-rate-deg` | 120 | Top slew rate (deg/s) |
| `pitch-min-deg` / `pitch-max-deg` | -90 / 25 | Pitch travel |
| `yaw-min-deg` / `yaw-max-deg` | -160 / 160 | Yaw travel, from the nose |
| `settling-time-s` | 0.1 | Lag reaching a commanded angle |
| `yaw-follow-time-s` | 0.5 | Follow mode: how far the camera trails the drone's turns |

These are typical of a ZR10-class gimbal; set them from your own gimbal's
datasheet. The C++ is `core_sim/src/actuators/gimbal.cpp` (rebuild the sim libs
and the Blocks plugin after changing it; the settings themselves only need a
restart).

**Yaw follow or yaw lock.** In *follow* (the default) the camera turns with the
drone, trailing its turns by `yaw-follow-time-s`. In *lock* it holds its
compass heading while the drone turns, until it reaches its yaw travel. For a
drone without a companion, press **Y** (or `gimbal lock on|off [N]`); the
GimbalCam title shows `yaw follow` / `yaw lock` and the angle the gimbal
actually reports. With a companion, X decides: `MAVLINK_YAW_LOCK` in its
`config_gimbal.yml`.

**Measuring it.**

- `sim attitude [SECONDS] [N]` logs drone N's attitude and its camera's actual
  (rendered) attitude to `attitude_<drone>_<time>.csv` in the log folder.
- `sim tracktest [N] [follow|lock]` (a drone without a companion, airborne)
  points the gimbal at a ground point 50 m ahead 20 times a second (an
  idealised tracker, so it measures the gimbal, not a vision tracker) while the
  drone dashes 120 m sideways, stops hard, turns 90 deg and dashes back. It
  reports the target's largest offset from the image centre in each phase and
  the share of frames it was in frame, with a CSV of every frame.

Results with the settings above:

| Yaw mode | In frame | Largest offset (deg): dash / stop / turn / dash back | Mean |
|---|---|---|---|
| follow | 100% | 5.0 / 0.7 / 21.2 / 2.8 | 2.2 |
| lock | 100% | 2.3 / 0.2 / 2.0 / 1.9 | 0.7 |

Pitch and roll stabilisation keep the target steady through the dashes and the
hard stop in both modes; in a turn, follow mode swings the target 21 deg off
centre (half the frame's width is 31 deg) while yaw lock holds it within 2 deg.

## Spotter

`--spotter` (or `python spotter_control.py` while a scene is running) opens a
window to place a walking human in the world:

- Pick a point from `config_spotter_points.yml` and press **Spawn here**,
  **Teleport** or **Walk to**. The spotter follows the ground and plays walk
  and idle animations.
- Click the window, then hold **W/S** to walk forward/back and **A/D** to turn.
- **Speed** sets walking speed; **Save position as point** appends the current
  spot to the YAML file.
- The status line shows its position and distance to the drone.

The **Spotter POV** window is the spotter's own eyes. Click it to capture the
mouse and look around (Esc releases it), W/S walk forward/back, A/D step
sideways, hold Shift to run. The view comes from an invisible camera robot
(`PovCam`) kept at eye height.

## Raspberry Pi (companion computer)

With `--pi-ip`, PX4 sends MAVLink to the Pi on UDP 15540 and 15541 (like the
old ArduPilot `--serial1/2 udpclient:` links). The Pi can:

- read video from `rtsp://<laptop LAN IP>:8554/gimbal` (printed at startup)
- steer the gimbal through PX4's gimbal manager (`MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE`,
  then `GIMBAL_MANAGER_SET_ATTITUDE` to PX4 1/1)
- zoom with `MAV_CMD_SET_CAMERA_ZOOM` to the camera component (1/100)

Allow UDP 15540-15541 and TCP 8554 through any firewall between the two.

### X vision pipeline on a Radxa (A5X_RPI_Release, `rknn-sim-*` profiles)

X's `rknn-sim-<airframe>-<autopilot>` profiles (branch `swarm-sim`) run X
against this sim instead of a real camera and gimbal: they read
`rtsp://<laptop>:8554/gimbal`, steer the sim gimbal through the autopilot's
gimbal manager and zoom the sim camera. Run the one matching the drone's
`model:` (its [aircraft](features/aircraft.md)); the sim logs which, per drone:

| Drone `model:` | X profile | Radxa folder |
|---|---|---|
| `quad_px4` | `rknn-sim-quad-px4` | `~/Code/X_sim_quad_px4` |
| `x8_arduplane` | `rknn-sim-fw-ardupilot` | `~/Code/X_sim_fw_ardupilot` |

(`rknn-sim-quad-ardupilot` and `rknn-sim-fw-px4` exist for aircraft the swarm
doesn't have yet, in `~/Code/X_sim_quad_ardupilot` and `~/Code/X_sim_fw_px4`.)

```bash
# laptop, in A5X_RPI_Release/X: one folder per profile (your flight copy is untouched)
./deploy.sh --profile rknn-sim-fw-ardupilot --dir radxa@192.168.199.99:~/Code/X_sim_fw_ardupilot/ --no-setup
ssh radxa@192.168.199.99 'ln -sfn ../X_quad/venv ~/Code/X_sim_fw_ardupilot/venv'   # once: reuse X_quad's venv

# laptop: the sim, with the Radxa as Drone1's companion
./run_cesium_sim.sh --qgc --model x8_arduplane --pi-ip 192.168.199.99

# Radxa
cd ~/Code/X_sim_fw_ardupilot && venv/bin/python main.py
```

The profiles' `INPUT_RTSP` names the laptop's address (192.168.199.98); edit
it in the profile's `config_cv.yml` if that changes.

The dashboard's main pane then always shows **Tracker**, X's annotated video
(`rtsp://<radxa>:8554/tracker`), with GimbalCam (or ChaseCam, picked with View
or V) in the small pane beside it. It talks to X like its ground station
(`map-plot/laptop_client.py`), over UDP 5556:

| Input (Tracker pane) | Sends |
|---|---|
| Click a target / drag a box around it | `CLICK:x,y` / `BOX:size` then `CLICK` at the box centre (tracker-video pixels) |
| Right-click, or Esc | `CANCEL` |
| Arrow keys | Nudge the tracked point by 8 px |
| W / S, A / D (hold) | `PITCH:UP/DOWN`, `YAW:LEFT/RIGHT`, then `…:STOP` on release (cancels a track first) |
| Mouse wheel over the camera, + / - | `ZOOM:IN/OUT`, then `ZOOM:STOP` |
| G or C | `GIMBAL:CENTER` (cancels a track first). X (look down) is off with a companion |
| **STRIKE** button (top right), or `strike on\|off [N]` | `STRIKE:START` / `STRIKE:STOP`, at once. Refused while a mission holds the camera; a mission taking the camera, or the ground station closing, releases it |

With a companion, the gimbal and zoom keys always go to X (it owns the gimbal),
whichever view is shown. X ignores manual gimbal and zoom commands while it is
tracking, so W/A/S/D and G cancel the track before steering; zoom does nothing
until the track ends. A heartbeat goes out every 0.2 s, and the status panel shows whether
the tracker video is arriving. `--companion IP` sets the companion without
`--pi-ip`. Drone control is off in the `rknn-sim-*` profiles for now, so X
ignores the STRIKE button there; fly the drone from the dashboard or QGC.

While a ground-station [mission](features/missions.md) holds the camera, X
stands down (`MAVLINK_STAND_DOWN_IN_MISSION`): it keeps detecting and
tracking, but sends no gimbal commands until the mission hands the camera back
(a `track` step) or ends. On ArduPilot, whose missions can't claim or release
the gimbal themselves, the ground station makes those handovers by command
(so they need its link to the drone). The profiles detect with the VisDrone model at
confidence 0.2 with camera motion compensation, chosen from the
[tracker experiments](tracker-cmc-experiments.md).

### Companions on several drones

Give each drone its own companion computer in the config file (`config.yml`,
or another with `--config`):

```yaml
drones:
  - companion: 192.168.199.99     # Drone1
  - companion: 192.168.199.100    # Drone2
  - {}                            # Drone3: no companion
```

```bash
./run_cesium_sim.sh --qgc --config my_fleet.yml
```

Drone N's PX4 sends MAVLink to its companion on UDP 15540 and 15541, and its
video is `rtsp://<laptop>:8554/gimbalN` (`/gimbal` for Drone1), so each Radxa's
`INPUT_RTSP` must name its own drone's stream. The dashboard shows the selected
drone's Tracker (keys 1-9 pick the drone). The sim reads only how many drones
there are and their companions; the rest of the file is for real drones.

## Ports

| Port | Used by |
|---|---|
| UDP 14550 ← 18570 (+N) | PX4 ↔ QGroundControl (drone N+1 of a swarm) |
| UDP 14030 ← 14280 (+N) | PX4 ↔ this script (gimbal, MAVLink camera, swarm commands) |
| TCP 4560 (+N) | PX4 ↔ simulator |
| UDP 15540, 15541 ← 14590, 14591 (+10N) | PX4 ↔ companion computer (with `--pi-ip`, or `companion:` in the config) |
| UDP 5556, TCP 8554 on the companion | Dashboard → X commands, X's tracker video |
| TCP 8554 | RTSP video |
| TCP 8989 / 8990 | Simulator API |

## Logs and disk space

- Sim, PX4 and RTSP server logs: `/tmp/projectairsim-run/`, overwritten each run
- PX4 flight logs: `~/Documents/PX4-Autopilot/build/px4_sitl_default/rootfs/log/`,
  20–60 MB per PX4 run. Only the newest 5 are kept: older ones are deleted each
  time PX4 starts (launch and `goto`). Set `PX4_KEEP_LOGS` to change the count;
  copy out any log you want to keep for good
- Cesium tile cache: `unreal/Blocks/cesium-request-cache.sqlite`, capped at 4096 tiles

## Troubleshooting

- **"Drone fell through the Cesium ground":** the ground didn't load in time.
  Run again (tiles are cached now) or raise `--tile-wait`.
- **Spotter "no skeletal object … or no animation":** run
  `./install_spotter_assets.sh` and restart the simulator.
- **Camera windows, video or the spotter POV freeze:** the sim sends camera
  frames to a single client. Don't run other Project AirSim scripts with
  `client.connect()` alongside this one; use `client.connect_services()` for
  extra scripts (requests only).
- **"Port … is in use":** a previous simulator or PX4 is still running. Close it,
  or run `pkill -x px4; pkill -x UnrealEditor`.
- **PX4 "Arming denied" at startup:** normal for a few seconds while its estimator
  settles; the demo retries.
