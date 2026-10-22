# PX4 gimbal drone in a Cesium world

Flies a PX4 quadrotor with a gimbaled payload camera (GimbalCam) over real-world
Google 3D tiles streamed through Cesium for Unreal. The default spawn is Seletar
Airport's runway, Singapore.

## Prerequisites

- Unreal Engine 5.7 at `~/UnrealEngine-5.7`, with the Blocks editor built
  (Cesium for Unreal installed in `unreal/Blocks/Plugins/CesiumForUnreal`)
- PX4-Autopilot SITL built at `~/Documents/PX4-Autopilot`
- Conda env `projectairsim` (Python client, pymavlink, PyYAML)
- For the spotter: UE's template Mannequin, copied in once with
  `./install_spotter_assets.sh` (then restart the simulator)
- A Cesium ion token in `~/.cesium_ion_token`:
  ```bash
  echo 'export CESIUM_ION_TOKEN=<token>' > ~/.cesium_ion_token && chmod 600 ~/.cesium_ion_token
  ```

## Running

```bash
cd ~/Documents/ProjectAirSim/client/python/example_user_scripts
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
| `./run_cesium_sim.sh --qgc --spotter` | Also open the spotter window |
| `./run_cesium_sim.sh --px4-params my.params --pi-ip 192.168.199.73 --qgc` | Your PX4 parameters, plus MAVLink and video for the Raspberry Pi |

## Launcher options

These are handled by `run_cesium_sim.sh`; everything else goes to
`px4_gimbal_cesium.py`.

| Option | Default | Meaning |
|---|---|---|
| `--vehicle quad\|vtol` | quad | Airframe. VTOL is not available with PX4 yet |
| `--px4-params FILE` | | Extra PX4 parameters applied at boot, after `px4_params.parm`. Accepts `NAME VALUE` lines (like ArduPilot `.parm` files, `#` comments allowed) or a QGC `.params` export. Repeatable; later files win |
| `--pi-ip IP` | | Also send MAVLink to a companion computer at `IP`, and announce the video at `rtsp://<this laptop's LAN IP>:8554/gimbal` |
| `--pi-ports P1,P2` | 15540,15541 | Companion-computer MAVLink UDP ports (as `--serial1/2` in the old ArduPilot scripts) |
| `--fleet FILE` | | The drones and each one's companion computer, from the same fleet file the real-drone ground station uses (see [Companions on several drones](#companions-on-several-drones)). Replaces `--drones` and `--pi-ip` |

The sim also sets `NAV_RCL_ACT`, `NAV_DLL_ACT` and `COM_OBL_RC_ACT` after PX4
connects (no RC in simulation), so those three override your file.

`px4_params.parm` (next to the launcher) is always applied first. It raises
PX4's speed limits (18 m/s horizontal, 15 m/s cruise, faster climb and
acceleration); edit it to taste.

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
| `--layer-step M` | 5 | Drone N flies (N-1) x M metres above the height asked for (0 = off) |
| `--objects FILE` | `config_spawned_objects.yml` | Objects to place in the world; `''` for none |
| `--actors FILE` | `config_moving_actors.yml` | People and vehicles moving along paths; `''` for none |
| `--camera-props FILE` | `camera_properties.json` | GimbalCam resolution, sensor and zoom lens |
| `--spotter` | off | Open the spotter window |
| `--qgc` | off | Manual mode instead of the scripted demo |
| `--rtsp-url URL` | `rtsp://127.0.0.1:8554/gimbal` | Where the GimbalCam video is published |
| `--rtsp-advertise-url URL` | same as `--rtsp-url` | Video address announced to QGC and the Pi (set by `--pi-ip`) |
| `--no-rtsp` | off | Don't publish video |
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
`route`, but random traffic stays off them. The map data is downloaded once into `osm_cache/` and needs
internet the first time. Preview routes without the sim:
`python road_routes.py --kind vehicle --random 1500 --count 5 --geojson routes.geojson`,
then drop the file onto https://geojson.io.

**Map editor.** Instead of typing coordinates, pick paths on a map in the
browser:

```bash
python actor_editor.py                    # edit config_moving_actors.yml
./run_cesium_sim.sh --qgc --edit-actors   # edit first; the sim starts on "Save & start"
```

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
[gcs/README.md](gcs/README.md).

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
and the `ue_noise_*` image noise. Lens distortion, aperture and platform
vibration aren't simulated yet; the script warns if they're enabled.

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
degrees, with a red tick at the heading it was told to face. Positions refresh
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

### X vision pipeline on a Radxa (A5X_RPI_Release, `rknn-sim` profile)

The `rknn-sim` profile (branch `swarm-sim`) runs X against this sim instead
of a real camera and gimbal: it reads `rtsp://<laptop>:8554/gimbal`, steers
the sim gimbal through PX4's gimbal manager and zooms the sim camera.

```bash
# laptop, in A5X_RPI_Release/X: deploy to its own folder (your flight copy is untouched)
./deploy.sh --profile rknn-sim --dir radxa@192.168.199.99:~/Code/X_sim/ --no-setup
ssh radxa@192.168.199.99 'ln -sfn ../X_quad/venv ~/Code/X_sim/venv'   # once: reuse X_quad's venv

# laptop: the sim, with the Radxa as Drone1's companion
./run_cesium_sim.sh --qgc --pi-ip 192.168.199.99

# Radxa
cd ~/Code/X_sim && venv/bin/python main.py
```

The profile's `INPUT_RTSP` names the laptop's address (192.168.199.98); edit
it in `utils/config/rknn-sim/config_cv.yml` if that changes.

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

With a companion, the gimbal and zoom keys always go to X (it owns the gimbal),
whichever view is shown. X ignores manual gimbal and zoom commands while it is
tracking, so W/A/S/D and G cancel the track before steering; zoom does nothing
until the track ends. A heartbeat goes out every 0.2 s, and the status panel shows whether
the tracker video is arriving. `--companion IP` sets the companion without
`--pi-ip`. Drone control (strike) is off in `rknn-sim` for now; fly the drone
from the dashboard or QGC.

### Companions on several drones

Give each drone its own companion computer with a fleet file
(`gcs/fleet.example.yml`):

```yaml
drones:
  - companion: 192.168.199.99     # Drone1
  - companion: 192.168.199.100    # Drone2
  - {}                            # Drone3: no companion
```

```bash
./run_cesium_sim.sh --qgc --fleet my_fleet.yml
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
| UDP 15540, 15541 ← 14590, 14591 (+10N) | PX4 ↔ companion computer (with `--pi-ip` or `--fleet`) |
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
