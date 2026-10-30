# Repeatable missions

A YAML file gives each drone a scripted flight: where to fly, the aircraft's
heading, the camera's pitch and yaw, holds, photos, a landing. The ground
station turns it into the autopilot's own mission and uploads it, so it flies
onboard at the autopilot's timing, the same way every run, and keeps flying if
the link drops. It works with PX4 and ArduPilot drones, quadrotors and
fixed-wings (the sim's X8). Tracked in [#32](https://github.com/Avetics-Software/ProjectAirSim/issues/32).

## Write and fly your own mission

**1. Start from a template.** In `swarm/`:

```bash
cp missions/template.yml missions/my_mission.yml              # a quadrotor
cp missions/template_fixedwing.yml missions/my_mission.yml    # the X8 (lands on a runway)
```

Edit the steps. Every one is commented in the template, and [Format](#format)
below lists them all. Positions are `north`/`east` metres from where the drone is
when you upload (`origin: start`), or `lat`/`lon`; heights are above where it
took off. To read a place off in the sim, click it on the dashboard's minimap,
or type `status`, which prints each drone's position.

**Or plan it on the map**, the way QGC's plan view works, instead of typing
coordinates ([#33](https://github.com/Avetics-Software/ProjectAirSim/issues/33)):

```bash
python actor_editor.py --mission my_mission                       # a name in missions/, or a path
python actor_editor.py --mission my_flight --model x8_arduplane   # check it as the X8
./run_cesium_sim.sh --qgc --edit-mission my_mission               # plan first; the sim starts on "Save & start"
```

A name that doesn't exist yet starts an empty mission; an existing file opens
with its route drawn. See [Planning on the map](#planning-on-the-map) below, then
carry on at step 3 (the file it writes is the same one, checked in step 2).

**2. Check it, with no sim.**

```bash
python -m gcs.mission missions/my_mission.yml                        # for the drones in config.yml
python -m gcs.mission missions/my_mission.yml --model x8_arduplane --drones 1
```

It loads the file as the ground station will, then prints what each drone's
autopilot would be sent (every item, the route's length and time), so you can
see a quadrotor's and a fixed-wing's missions differ. A mistake stops with a
message naming the step: `error: ... step 3: unknown speeed`, `a fixed-wing has
no heading of its own`, `no steps allowed after land or rtl`.

**3. Run it in the sim.**

```bash
./run_cesium_sim.sh --qgc                         # a PX4 quadrotor, Drone1
./run_cesium_sim.sh --qgc --model x8_arduplane    # the X8 instead
```

When the log says `World ready` (the X8 also needs its GPS, about 30 s more),
type in the terminal or the dashboard's command box:

```text
mission load my_mission      # a name in missions/, or a path
mission show                 # the steps, and whether each drone has it uploaded
mission upload               # to every drone in the file (mission upload 1: Drone1 only)
mission start                # arms, flies it; mission start 1: Drone1 only
```

Or have the sim load and upload it once the world is ready, so only `mission
start` is left to type: `./run_cesium_sim.sh --qgc --mission my_mission`.

Several drones, or a mix of aircraft: list each drone in the mission file by
name and give it its `model:` in `config.yml` (see [aircraft](aircraft.md)).

**4. Watch it, and stop it.** The map draws the route and the camera steps, the
drone's status line shows its mission item, and `mission start` prints the run's
CSV (5 Hz: position, height, heading, mode, mission item, gimbal angles) in
`runs/`. `mission stop` stops it (and `Ctrl+H` / the dashboard's HOLD ALL holds
every drone); see [Repeat and compare](#repeat-and-compare).
After it lands, the dashboard's **Disarm** button (or `N disarm`) stops the
motors; it is refused while the drone is in the air or moving, and
`N disarm force` overrides that (the aircraft drops).

**5. Change it and fly again.** Edit the file and `mission load` it again, then
`mission upload` and `mission start`. A drone ends where it landed, so for the
next run from the start point reload the world, which puts every drone back:
`goto 1.413425 103.865 33` (the default spawn; see
[changing location](../simulation.md#changing-location)). With `origin: home`
the same file flies the same route from wherever the drones are.

**On real drones** the commands are the same (`./run_real.sh`, then `mission
load` ...). Check the camera and failsafe settings in the [ground station's
checklist](../../gcs/README.md#before-flying-it-for-real) first.

### Planning on the map

The editor's **Drone mission** tab is a list of steps beside a satellite map:

- **Click the map** to add what the tool above the list says: a waypoint (the
  height is the last one's), a look-at point, or a landing. A quadrotor's
  landing is a waypoint then `land`; a fixed-wing's is a runway landing, its
  heading taken along the last leg (edit it in the step).
- **Pins are numbered by step.** Drag one to move it; click one, or a row, to
  edit that step: movement (take off, waypoint, land, return home, or none),
  position and height, heading, hold or wait, speed, camera pitch and yaw (from
  the nose, or a compass bearing), photo, track, and the look-at point.
  ↑ ↓ ✕ on a row reorder or remove it; **+ Add step** inserts any other step
  after the selected one.
- **The check is the ground station's.** Each change is checked as that drone's
  aircraft will have it uploaded (pick the aircraft beside the drone; it
  defaults to its `model:` in `config.yml`, or `--model`). The route is drawn,
  with its length and time, or the step that is wrong turns red with the same
  message as `python -m gcs.mission`: a fixed-wing refuses a heading step.
  Every drone's route is drawn at once; the selected drone's steps are the ones
  you edit. The aircraft chosen here is for the check only; the sim and ground
  station read it from `config.yml`.
- **Save mission** writes the file (the previous one is kept as `.bak`, and
  the comment block at its top stays). Places are saved as `lat`/`lon`: a file
  with `north`/`east` steps is converted from where the drones spawn
  (`--lat`/`--lon`), so a mission with `origin: start` keeps its meaning only
  if it is flown from there. Comments between steps aren't kept.

### Editing the scenario beside the mission

A mission's `actors:` file (the people and vehicles it spawns) is edited in the
same page: the **Traffic** tab is the [moving-actor editor](../simulation.md#moving-people-and-vehicles),
for the file the mission names (the *Traffic file* box sets it). On the Drone
mission tab the traffic is drawn faintly under the drone's route, so you can
see where they meet; set a car's *Start along the path* and *Stop short of the
end* (`start_at_m`, `stop_short_m`) to place it in a convoy. *Save & start*
(from the launcher) saves both.

### What differs for a fixed-wing

A fixed-wing can't stop or turn on the spot, so the same steps mean a little
more or less. The checker above reports each case.

| Step | Quadrotor | Fixed-wing (the X8) |
|---|---|---|
| `takeoff: H` | climbs in place | launched (the sim's catapult), climbs to H and circles |
| `goto` | flies there; stops if there is a `hold` | flies through it; `hold`/`wait` make it circle there |
| `heading` | turns the aircraft | an error: it faces the way it flies |
| `speed` | ground speed | airspeed (the X8 cruises at 18 m/s, stalls near 12) |
| `land: true` | lands where it is | an error: give a runway |
| `land: {north, east, heading}` | lands at that point | flies an approach `approach_m` (400) back at `approach_alt` (30 m), then lands along `heading` |
| `rtl: true` | returns and lands | returns and circles home (lands if the mission has a landing) |
| camera steps, `look_at`, `track` | work | work the same |

### With a companion (X)

Give the drone a companion (`--pi-ip`, or `companion:` in `config.yml`) and run
the X profile for its aircraft: the launch log names it (`Drone1 (x8_arduplane):
its companion ... should run X's rknn-sim-fw-ardupilot profile`). During a
mission X stands down while the mission holds the camera, and takes it back for a
`track` step: see [X and camera ownership](#x-and-camera-ownership).
`missions/x8_track.yml` is a ready example for the X8.

### If something goes wrong

| You see | Why |
|---|---|
| `mission upload failed: ... rejected (MAV_MISSION_RESULT 8)` | The autopilot refused an item's parameter 3. ArduPilot refuses a NaN in an item's parameters 1-4; a gimbal step needs a `pitch` on an ArduPilot drone |
| `mission start: upload it first` | A drone in the file has no successful upload (`mission show`); a `goto` reload also clears uploads |
| `PX4 refused (...)` or `ArduPilot refused (...)`, or no arming | The autopilot isn't ready; arming is retried for 60 s. The X8 needs its GPS first (30-45 s after `World ready`) |
| A fixed-wing won't arm again after a landing | Reload the world (`goto 1.413425 103.865 33`); it is back on the runway with a fresh autopilot |
| `no drone 'Drone2'` | The mission names a drone `config.yml` doesn't list: add it there, or `--drones 2` to the checker |

## Format

```yaml
mission: camera_pass
origin: start
speed: 6
drones:
  Drone1:
    steps:
      - takeoff: 30
      - goto: {north: 80, east: 0, alt: 30}
        heading: 90
        gimbal: {pitch: -45, yaw: 20}
        hold: 5
      - gimbal: {pitch: -60, heading: 180}
      - goto: {north: 80, east: 50, alt: 30}
        heading: 0
      - wait: 3
      - rtl: true
```

| Key | Meaning |
|---|---|
| `mission` | Run filename prefix; letters, digits, underscores, dots and hyphens |
| `origin` | `start`: offsets from each drone's position **at upload**; `home`: offsets from the station's common map origin |
| `speed` | Metres/second; fleet default, per-drone override, or a step |
| `record` | `true` (or a list of drones): in the sim, record each run frame by frame for tracking experiments (video, the drone's metadata, true boxes); see [MOT experiments](mot-experiments.md) |
| `actors` | The sim's moving actors for this mission (a file like `config_moving_actors.yml`, relative to the mission), spawned afresh at each `mission start` |
| `fence` | Optional geofence for the mission: `corners` (3 or more, `{lat, lon}` or `{north, east}` metres from home) and optional `ceiling` (metres above home; replaces the safety settings' `fence_max_alt_m`). `mission upload` sends it to every drone, with the safety settings' fence action, before the mission items, as PX4 refuses waypoints outside the fence it holds; if a drone doesn't take it, the mission isn't uploaded. It stays in force afterwards (`fence clear` removes it), and so does its ceiling. `python -m gcs.mission` reports a waypoint outside it or above its ceiling as a mistake |
| `takeoff` | Height above home, first step only; skipped if uploaded airborne. If omitted on the ground, takeoff to the configured default height is inserted |
| `goto` | `{north, east, alt}` in metres, or `{lat, lon, alt}`. Height is above each aircraft's home; omitted height keeps the previous step's height |
| `heading` | Aircraft heading in degrees clockwise from north, kept for later steps until another `heading`; `travel` lets PX4 face the way it flies. Without `goto`, turn at the previous waypoint |
| `gimbal` | Pitch in degrees, negative down; `yaw` relative to the nose, or `heading` as a compass bearing with yaw lock. Omitted yaw/heading uses nose-relative zero |
| `zoom` | Optical zoom on arrival: a factor (`2x`: twice the lens's widest focal length; in the sim the ZR10's 10x range makes `2x` 11.1 %) or a percentage of the zoom range (`25`). The ground station sends it (`MAV_CMD_SET_CAMERA_ZOOM`) when the drone moves on from the step's waypoint, as PX4 missions can't zoom, so it needs the station's link. A real drone's camera takes percentages only (its lens range is unknown to the station) |
| `hold` | Seconds at the waypoint; when combined with a camera step, change the camera first, then hold |
| `look_at` | Point the camera at a place (ROI), in either coordinate format, optional `alt` (default ground at home height); re-aimed at each later waypoint; `none` clears it and centres the camera |
| `wait` | Seconds of delay, including after a camera-only step |
| `track` | Hand the camera to a tracker while the drone keeps flying: `{target: NAME}` (a sim object to lock onto and score; optional `tracker: auto\|x\|truth`), `on` (real drones: pick the target yourself), `off` to take it back. A `gimbal` or `look_at` step also takes it back. See [Tracking from a mission](#tracking-from-a-mission) |
| `photo` | `true` sends a native image-capture command |
| `land`, `rtl` | `true` ends with landing or return home; no later steps allowed. A fixed-wing lands on a runway: `land: {north: N, east: E, heading: DEG}` (or `lat`/`lon`; optional `approach_m`, `approach_alt`), see [aircraft](aircraft.md#takeoff-and-landing) |

Camera steps on a `goto` apply on arrival. Put a camera step **before** the
`goto` to set it for that flight leg. Gimbal travel/slew limits still apply.
QGC `.plan` import is not supported in this version.

How PX4 runs the camera steps (the station allows for these; ArduPilot's
differences are below):

- **Gimbal control.** PX4 ignores a mission's gimbal commands unless the
  autopilot holds the gimbal, which a mission doesn't claim by itself. Each
  `gimbal` step is uploaded with a `DO_GIMBAL_MANAGER_CONFIGURE` claim first.
- **`look_at` aims once.** PX4 points the camera at the place when the item
  runs, not continuously as the drone flies. The station re-issues it at every
  later waypoint, so the aim is refreshed there and before any `hold`.
  Long legs: split them into shorter `goto`s for a smoother follow.
- **`look_at` and the nose.** With PX4's default `MIS_MNT_YAW_CTL` 0, an ROI
  turns the *drone* to face it, overriding `heading`. The sim sets
  `MIS_MNT_YAW_CTL 1` (`px4_params.parm`), so the camera turns and the drone
  keeps its headings. **Set it on real drones too.**
- **End of a mission.** PX4 centres the camera when the mission ends or a
  `look_at` is cleared.

ArduPilot differs in three ways. Its missions can't carry the
`DO_GIMBAL_MANAGER_CONFIGURE` claim items (it rejects the upload), and it obeys a
mission's gimbal items whoever holds the gimbal, so the ground station makes the
hand-overs itself, by command (see below). It refuses a NaN in an item's
parameters 1-4, so a gimbal step's unused rates are sent as 0 and a gimbal step
needs a `pitch`. And item 0 of its missions is home, which the station
accounts for, so item numbers (the run log, the status line) count the same as
PX4's.

## X and camera ownership

Deploy the mission-aware MAVLink gimbal backend from `A5X_RPI_Release` to
each companion, with these settings in its active `config_gimbal.yml`:

```yaml
MAVLINK_STAND_DOWN_IN_MISSION: true
MAVLINK_RECLAIM_GIMBAL: true
MAVLINK_CLAIM_MODE: explicit
```

The `rknn-sim-*` profiles enable these. Older deployed binaries need
updating too; the laptop cannot confirm this setting through X's current
UDP protocol.

**Who has the camera:** the mission claims the gimbal for the autopilot (on PX4
with an item right after takeoff; on ArduPilot the station does it by command
once the drone is flying the mission), and X stands down while the drone is in
its mission mode (PX4 AUTO.MISSION, ArduCopter or ArduPlane AUTO) **and** the
autopilot holds the gimbal (`GIMBAL_MANAGER_STATUS`; not yet reported counts as
holding it). It reads both from the autopilot itself, so the stand-down holds
without the ground link (on ArduPilot the hand-overs at `track` steps need the
link). A `track` step gives the gimbal to nobody; X re-claims it and tracks as
usual until `track: off` (or a `gimbal`/`look_at` step) claims it back. The
station applies the same rule to the dashboard's controls. If the autopilot
restarts (the sim's `goto`), X claims the gimbal and requests its streams again
when it hears it.

The station cancels the selected track and held jogs before start and blocks
new manual gimbal/tracking input while the mission holds the camera. Video and heartbeat
continue. Zoom is manual, apart from the mission's own `zoom` steps. After a hovering mission completes, the station
requests HOLD so X can reclaim the gimbal. For land/RTL it follows until
landing or interruption. `mission stop`, emergency commands or another flight
command replace the mission task. A link loss does not release camera control.
If disconnected when a hovering mission finishes, use HOLD after reconnecting
to return manual camera control.

## Repeat and compare

Every start writes a distinct CSV in `runs/` at 5 Hz: position, height,
heading, mode, mission item and reported gimbal pitch/yaw. The map shows
routes and camera steps (P: pitch, Y: nose-relative yaw, H: compass heading).
The drone status line shows progress.

```text
mission runs
mission compare
mission compare RUN_A.csv RUN_B.csv
mission stop
mission clear
```

Comparison matches elapsed-time samples within 0.5 s: mean/maximum position
error, maximum heading difference and maximum gimbal pitch/yaw differences.
`nan` means camera feedback was unavailable.

The sim resets actors to their seeded paths at start. Uploaded coordinates
remain fixed; uploading again with `origin: start` after moving the aircraft
changes the origin, so for repeated runs from wherever the drones landed use
`origin: home`. Recreate spawn, initial headings/camera state and weather
for experiments. Concurrent starts are not hardware-synchronised; arming and
mode-change timing vary. CSV time starts at dispatch. Physics/perception are
not guaranteed deterministic.

Specify height layers explicitly in the file; scripted heights are not
silently offset. Routes feed the crossing check and live separation monitor;
varying-height routes have no single safe crossing height. Separation
manoeuvres can change run timing. PX4's geofence and failsafes remain active.

Each run's ID tags what the sim measures during it: `sim attitude` and
`sim tracktest` files are named `<mission>_<run>_...`, tracking scores are
`runs/<mission>_<run>_track_<drone>.csv`, and with `--ground-truth` the
capture's `mission_runs.csv` has each run's start and track steps in sim
time, to pick its frames out. Full telemetry/command replay is #26.

## Tracking experiments

For multi-object tracking (fixed camera, ID switches), see
[MOT experiments](mot-experiments.md): `record: true` missions with scripted
traffic, replayed through X and scored.

## Tracking from a mission

The mission flies the drone; a `track` step hands the camera to a tracker for
a stretch, so tracking can be tried on a moving target from a moving drone,
the same way every run. `missions/track_vehicle.yml` is the curated one: Drone1
waits by the road with its camera on it, takes over as the sim's Vehicle1 (a
15 m/s loop round a block, reset at `mission start`) comes past, then chases it
round the block for a lap and returns home.

```text
mission load track_vehicle
mission upload 1
mission start 1
```

The tracker (`tracker:`, default `auto`):

- **x**: the drone's companion (X, vision). In the sim the station locks X onto
  the target from the ground truth once it is in the frame and at least
  50 px, as an operator's drag would, then leaves it to X. On real drones,
  `track: on` hands over and you pick the target in the Tracker view.
- **truth** (sim, drones without a companion): the gimbal aimed from the
  target's true position 20 times a second. No vision: the best the gimbal and
  this flight allow, the reference for X's result.

Every rendered frame is scored against where the target really is (offset from
the image centre, in frame or not, distance, size in pixels):
`runs/<mission>_<run>_track_<drone>.csv`, a summary in the messages, and
`mission compare` puts two runs' scores side by side.

When a `look_at` is in force at the hand-over, the mission keeps the camera
pointing there (as a fixed pitch and compass heading) rather than centring it,
so X starts from a view of the road.

## Validation

Run `python -m unittest discover -s tests -v` for offline encoding,
validation, lifecycle and companion input checks.

In the sim (two drones, 2026-10-23), checked against the rendered camera
attitude (`sim attitude`), not only the reported angles:

| Check | Result |
|---|---|
| Gimbal steps, both drones: compass 180 (yaw lock) → body yaw 45 → pitch -60 / yaw -30 | each reached; the lock held 180 until the drone's turn hit the ±160 yaw travel |
| `heading` | held 0 across four legs; 90 / 270 on the example's legs |
| `look_at` past a point on four legs | camera bearing 34 → 18 → -10 → -34 deg at the waypoints, drone heading unchanged |
| Repeatability: the example (`origin: home`) twice from the same spots | Drone1 0.4 m apart on average (5.3 max), Drone2 0.3 m (1.5 max) |
| Completion | hovering, landing and RTL endings detected on three successive runs |

Fixed along the way, in the sim and the station:

- Each gimbal's MAVLink identity is its own drone's: it was system 1, so
  Drone2 onwards reported no gimbal angles.
- The sim gimbal now goes to an angle sent with a zero rate (how PX4 sends
  mission gimbal steps); it used to stay put.
- Its yaw turns back through the front within its travel, instead of the
  short way through the back, where it stuck at the travel limit.

`track_vehicle` in the sim (2026-10-26), truth tracker, two runs from a fresh
start: Vehicle1 in frame 100% / 99% of 90 s (1200+ frames each), offset from
the centre 3.5 / 3.4 deg on average, the car 40-220 m away; the drone's track
0.8 m apart on average between the runs. Its only time out of frame is the
first 0.6 s, slewing from the road to the car.

With a stand-in companion (a UDP listener in X's place): the camera stayed on
the road after the hand-over, the station sent X `BOX:50` and the click 7.7 s
later as the car came down the road, and `CANCEL` at `track: off`. With nobody
steering, the car was in frame 20% of the time: the baseline X has to beat.

Not yet tested: real drones, and X locking onto a target during a `track` step
on an ArduPilot drone (the hand-overs below were).

The X8 in the sim (2026-10-27, ArduPlane 4.7.2, Cesium ground at Seletar):

| Check | Result |
|---|---|
| `missions/x8_circuit.yml`: takeoff, three legs, a runway landing | flown; touched down at 12-14 m/s with a 0.4 m/s sink rate, stopped 12-29 m past the touchdown point, ArduPlane disarmed |
| `missions/x8_track.yml` with a live Radxa running X (`rknn-sim-fw-ardupilot`) | X's log matched the station's to the millisecond: stood down once the mission was flying, resumed at `track` (15 ms), stood down at `track: off` (5 ms), resumed after the landing |
| Gimbal steps on ArduPlane | pitch -60 then -30 / yaw 20 reached; X's own pitch jog moved the camera through ArduPlane |
| A search (`1 search ...`) over 334 m x 334 m at 40 m spacing | all 8 passes flown in the planned order, 91-100% of each covered |

The quadrotor starter, `missions/template.yml` copied to `my_mission.yml` and run
with the steps above (PX4, same day): 15 items uploaded, 104 s, heading 90 held
on the legs, the camera at -45° on arrival, the `look_at` aiming it at the spot,
the compass-bearing step holding 180° while the drone faced 90°, then RTL, the
landing and HOLD.
