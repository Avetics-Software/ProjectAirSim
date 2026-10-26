# Repeatable missions

A YAML file gives each drone a scripted flight, including aircraft heading
and camera pitch/yaw. Native PX4 missions execute the steps onboard without
the laptop timing every move. Tracked in [#32](https://github.com/Avetics-Software/ProjectAirSim/issues/32).

Start the simulator with `./run_cesium_sim.sh --qgc --drones 2`, then type in
the dashboard command box or terminal:

```text
mission load example
mission show
mission upload
mission start
```

The same commands work on real drones. `missions/example.yml` lists two
drones; use a file listing only your drone when flying one. `mission upload 1`
and `mission start 1` select Drone1; no number selects every drone in the
file. Every selected drone must have a successful upload before start.
Starting again restarts the uploaded mission at item zero.

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
| `takeoff` | Height above home, first step only; skipped if uploaded airborne. If omitted on the ground, takeoff to the configured default height is inserted |
| `goto` | `{north, east, alt}` in metres, or `{lat, lon, alt}`. Height is above each aircraft's home; omitted height keeps the previous step's height |
| `heading` | Aircraft heading in degrees clockwise from north, kept for later steps until another `heading`; `travel` lets PX4 face the way it flies. Without `goto`, turn at the previous waypoint |
| `gimbal` | Pitch in degrees, negative down; `yaw` relative to the nose, or `heading` as a compass bearing with yaw lock. Omitted yaw/heading uses nose-relative zero |
| `hold` | Seconds at the waypoint; when combined with a camera step, change the camera first, then hold |
| `look_at` | Point the camera at a place (ROI), in either coordinate format, optional `alt` (default ground at home height); re-aimed at each later waypoint; `none` clears it and centres the camera |
| `wait` | Seconds of delay, including after a camera-only step |
| `track` | Hand the camera to a tracker while the drone keeps flying: `{target: NAME}` (a sim object to lock onto and score; optional `tracker: auto\|x\|truth`), `on` (real drones: pick the target yourself), `off` to take it back. A `gimbal` or `look_at` step also takes it back. See [Tracking from a mission](#tracking-from-a-mission) |
| `photo` | `true` sends a native image-capture command |
| `land`, `rtl` | `true` ends with landing or return home; no later steps allowed |

Camera steps on a `goto` apply on arrival. Put a camera step **before** the
`goto` to set it for that flight leg. Gimbal travel/slew limits still apply.
Zoom steps and QGC `.plan` import are not supported in this version.

How PX4 runs the camera steps (the station allows for these):

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

## X and camera ownership

Deploy the mission-aware MAVLink gimbal backend from `A5X_RPI_Release` to
each companion, with these settings in its active `config_gimbal.yml`:

```yaml
MAVLINK_STAND_DOWN_IN_MISSION: true
MAVLINK_RECLAIM_GIMBAL: true
MAVLINK_CLAIM_MODE: explicit
```

The local `rknn-sim` profile enables these. Older deployed binaries need
updating too; the laptop cannot confirm this setting through X's current
UDP protocol.

**Who has the camera:** the mission claims the gimbal for the autopilot right
after takeoff, and X stands down while the drone is in MISSION mode **and**
the autopilot holds the gimbal (PX4's `GIMBAL_MANAGER_STATUS`; not yet
reported counts as holding it). It reads both from PX4 itself, so this holds
without the ground link. A `track` step gives the gimbal to nobody; X re-claims
it and tracks as usual until `track: off` (or a `gimbal`/`look_at` step) claims
it back. The station applies the same rule to the dashboard's controls.

The station cancels the selected track and held jogs before start and blocks
new manual gimbal/tracking input while the mission holds the camera. Video and heartbeat
continue. Zoom remains manual. After a hovering mission completes, the station
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

Not yet tested: a live Radxa running X through a mission (X's stand-down and
hand-back were checked against stand-in heartbeats and gimbal status only),
and real drones.
