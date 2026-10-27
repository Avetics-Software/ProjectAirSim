# Aircraft: quadrotors and fixed-wings in one swarm

Every drone in the config file has a `model:`, an entry of
[`aircraft/models.yml`](../../aircraft/models.yml). The model says what to
simulate and how the ground station should fly it:

| Model | Aircraft | Autopilot |
|---|---|---|
| `quad_px4` (the default) | Quadrotor with a gimbal camera | PX4 |
| `x8_arduplane` | Skywalker X8 flying wing (JSBSim) with the same gimbal camera | ArduPlane |

```yaml
drones:
  - name: Drone1
    sysid: 1                 # model: quad_px4 (the default)
  - name: Drone2
    sysid: 2
    model: x8_arduplane
```

`./run_cesium_sim.sh --qgc --config that.yml` then flies both in one world,
from one ground station. Real drones use the same key in their fleet file, so
the ground station knows which autopilot and airframe it is talking to. The
sim's `--model MODEL` sets the model of drones the file doesn't give one (for
example those added with `--drones`).

## How it fits together

Three layers, each holding one kind of fact. This is the same split the
packaged Avetics sim uses ([comparison](../avetics-sim-comparison.md)), made per
drone so the layers can be mixed in one swarm.

| Layer | File | Holds | Never holds |
|---|---|---|---|
| Airframe | `aircraft/airframes/<name>.jsonc` | The physical aircraft: links, actuators, sensors, physics. Its output order per autopilot family, and any settings of its own (the X8's catapult). Where the gimbal camera mounts (`gimbal-mount`) | A port, a gain |
| Autopilot | `models.yml` `autopilots:` | The link: controller type, ports (offset per drone), settings every aircraft on it shares | A gain |
| Model | `models.yml` `models:` | An airframe + autopilot pair: how its SITL starts (frame or binary, parameter files in order) and its `capabilities` | Anything about the airframe or the link |

[`aircraft.py`](../../aircraft.py) combines them into each drone's robot config
(`sim_config/robot_<model>_custom[_N].jsonc`) and its SITL start-up.
[`gcs/airframe.py`](../../gcs/airframe.py) reads only a model's autopilot family
and capabilities, so the ground station still needs nothing from the sim.

```
config.yml  drone: {model: x8_arduplane}
      │
      ├── sim:  aircraft.py ── robot config (airframe + link + gimbal camera)
      │                    └── SITL: px4_sitl.Px4Sitl | ardupilot_sitl.ArduPilotSitl
      │
      └── gcs:  VehicleLink ── .ap        gcs/autopilot.py  (PX4 | ArduPilot on MAVLink)
                           └── .airframe  gcs/airframe.py   (what it can be asked to do)
```

- **The gimbal camera** ([`aircraft/payload_gimbal.jsonc`](../../aircraft/payload_gimbal.jsonc))
  is added to every airframe at its `gimbal-mount`, so all aircraft carry the
  same camera. `camera_properties.json`, RTSP, `--ground-truth` and the MOT
  tools work on any of them.
- **Ports**: drone N (0-based) uses autopilot instance N + the autopilot's
  `instance-offset`. PX4's instance is N; ArduPilot's is N + 5, which keeps
  clear of another ArduPilot on instance 0. Either way the ground station
  finds drone N on UDP 14030+N, and its system id is N+1.
- **The sim scene's home is every autopilot's home**: PX4's `LPE_LAT/LON` and
  ArduPilot's `--home`, so they agree with the scene.
- **Tuning lives only in a model's parameter files.** The X8's is
  `aircraft/params/x8_arduplane.param`; `aircraft/params/arduplane_swarm.parm`
  adds only the swarm's links, failsafes and gimbal mount.
- **Everything the swarm flies is in `swarm/`.** `aircraft.py --audit` fails on
  a model or airframe that reads a file from outside it (other than the
  autopilots' own checkouts).

### Checks

```bash
python aircraft.py --list                    # the models
python aircraft.py --dump x8_arduplane       # the robot config it generates (--index N)
python aircraft.py --audit                   # cross-checks; exits 1 on an error
```

`--audit` checks numbers that describe one fact in two places, because
nothing else would catch them disagreeing:

- every `actuator-order` output has a `SERVOn_FUNCTION`
- the gimbal's travel, which the sim decodes the mount PWM across, equals
  ArduPilot's `MNT1_*_MIN/MAX`, and the mount outputs span 1000–2000 µs
- `MNT1_TYPE` is 7 (a self-stabilising gimbal)

`tests/test_aircraft.py` also runs every generated robot config through the
client's schema. The sim's C++ loaders accept configs that the client rejects.

## What the ground station does with a fixed-wing

The commands ask the drone's capabilities, never its type. A fixed-wing
(`hovers: false`) differs in these ways:

| | Multirotor | Fixed-wing |
|---|---|---|
| `takeoff` | climbs in place | TAKEOFF mode: launched along the way it faces (the sim's catapult), climbs to the height, circles |
| `goto` | flies there and hovers; several line up | flies there and circles it (GUIDED); several circle the same point, apart in height (their layers) |
| `heading`, a goto's heading | turns on the spot | refused: it faces the way it flies |
| `hold` | hovers (PX4 Hold) | circles (LOITER) |
| `land` | lands where it is | its mission's landing sequence (`DO_LAND_START`); refused without one |
| `rtl` | returns and lands | lands through its mission's landing sequence (`RTL_AUTOLAND 2`); without one, circles home |
| Arrived at a point | within `arrive_m` | within `arrive_m` + its loiter radius |
| Giving way | stops and climbs | keeps flying to where it was going, higher. Beside a multirotor, the multirotor gives way |
| Lost link | others keep `lost_margin_m` from its last position | the same plus its loiter radius; it returns home itself after `FS_LONG_TIMEOUT` (`FS_LONG_ACTN 1`) |
| Battery response `land` | lands | returns home instead |
| Mission `hold`, `wait` | NAV_WAYPOINT hold, NAV_DELAY | NAV_LOITER_TIME (circles) |
| Mission `heading` | turns | a validation error |
| Mission `land` | lands here (`true`) or at a point | on a runway: `land: {north, east (or lat, lon), heading}` flies an approach (`approach_m` 400 back, at `approach_alt` 30 m) and lands along `heading` |
| Area search | passes in order, turning on the spot | passes ordered so each is at least twice its turn radius (`turn_radius_m`) from the last |
| Sim faults (safety panel) | link cut: the sim stands in for PX4's failsafe; battery: PX4's battery simulator | link cut: ArduPilot's own GCS failsafe; battery: `MAV_CMD_BATTERY_RESET` |

ArduPilot's own differences stay inside `VehicleLink` (`gcs/autopilot.py`):

- **Modes** use the ground station's names, so the rest of it compares
  them without knowing the autopilot: AUTO is `MISSION`, LOITER is `HOLD`.
  The dashboard shows the autopilot's own name (`mode_name`).
- **Integer parameters**: PX4 packs them bytewise into the float field;
  ArduPilot sends the plain value.
- **Mission numbering**: ArduPilot's item 0 is home. Uploads start with a
  placeholder for it, and item numbers are shifted back, so missions, the
  minimap and run logs count items the same for both autopilots.
- **Missions don't claim the gimbal.** ArduPilot has no
  `DO_GIMBAL_MANAGER_CONFIGURE` mission item (it rejects the upload) and obeys a
  mission's gimbal items whoever holds it. The ground station makes the same
  handovers by command instead (`VehicleLink.give_gimbal`): to the autopilot
  at the mission's start, to nobody at a `track` step, back at its end. X's
  stand-down then works as it does with PX4, as long as the ground station's
  link to the drone is up.
- **Parameter names**: link loss is `FS_LONG_ACTN`/`FS_LONG_TIMEOUT`, and the
  fence is `FENCE_ACTION`/`FENCE_ALT_MAX`, turned on with
  `FENCE_TYPE 5`/`FENCE_ENABLE 1`.

## The plane's gimbal

The ground station steers the plane's camera exactly as it steers the quads':
through the gimbal manager in the autopilot (`GIMBAL_MANAGER_SET_ATTITUDE`,
mission `DO_GIMBAL_MANAGER_PITCHYAW`, ROI). ArduPlane drives a self-stabilising
gimbal on servo outputs 5–7 (`MNT1_TYPE 7`). `arduplane-api` decodes those
outputs back to angles across the travel and drives the same gimbal actuator
the quads use. Checked against ArduPlane 4.7.2 SITL: pitch -45° comes out at
-44.9°, yaw -120° at -120.0°, and ArduPlane reports both back over MAVLink.

## Takeoff and landing

The X8 has no wheels. Fixed-wings spawn on the runway centreline behind the
spawn point (40 m apart; the scenario's test cars are ahead of it), facing
along it, and are placed on the real ground
(0.5 m up, as the quads are) once the Cesium tiles are loaded. That matters:
ArduPlane measures every height from where it arms, so its landing point is
the runway only if it arms on it. `arduplane-api` holds each one on a virtual
catapult until TAKEOFF applies throttle, then launches it at 18 m/s. It lands
on its belly: the JSBSim model has skids (a nose and two rear belly points,
and wingtip skids), and a mission's `land` step flies ArduPlane's landing
approach onto the runway. ArduPlane flares and cuts the motor, and disarms once
it has stopped.

`missions/x8_circuit.yml` flies it on the Seletar runway: a `takeoff`, three
`goto` legs and a `land`. Checked in Unreal on Cesium ground (true pose from
the sim, ArduPlane 4.7.2 SITL): it launched from the runway, flew the circuit
at 18 m/s and 37 m above the ground, descended the glide slope from the 30 m
approach point, touched down at 14 m/s with a 0.4 m/s sink rate, slid about
14 m and stopped level, 12 m past the touchdown point, and ArduPlane
disarmed. (The same mission on the headless runtime's flat ground stopped 47 m
long; the flare floats, and LAND_FLARE_SEC and the approach speed tune it.)
The skid positions and springs are estimates, not measurements.

Two things this needed in the sim. A JSBSim aircraft could not be placed on the
ground after the scene loaded, because `SetPose` was ignored for JSBSim robots
(fixed in the physics step). And the X8 spawned 15 m up, so ArduPlane's
"height 0" was 15 m in the air: it flared there and fell the rest of the way.

A real fixed-wing with wheels gets its own airframe file and JSBSim model
with gear contacts, and can then spawn on the runway without the catapult
(`launch.hold` off) and take off with ArduPlane's TKOFF_* settings.

## Its companion (X)

Each model names the X profile its companion computer runs in the sim
(`companion-profile` in `models.yml`), and the sim logs it for every drone
with a companion: `rknn-sim-quad-px4` for `quad_px4`, `rknn-sim-fw-ardupilot`
for `x8_arduplane`. The profiles differ in the autopilot (`FIRMWARE`), the
airframe (`AIRFRAME`, which picks the control law) and nothing that sees the
sim camera. See the [simulation guide](../simulation.md#x-vision-pipeline-on-a-radxa-a5x_rpi_release-rknn-sim--profiles).

## Adding an aircraft

1. **Airframe**: `aircraft/airframes/<name>.jsonc`. Write the physical
   aircraft, or `extends` an existing robot config and override what differs.
   Add an `autopilot.<family>.actuator-order` per autopilot it can fly with,
   and a `gimbal-mount`.
2. **Model**: an entry in `models.yml` with its `airframe`, `autopilot`, `sitl`
   (PX4: `frame`; ArduPilot: `binary`) and its parameter files, in order.
   Give its `capabilities` as `multirotor`, `fixedwing`, or a mapping such as
   `{kind: fixedwing, loiter_radius_m: 50, cruise_mps: 22}`.
3. `python aircraft.py --audit`, then `python -m unittest discover -s tests`.
   The tests run every model through the audit and the schema.

Start without tuning. Get the aircraft flying on the autopilot's stock
parameters first, then add its own parameter file.

A new autopilot family, for example ArduCopter quads, also needs:
- an `autopilots` entry
- a dialect in `gcs/autopilot.py`, at least its mode table, since ArduCopter's
  modes differ from ArduPlane's
- a SITL class with `start`, `stop` and `restart_modules`
