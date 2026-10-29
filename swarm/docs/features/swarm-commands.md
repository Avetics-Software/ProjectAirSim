# Swarm commands

One command, or one button, for any group of drones. Every command goes to
each drone's autopilot (PX4 or ArduPilot) over MAVLink, the way QGC's would, so
QGC shows what the drones are doing and can take over at any time.

A fixed-wing (the X8) can't hover or turn on the spot, so the commands do the
nearest thing: `goto` flies there and circles it (several fixed-wings circle the
same point at different heights), `hold` circles where it is, `heading` is
refused, `land` needs a mission's landing sequence (fly a
[mission](missions.md) with a `land` step), and `rtl` returns and circles
home (or lands, if the mission has a landing). See [aircraft](aircraft.md).

## Console

Type in the terminal or the dashboard's *Command* box. Name the drones first:
a number (`2`), a list (`1,3`) or `all`.

| Command | Effect |
|---|---|
| `all takeoff [ALT]` | Arm and climb to ALT m above home (default 30), each drone on its own [height layer](deconfliction.md) |
| `2 goto LAT LON [ALT] [HEADING]` | Fly there and hover (a fixed-wing circles it). Several drones form an east-west line 15 m apart, centred on the point. With HEADING (degrees from north) the drone turns to face it on arrival. `-` for ALT keeps the current height |
| `2 heading DEG` | Turn in place to face DEG from north (not a fixed-wing) |
| `1,3 hold` / `land` / `rtl` | Hover / land where they are / return home and land (a fixed-wing: circle / its mission's landing / return and circle) |
| `1,3 searcharea ALT SPACING LAT,LON;...` | [Search an area](area-search.md), split between the drones |
| `all search LAT1 LON1 LAT2 LON2 [ALT] [SPACING]` | The same for a lat/lon box |
| `status` | Each drone's mode, arming, height, position, position rate and current task |
| `select N` | Show and steer drone N in the dashboard (or keys 1-9) |
| `gimbal PITCH YAW [N]`, `zoom PERCENT [N]` | Point or zoom a drone's camera (default: the selected one) |
| `strike on\|off [N]` | Arm or release STRIKE on a drone's companion, as the dashboard's STRIKE button (default: the selected one) |
| `alerts`, `ack`, `fence ...`, `safety ...` | Alerts, the geofence and safety settings; see [safety](safety.md) |
| `help` | Every command |

A new command for a drone replaces what it was doing. The sim adds its own
commands (`goto` a new location, objects, moving actors); see the
[simulation guide](../simulation.md).

## Dashboard control bar

The same without typing:

- **Drone** buttons pick the drone the cameras show and the buttons command.
  *Buttons command all drones* sends the flight buttons to every drone.
- **Take off**, **Hold**, **Land**, **Return home** with the *Height* box.
- **Go to target**: click the minimap to set the selected drone's target,
  then press it. With a *Heading*, the drone turns to it on arrival.
  **Set heading** turns the drone in place.
- **Heading**: type it, or click the minimap's compass (bottom right) on the
  direction you want; it rounds to 5° and fills the box. The picked heading
  shows on the compass as a hollow pointer (and `> 135` by the drone's
  heading) until you send it with *Set heading* or *Go to target*.
- **Search area**: draw the area on the minimap and press it (see
  [area search](area-search.md)).
- **Messages** below show replies, warnings and what each drone is doing.

## Shortcuts

**F1** opens a sheet of every shortcut, over the camera pane, grouped into
Camera, Map, Drones and Safety; it lists only the keys that work for the
selected drone (the companion's keys when it has one). F1 or Esc closes it.
The buttons with a shortcut (drones, views, Safety, Sim, HOLD ALL, RTL ALL)
show it when the pointer rests on them, and the status panel keeps one line: `F1
shortcuts  F2 safety and alerts  F3 sim settings  Ctrl+H hold all  Ctrl+R return all`.

In the sim, **F3** (or the **Sim** button) opens the sim settings drawer where
the safety drawer goes (one at a time). Its tabs:

- **Camera:** resolution, aperture, sensor noise, lens distortion and vibration
  of every GimbalCam. See [changing the camera while the sim runs](../simulation.md#changing-the-camera-while-the-sim-runs).
- **Time of day:** a time and date at the site, Now, and quick times (dawn,
  morning, noon, afternoon, dusk, night). See [time of day](../simulation.md#time-of-day).

## The map and status panel

- **Minimap:** every drone (the selected one yellow, the others blue), where
  each is heading, its remaining route, predicted crossings and a compass with
  the selected drone's heading. `[`/`]` zoom, `L` toggles labels. In the sim,
  spawned objects and moving people and vehicles are drawn too.
- **Status:** each drone's mode, arming, height, heading and speed, all from
  its MAVLink telemetry; `LINK LOST` after 3 s without a position. Companions'
  video and tracking state are listed under the drones.

## How commands run

`gcs/swarm.py` runs each drone's command as its own task, so drones carry out
their commands independently. A command waits for PX4 to confirm it, retrying
arming until PX4's estimator is ready. Searches are PX4 missions and live on
the drone; `goto` uses PX4's reposition command, which also flies on if the
ground station stops.
