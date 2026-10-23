# Safety

The ground station watches every drone's health, raises alerts, and responds
where it can. When everything is healthy it stays out of the way: one status
line per drone, and nothing else. The detail is in a drawer you open when you
need it.

## On screen

- **Each drone's status line** adds its battery and GPS (`bat 76%  GPS 3D 14`)
  and, if anything is late, the link age. The line turns **amber** for a
  warning and **red** for a critical alert.
- **The alert line**, at the top of the status panel, appears only when there
  is an alert: the most urgent one not yet acknowledged, and how many others
  there are.
- **`HOLD ALL`** and **`RTL ALL`** sit at the right of the control bar.
  `RTL ALL` asks first. **Ctrl+H** holds every drone and **Ctrl+R** returns
  them all (after the same confirmation), from anywhere, even while typing a
  command.
- **F1** shows the full key help (it is one line otherwise). **F2** or
  *Safety* opens the drawer.

## The safety drawer (F2)

It docks at the right of the window; the cameras and map shrink to make room
and grow back when it closes.

| Tab | What |
|---|---|
| **Alerts** | Active alerts, most urgent first. *Ack selected* / *Ack all* take them off the alert line (an alert that gets worse shows again). Below, the recent history of alerts raised and cleared |
| **Health** | Per drone: battery (%, voltage, time left if PX4 estimates it), GPS (fix, satellites, accuracy), link (age, positions per second), what PX4 itself will do if the link is lost, and PX4's latest message |
| **Responses** | The thresholds and responses below, changed while flying (*Apply*) |
| **Geofence** | *Draw fence* (click corners on the minimap), *Undo corner*, *Clear*, the ceiling, PX4's action at the fence, *Upload to all drones* |
| **Sim** | Sim only (also **F9**): fault controls; see [Testing in the sim](#testing-in-the-sim) |

## What is watched, and what happens

| Condition | Alert | Response |
|---|---|---|
| Nothing from a drone for `link_lost_s` (3 s) | critical | The drone can't be commanded. The others keep `lost_margin_m` (15 m) from where it was last seen (the [separation monitor](deconfliction.md) gives way around it). The alert says what PX4 will do on its own (`NAV_DLL_ACT` after `COM_DL_LOSS_T`) |
| Fewer than `min_position_rate` (2) positions/s | warning | none |
| Battery at `battery_warn_pct` (30%) | warning | none |
| Battery at `battery_act_pct` (20%) | critical | `battery_action`: **rtl** (return home, the default), **land**, or **warn** only. The alert says how many search waypoints it left unflown |
| GPS without a 3D fix | critical | none |
| GPS with fewer than `gps_min_sats` (8) satellites or worse than `gps_max_eph_m` (2.5 m) | warning | Separation margins widen by both drones' GPS inaccuracy (up to 10 m) |
| PX4 switched to RTL or LAND without being told (a failsafe) | warning | none; PX4 is already acting |
| Outside the geofence, or above its ceiling | critical | PX4 enforces the fence itself (`GF_ACTION`) |
| Within `fence_margin_m` (10 m) of the fence | warning | none |
| A planned route (goto or search) with waypoints outside the fence, or above its ceiling | warning | none; PX4 will refuse it or stop at the fence |
| A drone giving way to another | warning | the separation monitor's |
| PX4 reports an error (`STATUSTEXT` severity ERROR or worse) | warning, 30 s | none |
| A companion's tracker video stops after it was working | warning | none |

Each alert is raised once, updated while it lasts (a battery alert follows the
percentage), and cleared when its condition ends; everything is logged in the
message box and the history. A response runs once per alert.

Settings come from the config file's `safety:` section
([`swarm/config.yml`](../../config.yml) in the sim, the fleet file for real
drones), the drawer's
*Responses* tab, or the console: `safety battery_act_pct 25` (plain `safety`
lists them all).

### The geofence

Draw it on the minimap and upload it. While *Draw fence* is on, map clicks add
fence corners instead of setting targets, and a banner across the top of the
map says so; **Esc**, *Upload*, leaving the Geofence tab or closing the drawer
ends drawing. A target clicked outside the uploaded fence gets a warning at
once, as PX4 will refuse to fly there. Each PX4 gets the fence as an inclusion
polygon (MAVLink fence protocol), with the ceiling (`GF_MAX_VER_DIST`) and its
action (`GF_ACTION`: warn, hold, rtl or land). **PX4 enforces it itself**, so
it holds without the ground station's link: PX4 refuses `goto`s and missions
that leave it and acts on a breach. The ground station also checks every drone
against it and alerts near and outside it. Uploading an empty fence (*Clear*,
then *Upload*, or `fence clear`) removes it. PX4 keeps an uploaded fence on
disk across restarts, so the sim clears every drone's fence each time it loads
the world (at start-up and on `goto`); a session never starts with an
invisible fence left from an earlier one.

### Console

```
alerts                      list active alerts
ack | ack all               acknowledge the most urgent alert, or all
fence set LAT,LON;...       set the fence's corners (or draw them on the map)
fence upload | fence clear  send the fence to every drone / remove it
safety [KEY VALUE]          show or change a threshold or response
all hold | all rtl          what HOLD ALL / RTL ALL send
```

## Testing in the sim

The drawer's **Sim** tab (F9) injects faults, one row per drone:

- **Cut link** stops all MAVLink between the ground station and that drone's
  PX4, like a radio dropping out. After `COM_DL_LOSS_T` (10 s) the drone does
  its link-loss action, `NAV_DLL_ACT` (the sim sets it to return home, as the
  real drones should be). In the sim this last step is sent on PX4's behalf:
  the simulator keeps its own control link to PX4 open with ground-station
  heartbeats, so PX4 would never notice by itself.
- **Battery %** + *Set* brings the drone's simulated battery to that level
  within seconds and keeps it there while armed (it is full again once
  disarmed).
- **Reset** reconnects, returns the battery to normal, and restarts the
  drone's simulated hardware in PX4 (its battery and power simulators). Use it
  if PX4 refuses to arm with "Battery unhealthy": PX4's battery simulator has
  been seen to hang after a landing. It leaves the modules that link PX4 to the
  simulator alone; restarting a whole PX4 needs the scene reloaded for every
  drone (`goto`).

Or from the console: `sim cut 3`, `sim battery 2 15`, `sim reset 3` (or
`sim reset all`).

The sim's battery (PX4's battery simulator) lasts 20 minutes armed and never
falls below 35% on its own (`px4_params.parm`), so normal sessions don't trip
the 30% warning.

Tested in the sim with three drones:

- **Battery:** 25% gave a warning; 15% gave a critical alert, and the drone
  returned home.
- **Link loss:**
  - a cut link raised `link lost` within 3 s
  - PX4 switched to return home 10 s later
  - on reconnecting, the switch was flagged as a failsafe
- **Geofence:**
  - the fence uploaded to all three PX4s with its parameters
  - PX4 refused a `goto` outside it
  - a drone outside it was flagged critical

## On real drones

- **Set the failsafes on every PX4**, not just in the ground station:
  - `NAV_DLL_ACT` and `COM_DL_LOSS_T` for link loss
  - the battery thresholds and `COM_LOW_BAT_ACT`
  - the geofence

  The ground station's responses come on top of those and need the link to
  work.
- The Health tab shows each drone's `NAV_DLL_ACT` as read from PX4, re-read
  every minute.
- What is not done yet (onboard deconfliction that survives a lost link,
  re-planning a dropped drone's search) is on the [roadmap](../roadmap.md).
