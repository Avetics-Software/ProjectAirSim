# Architecture

A small Python package, `gcs/`, that anyone on the team can read end to end
and extend: new search patterns, detection layers, ReID, safety checks.

## The pieces

```
                         dashboard (Tk)          console (terminal / command box)
                               │                          │
                               └──────────┬───────────────┘
                                   GroundStation (station.py)
               ┌──────────────┬───────────┼──────────────┬────────────────┐
         SwarmControl    DroneCamera   CompanionLink    MiniMap      hooks (sim only:
         (swarm.py)      per drone     per drone        (minimap.py)  objects, actors,
               │              │        (companion.py)                 goto)
         search_planner       │              │
               │              │              └── UDP 5556 + tracker video ──► X on the Radxa
         VehicleLink per drone (mavlink_link.py)
               │
         MavConnection per link ──── MAVLink ────► PX4 or ArduPilot on each drone
```

| Module | Does |
|---|---|
| `station.py` | `GroundStation`: which drone is selected, the console, map and status from telemetry, the UI loop. `DroneCamera`: a drone's video, gimbal and zoom |
| `dashboard.py` | The window: camera panes, minimap, status panel, control bar, keys and mouse |
| `swarm.py` | Swarm commands, height layers, crossing check, separation monitor |
| `safety.py` | `SafetyMonitor`: health rules, alerts, responses (low battery, lost link); the fence check |
| `safety_panel.py` | The safety drawer (F2): alerts, health, responses, geofence, sim faults |
| `sim_panel.py` | The sim settings drawer (F3, sim only): every GimbalCam's resolution, aperture, noise, distortion and vibration, and the time of day, as `sim camera` and `sim time` commands |
| `search_planner.py` | Splitting an area into balanced lawnmower blocks; route crossings |
| `mavlink_link.py` | `MavConnection` (one link, shared by the drones on it) and `VehicleLink` (one drone: telemetry, commands, missions, gimbal, zoom) |
| `autopilot.py` | Where PX4 and ArduPilot differ on MAVLink (modes, parameters, mission numbering, failsafe and fence parameters): `VehicleLink.ap` |
| `airframe.py` | What a drone's model (`aircraft/models.yml`) can be asked to do (hovers, loiter radius, speeds): `VehicleLink.airframe`; see [aircraft](aircraft.md) |
| `companion.py`, `video.py` | X's ground-station protocol; RTSP readers keeping the newest frame |
| `minimap.py`, `road_routes.py` | The map; OpenStreetMap roads |
| `fleet.py`, `settings.py` | The config / fleet file: drones, and the swarm, deconfliction and control settings (safety's are in `safety.py`) |
| `__main__.py` | `python -m gcs`: the station for real drones |

## Threads and loops

- One **asyncio** loop runs the dashboard (input handled every 15 ms, redrawn
  up to 30 times a second), the console, the map update (1 Hz), the safety
  monitor (2 Hz), the separation monitor (4 Hz) and each drone's current
  command.
- Each **MAVLink link** has a reader thread and a heartbeat thread. Telemetry
  is written straight onto each drone's `VehicleLink` attributes; commands
  that wait for PX4's reply run in a worker thread so the window never freezes.
- Each **video stream** has a reader thread keeping only the newest frame.

## Extending it

**A new swarm command:** add a branch to `SwarmControl.run_command()` in
`swarm.py` and a line to `SWARM_HELP`. Run per-drone work with
`self._start(i, coroutine)`, so a new command replaces the old one.

**A new search pattern:** add a planner next to `plan_area_search()` that
returns one `[(lat, lon), ...]` path per drone, and fly it with the existing
`search_area()` machinery (missions, layers, crossing check).

**Something new on the map** (detections, say): `GroundStation` takes
`extra_objects()`, returning `(name, kind, north, east, heading)` tuples; give
the kind a colour in `minimap.py`'s `KIND_COLOURS`.

**New status lines:** `extra_status()`.

**A new safety rule:** add a `_check_...` method to `SafetyMonitor` in
`safety.py` that puts `key: (drone, severity, text)` into `found`, and call it
from `evaluate()`; a response goes in `_respond()`. Raising, updating,
clearing, logging and the drawer follow.

**Extra commands for one setup:** `extra_commands(line, parts)`, returning
True if it handled the line. The sim adds `goto`, objects and actors this way.

**A different camera:** subclass `DroneCamera` (`steer`, `set_zoom`,
`nudge_zoom`, `zoom_text`, `tick`) and pass a factory to `GroundStation`. The
sim's `SimPayload` is one.

## Rules it keeps

- **No Project AirSim imports in `gcs/`.** Sim-only code lives in the files
  around it and plugs in through the hooks.
- **Everything about a drone comes from MAVLink**, never from the sim's ground
  truth, so the sim tests what real drones will see.
- **QGC stays usable alongside.** Commands go to PX4 the way QGC's do, and the
  ground station re-claims the gimbal only when the operator steers it.
