# Deconfliction

PX4 knows nothing about the other drones, so nothing on a drone stops it
flying into another. The ground station keeps them apart in three layers, each
catching what the one before can miss.

## 1. Height layers

Drone N flies (N − 1) × 5 m above the height asked for: `all takeoff 30` puts
Drone1 at 30 m, Drone2 at 35 m, Drone3 at 40 m. Routes that cross then pass at
different heights. Set the step as `layer_step_m` in the config file's
`swarm:` section (`swarm/config.yml` in the sim, the fleet file for real
drones), or with `--layer-step` in the sim; 0 puts every drone at the height
asked for. The separations, look-ahead and give-way distances below are the
config file's `deconfliction:` section.

## 2. Crossing check, when a route is planned

Every new route (a `goto`, a search) is compared with every other drone's
remaining route (`gcs/search_planner.py`, `find_crossings()`). Where two come
within 6 m of each other, the crossing is reported in the messages and marked
on the minimap:

- **green** if the drones' heights keep them at least 3 m apart there
- **red** if not: the separation monitor (below) will make one give way if
  they actually meet, or the operator can change the plan

This is a warning only; it doesn't change the routes. Up to three places are
reported per pair of drones.

## 3. Separation monitor, while flying

`gcs/swarm.py`, `_separation_monitor()`, four times a second:

1. For every pair of airborne drones, it takes their latest positions and
   velocities, brings them up to the present (positions arrive only a few
   times a second), and predicts their closest approach in the next 6 s,
   assuming they keep their velocities.
2. If they would come within **6 m horizontally and 3 m vertically**, the
   higher-numbered drone gives way: it stops where it is and climbs 6 m.
3. Once that drone is **10 m** from every other drone and no longer closing on
   any, it resumes what it was doing: its mission from where it stopped, its
   `goto`, or a hover.

The monitor predicts rather than waiting until the drones are close because
of the delay: positions are a fraction of a second old, and a drone takes time
to stop. It watches every airborne drone however it is being flown, including
from QGC.

## Limits

- **It runs on the ground station.** If the link to a drone drops, nothing
  protects that drone. Each drone's own failsafes (geofence, data-link-loss
  action) still matter.
- **It trusts the drones' GPS.** Tested on the sim's clean positions; real GPS
  error of a few metres eats into the 6 m.
- **Straight-line prediction.** A drone about to turn can be predicted wrong
  for the next few seconds.
- **Heights are above home, not terrain.** Layers keep drones apart from each
  other, not from hills or buildings.

Moving the monitor onto the companion computers, with drones telling each
other where they are, would make it work without the ground link; see the
[roadmap](../roadmap.md).
