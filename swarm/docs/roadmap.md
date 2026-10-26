# Roadmap

What the swarm ground station still lacks, in the order to tackle it. The
first group matters before any real multi-drone flight. Each item has a
GitHub issue in [Avetics-Software/ProjectAirSim](https://github.com/Avetics-Software/ProjectAirSim/issues), linked at its start.

## 1. Safety

Done ([safety](features/safety.md)):

- ([#8](https://github.com/Avetics-Software/ProjectAirSim/issues/8)) **Drone health** on each status line and in the safety drawer:
  battery, GPS, link, PX4's link-loss action and messages.
- ([#9](https://github.com/Avetics-Software/ProjectAirSim/issues/9)) **Alerts:** the alert line, and the drawer's alert list with
  acknowledgement and history.
- ([#10](https://github.com/Avetics-Software/ProjectAirSim/issues/10)) **Responses:** low battery returns the drone home (or lands
  it); a lost drone is kept clear of at its last position. The sim's Sim tab
  cuts links and drains batteries to test them.
- ([#11](https://github.com/Avetics-Software/ProjectAirSim/issues/11)) **HOLD ALL / RTL ALL** buttons and Ctrl+H.
- ([#12](https://github.com/Avetics-Software/ProjectAirSim/issues/12)) **Geofence** drawn on the map and uploaded to every PX4.

Still to do:

- ([#13](https://github.com/Avetics-Software/ProjectAirSim/issues/13)) **Deconfliction that survives a lost link:** run the separation
  monitor on the companions, with drones telling each other their positions,
  instead of only on the ground, and fly height layers above terrain rather
  than above home. (Margins already widen with GPS inaccuracy.) See
  [deconfliction](features/deconfliction.md#limits).
- ([#31](https://github.com/Avetics-Software/ProjectAirSim/issues/31)) **The sim's own link-loss detection:** the simulator keeps PX4's link alive
  with ground-station heartbeats of its own, so the Sim tab sends PX4's
  link-loss action on its behalf. Changing those heartbeats' type in
  `vehicle_apis/multirotor_api/src/mavlink_api.cpp` (needs a plugin rebuild)
  would let PX4 detect it itself.

## 2. Tasking that adapts in flight

- ([#14](https://github.com/Avetics-Software/ProjectAirSim/issues/14)) **Coverage tracking:** record the ground each camera has actually seen, not
  only the waypoints flown.
- ([#15](https://github.com/Avetics-Software/ProjectAirSim/issues/15)) **Re-planning:** when a drone drops out, share its unflown passes among the
  others.
- ([#16](https://github.com/Avetics-Software/ProjectAirSim/issues/16)) **Spacing from the camera:** set pass spacing from the camera's footprint at
  the height and zoom flown.
- ([#17](https://github.com/Avetics-Software/ProjectAirSim/issues/17)) **More patterns:** expanding square and sector searches; orbiting and
  following a target.

- ([#32](https://github.com/Avetics-Software/ProjectAirSim/issues/32)) **Repeatable scripted missions:** native PX4 missions with
  aircraft headings, camera pitch/yaw and ROI, X standing down while the
  mission holds the camera, `track` steps that hand it to a tracker (moving
  target, moving drone, scored against ground truth in the sim), and run
  comparisons. Validated in the sim; see [missions](features/missions.md).
  A live Radxa (X) run and real drones are still pending.

- **MOT experiments** ([guide](features/mot-experiments.md)): scripted traffic
  (convoy, junction, drone motion) recorded frame by frame with the drone's
  metadata and true boxes, replayed through X's detector and ByteTrack
  (`mot_replay.py`), scored for ID switches with their causes (`mot_eval.py`).
  Found and fixed: X's ByteTrack gave a new ID after a single missed
  detection. Pending: the Radxa runs and ByteTrack tuning.

## 3. Targets and tracking

- ([#7](https://github.com/Avetics-Software/ProjectAirSim/issues/7)) **Detections on the map:** each X's detections, geolocated (X already
  estimates where a target is).
- ([#7](https://github.com/Avetics-Software/ProjectAirSim/issues/7)) **ReID across drones:** the same person or vehicle seen by two drones is one
  target. The sim logs every moving actor's true position
  (`ground_truth_*.csv`) to score this against.
- ([#18](https://github.com/Avetics-Software/ProjectAirSim/issues/18)) **Hand-off:** tell another drone to take over tracking a target, and hand
  tracking between drones as they move.
- ([#19](https://github.com/Avetics-Software/ProjectAirSim/issues/19)) **A stabilised gimbal camera:** done
  (yaw lock and smooth follow, slew and travel limits, `sim tracktest`; see the
  [simulation guide](simulation.md#gimbal-stabilisation-and-yaw-modes)). Roll and
  pitch were already held level by the camera mount. Not done: residual jitter
  growing with the drone's angular rates.

## 4. The dashboard

- ([#20](https://github.com/Avetics-Software/ProjectAirSim/issues/20)) **Every Tracker at once:** a grid of all drones' tracker videos, not only
  the selected drone's.
- ([#21](https://github.com/Avetics-Software/ProjectAirSim/issues/21)) **The map:** satellite imagery, free pan and zoom (it now follows the
  selected drone), measuring distances.
- ([#22](https://github.com/Avetics-Software/ProjectAirSim/issues/22)) **Joystick or gamepad**, as A5X's `laptop_client.py` has.
- ([#23](https://github.com/Avetics-Software/ProjectAirSim/issues/23)) **Save and reload** search areas, targets and geofences. Scripted flight plans are #32.
- ([#24](https://github.com/Avetics-Software/ProjectAirSim/issues/24)) **X's gains with zoom:** have the camera report its zoom to X, so tracking
  doesn't overshoot when zoomed in.

## 5. Backend and reliability

- ([#25](https://github.com/Avetics-Software/ProjectAirSim/issues/25)) **Automated tests:** the shared-link and ground-station checks run so far
  were one-off scripts against stand-in PX4s. Make them tests and run them on
  every change.
- ([#26](https://github.com/Avetics-Software/ProjectAirSim/issues/26)) **Recording and replay** of telemetry and commands, to review a sortie.
- ([#27](https://github.com/Avetics-Software/ProjectAirSim/issues/27)) **Split the core from the window,** so several operators, or a viewer on
  another machine, can connect.
- ([#28](https://github.com/Avetics-Software/ProjectAirSim/issues/28)) **Link security:** MAVLink signing.
- ([#29](https://github.com/Avetics-Software/ProjectAirSim/issues/29)) **ArduPilot,** if any airframe needs it; gimbal control assumes PX4's
  gimbal manager today.
