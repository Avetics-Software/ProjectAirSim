# Roadmap

What the swarm ground station still lacks, in the order to tackle it. The
first group matters before any real multi-drone flight.

## 1. Safety

- **Drone health in the status panel:** battery, GPS fix and satellites,
  link quality and PX4's warnings, next to the mode and height shown today.
- **Alerts:** a short list of what needs attention now (low battery, link
  lost, geofence breach, predicted conflict), instead of messages scrolling
  past in the log.
- **Responses on the ground:**
  - a lost link is shown but nothing reacts; check each PX4's data-link-loss
    action, or add a ground policy
  - low battery: return home, and hand the drone's remaining work to others
  - **hold all / return all** buttons for emergencies
- **Geofence:** upload one to every drone from the ground station.
- **Deconfliction that survives a lost link:** run the separation monitor on
  the companions, with drones telling each other their positions, instead of
  only on the ground. Also account for GPS error and latency, and fly height
  layers above terrain rather than above home. See
  [deconfliction](features/deconfliction.md#limits).

## 2. Tasking that adapts in flight

- **Coverage tracking:** record the ground each camera has actually seen, not
  only the waypoints flown.
- **Re-planning:** when a drone drops out, share its unflown passes among the
  others.
- **Spacing from the camera:** set pass spacing from the camera's footprint at
  the height and zoom flown.
- **More patterns:** expanding square and sector searches; orbiting and
  following a target.

## 3. A shared picture of targets (issue #7)

- **Detections on the map:** each X's detections, geolocated (X already
  estimates where a target is).
- **ReID across drones:** the same person or vehicle seen by two drones is one
  target. The sim logs every moving actor's true position
  (`ground_truth_*.csv`) to score this against.
- **Hand-off:** tell another drone to take over tracking a target, and hand
  tracking between drones as they move.
- **A stabilised gimbal camera:** hold the camera steady in the world frame
  (roll and pitch, and yaw when locked) while the drone accelerates, brakes,
  turns or is pushed by wind, as a real 3-axis gimbal does. Sudden drone
  movements should then not throw the target across the frame or out of it,
  and break X's track. Model the gimbal's own limits too (slew rate, travel,
  settling lag), so tracking is tuned against realistic behaviour. Check first
  how far the sim's gimbal already compensates the drone's attitude.

## 4. The dashboard

- **Every Tracker at once:** a grid of all drones' tracker videos, not only
  the selected drone's.
- **The map:** satellite imagery, free pan and zoom (it now follows the
  selected drone), measuring distances.
- **Joystick or gamepad**, as A5X's `laptop_client.py` has.
- **Save and reload** search areas and plans.
- **X's gains with zoom:** have the camera report its zoom to X, so tracking
  doesn't overshoot when zoomed in.

## 5. Backend and reliability

- **Automated tests:** the shared-link and ground-station checks run so far
  were one-off scripts against stand-in PX4s. Make them tests and run them on
  every change.
- **Recording and replay** of telemetry and commands, to review a sortie.
- **Split the core from the window,** so several operators, or a viewer on
  another machine, can connect.
- **Link security:** MAVLink signing.
- **ArduPilot,** if any airframe needs it; gimbal control assumes PX4's
  gimbal manager today.
