# Vision and tracking

Each drone can carry a companion computer (a Radxa) running the X vision
pipeline from A5X_RPI_Release. X reads the drone's gimbal camera, detects
targets, tracks the one the operator picks and steers the gimbal to keep it
centred. The ground station is X's ground station for every drone in the
swarm.

## Setting it up

Give a drone a companion with `--pi-ip IP` (Drone1) or a fleet file (any
drone):

```yaml
drones:
  - companion: 192.168.199.99     # Drone1
  - companion: 192.168.199.100    # Drone2
```

In the sim, each Radxa runs X's `rknn-sim-*` profile for its drone's model, whose `INPUT_RTSP` must
name its own drone's stream: `rtsp://<laptop>:8554/gimbal1` for Drone1,
`/gimbal2` for Drone2, and so on. The sim's PX4 for that drone sends it
MAVLink on UDP 15540/15541. Deploying and running X is in the
[simulation guide](../simulation.md#x-vision-pipeline-on-a-radxa-a5x_rpi_release-rknn-sim--profiles).

## In the dashboard

When the selected drone has a companion, the main pane shows **Tracker**:
X's own video (`rtsp://<radxa>:8554/tracker`) with its detection boxes and
the track drawn on it. GimbalCam or ChaseCam goes in the small pane beside it
(*View*, or V). Keys 1-9 switch drones, and with them the Tracker shown.

| Input | What X does |
|---|---|
| Click a target | Track it (`CLICK:x,y`) |
| Drag a box around a target | Track that box: `BOX:size`, then `CLICK` at its centre |
| Enter an object ID | Track the confirmed MOT track with that `ID:<n>` (`TRACK:<id>`), no click |
| Right-click or Esc | Stop tracking (`CANCEL`) |
| Arrow keys | Move the tracked point 8 px |
| W/A/S/D (hold) | Jog the gimbal; stops on release. Cancels a track first |
| G or C | Centre the gimbal. Cancels a track first |
| Wheel or +/- over the camera | Zoom in or out |
| **STRIKE** button (top right) | `STRIKE:START`; pressed again, `STRIKE:STOP`. Greyed out on a drone without a companion |

What X does with STRIKE depends on its profile: with `RUN_DRONE_CONTROL_THREAD`
off (the `rknn-sim-*` default) it only records it. The ground station refuses
to arm while a [mission](missions.md) holds the camera, and releases an armed
strike when a mission takes the camera or the ground station closes. The
console's `strike on|off [N]` does the same as the button.

The status panel shows whether each companion's video is arriving,
whether it is tracking and whether STRIKE is armed. The GimbalCam title shows the gimbal's real angle,
reported by PX4, since X is the one moving it.

## Who controls the camera

| | Drone with a companion | Drone without |
|---|---|---|
| Gimbal | **X**, through PX4's gimbal manager: the dashboard's keys go to X | The ground station, through PX4's gimbal manager |
| Zoom | **X** (`ZOOM:IN/OUT`), forwarded by PX4 to the camera | The ground station (in the sim, the sim camera directly) |
| Console `gimbal` / `zoom` | Refused, with a pointer to the keys | Steer / zoom directly |
| GimbalCam title | `(via X)` | `(via PX4)` or `(via ArduPilot)`, the drone's autopilot |

So the ground station never takes the camera from X. X keeps it too: with
`MAVLINK_RECLAIM_GIMBAL` (on in `rknn-sim-*`) it takes the gimbal back when PX4
hands it away around takeoff and return-home, and it repeats a held jog every
0.5 s, as PX4 drops a gimbal rate that isn't refreshed within 2 s.

The exception is a [mission](missions.md). While a mission holds the camera,
X stands down (`MAVLINK_STAND_DOWN_IN_MISSION`) and the dashboard's gimbal
keys are blocked on that drone. X still detects and tracks; only its gimbal
commands stop. A mission's `track` step hands the camera back to X for a
stretch, and it returns when the mission ends.

## How it works

`gcs/companion.py` speaks X's ground-station protocol, the same as A5X's
`map-plot/laptop_client.py`: text commands over UDP to port 5556 and a
heartbeat every 0.2 s, so X knows a ground station is there. The Tracker video
is read by `gcs/video.py`, which keeps only the newest frame.

X steers the gimbal itself, through PX4's gimbal manager, so the dashboard
doesn't send its own gimbal commands to a drone with a companion: they would
take the gimbal from X.

## Behaviour to know

- **X ignores manual gimbal and zoom commands while tracking**, so they don't
  fight its tracking loop. That is why W/A/S/D and G cancel the track first.
  Zoom does nothing until the track ends.
- **Featureless ground**, such as an empty runway, gives the tracker nothing
  to hold on to; it can slide to a similar-looking patch.
- **X's gains assume 1x zoom.** The sim camera doesn't report its zoom to X,
  so zoomed in, X steers too hard and can overshoot.
- **Detections stay in X's video for now.** Showing them on the map, and
  recognising the same target across drones, are on the
  [roadmap](../roadmap.md) (issue #7).
