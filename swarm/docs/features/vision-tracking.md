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

In the sim, each Radxa runs X's `rknn-sim` profile, whose `INPUT_RTSP` must
name its own drone's stream: `rtsp://<laptop>:8554/gimbal` for Drone1,
`/gimbal2` for Drone2, and so on. The sim's PX4 for that drone sends it
MAVLink on UDP 15540/15541. Deploying and running X is in the
[simulation guide](../simulation.md#x-vision-pipeline-on-a-radxa-a5x_rpi_release-rknn-sim-profile).

## In the dashboard

When the selected drone has a companion, the main pane shows **Tracker**:
X's own video (`rtsp://<radxa>:8554/tracker`) with its detection boxes and
the track drawn on it. GimbalCam or ChaseCam goes in the small pane beside it
(*View*, or V). Keys 1-9 switch drones, and with them the Tracker shown.

| Input | What X does |
|---|---|
| Click a target | Track it (`CLICK:x,y`) |
| Drag a box around a target | Track that box: `BOX:size`, then `CLICK` at its centre |
| Right-click or Esc | Stop tracking (`CANCEL`) |
| Arrow keys | Move the tracked point 8 px |
| W/A/S/D (hold) | Jog the gimbal; stops on release. Cancels a track first |
| G or C | Centre the gimbal. Cancels a track first |
| Wheel or +/- over the camera | Zoom in or out |

The status panel shows whether each companion's video is arriving and
whether it is tracking. The GimbalCam title shows the gimbal's real angle,
reported by PX4, since X is the one moving it.

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
