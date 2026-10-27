# ArduPilot Flight Controller

[ArduPilot](https://ardupilot.org/) is a flexible open-source autopilot that can run with Project AirSim through Software-In-The-Loop (SITL). The repository includes quadrotor and hexarotor examples in [`client/python/example_user_scripts/ardupilot`](../../client/python/example_user_scripts/ardupilot). Multirotors use the `ardupilot-api` controller; fixed-wing aircraft use `arduplane-api`, described in [ArduPlane fixed-wing aircraft](#arduplane-fixed-wing-aircraft); the swarm simulation's Skywalker X8 uses it.

**Compatibility note:** this integration has not been recently validated against the latest ArduPilot release. Start with the supplied examples and their parameter files before adapting it to a different vehicle.

## Supported airframes and control

Project AirSim includes these ArduPilot SITL examples:

| Airframe | Scene configuration | Client script | Parameter file |
| --- | --- | --- | --- |
| Quadrotor | `sim_config/scene_ardu_quadrotor.jsonc` | `ardupilot_quadrotor.py` | `project-airsim-quad.param` |
| Hexarotor | `sim_config/scene_ardu_hexarotor.jsonc` | `ardupilot_hexarotor.py` | `project-airsim-hexa.param` |

The examples display camera streams and load the scene; Project AirSim Client API flight commands such as arm, takeoff, and move are not supported with `ardupilot-api`. Control the vehicle from the ArduPilot console or through a ground control station (GCS), for example Mission Planner or QGroundControl.

## ArduPilot settings

Set the robot controller type to `ardupilot-api` and configure `ardupilot-settings`:

```json
"controller": {
  "id": "ArduPilot_Controller",
  "airframe-setup": "hexarotor-x",
  "type": "ardupilot-api",
  "ardupilot-settings": {
    "local-host-ip": "127.0.0.1",
    "ardupilot-ip": "127.0.0.1",
    "ardupilot-udp-port": 9003,
    "local-host-udp-port": 9002,
    "use-distance-sensor": true,
    "actuator-order": [
      { "id": "Prop_2_actuator" }
    ]
  }
}
```

| Parameter | Type | Description |
| --- | --- | --- |
| `local-host-ip` | string | IP address of the Project AirSim host. |
| `ardupilot-ip` | string | IP address of the ArduPilot host. |
| `ardupilot-udp-port` | integer | UDP port on the ArduPilot host. The examples use `9003`. |
| `local-host-udp-port` | integer | UDP port on the Project AirSim host. The examples use `9002`. |
| `use-distance-sensor` | boolean | Sends distance-sensor data to ArduPilot when enabled. |
| `actuator-order` | array | Ordered actuator identifiers connected to the ArduPilot outputs. |

ArduPilot and Project AirSim exchange simulator messages, including sensor and actuator data, over UDP/IP. The Project AirSim receive port is `local-host-udp-port`.

### Network configurations

| Configuration | `ardupilot-ip` | `local-host-ip` |
| --- | --- | --- |
| Same Linux computer | `127.0.0.1` | `127.0.0.1` |
| Separate computers | ArduPilot host IP | Project AirSim host IP |
| ArduPilot in WSL1 | `127.0.0.1` | `127.0.0.1` |
| ArduPilot in WSL2 | WSL virtual adapter IP | WSL virtual adapter IP |

For WSL2, run `ipconfig` on Windows and use the IPv4 address of the WSL virtual network adapter in both settings. When using separate hosts or WSL, configure firewalls to allow the required UDP traffic. A GCS on another host also needs outgoing access from the Project AirSim host and incoming access on its configured UDP port.

## Install and run ArduPilot SITL

Complete the Project AirSim installation and [client setup](../client_setup.md), then prepare ArduPilot on a Linux host, VM, WSL1, or WSL2. The standard ArduPilot prerequisites may change, so consult the [ArduPilot build documentation](https://ardupilot.org/dev/docs/building-setup-linux.html) if the setup script reports an error.

```bash
git clone --recursive https://github.com/ArduPilot/ardupilot.git
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

Copy the matching parameter file from the Project AirSim examples directory into ArduPilot's `Tools/autotest` directory. For a hexarotor, copy `project-airsim-hexa.param`; for a quadrotor, copy `project-airsim-quad.param`.

Start the hexarotor SITL instance from the ArduPilot repository:

```bash
cd Tools/autotest
python sim_vehicle.py -v ArduCopter -f airsim-copter --add-param-file=project-airsim-hexa.param
```

When ArduPilot runs in WSL2 or on a different computer, append `--sim-address=<project-airsim-host-ip>` to that command.

### WSL1 display setup

If you use an X Windows application in WSL1, install and start an X server such as VcXsrv. In XLaunch, select **Multiple windows**, then **Start no client**, and enable **Clipboard** without Native OpenGL. Set `DISPLAY` in WSL; the following persistent configuration uses the Windows-side nameserver address:

```bash
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
```

If the X client cannot connect, enable **Disable access control** in the X server only when appropriate for your environment.

## Run an example flight session

1. Start ArduPilot SITL with the matching parameter file.
2. In a terminal with the Project AirSim Python client environment activated, change to `client/python/example_user_scripts/ardupilot`.
3. Run `python ardupilot_hexarotor.py` or `python ardupilot_quadrotor.py`.
4. Control the vehicle from the ArduPilot console or a GCS.

For a local Linux host or WSL1, the checked-in configurations already use loopback addresses. For WSL2 or separate machines, update `local-host-ip` and `ardupilot-ip` in the matching `sim_config/robot_ardu_*.jsonc` file before running the example.

Always stop Project AirSim before stopping ArduPilot. After Project AirSim stops, restart ArduPilot before beginning another session.

### Useful ArduPilot console commands

| Command | Notes |
| --- | --- |
| `arm throttle` | Arms the vehicle. |
| `mode guided` | Wait until the console reports that EKF3 is using GPS before switching to Guided mode. |
| `takeoff <alt>` | Takes off to the requested altitude in metres. Wait for completion before the next command. |
| `guided <lat> <lon> <alt>` | Flies to latitude/longitude in degrees and altitude in metres. Wait for the waypoint to be reached. |
| `mode land` | Lands the vehicle. |

If `AP: Arm: Main loop slow...` appears, try `arm throttle` again. If it persists, review the simulation clock settings such as `step-ns`.

## ArduPlane fixed-wing aircraft

The `arduplane-api` controller connects ArduPlane SITL to a robot that uses [JSBSim physics](../physics/jsbsim.md). It speaks ArduPilot's [JSON SITL interface](https://github.com/ArduPilot/ardupilot/tree/master/libraries/SITL/examples/JSON). ArduPilot sends one servo packet per physics frame, and Project AirSim answers it with the vehicle state from the same tick. This keeps the two simulators in lockstep, so ArduPilot never runs ahead of the physics.

The state sent to ArduPilot is ground truth: position and velocity in NED from the scene's `home-geo-point`, attitude, body rates, the body-frame accelerometer reading, and airspeed. ArduPilot's SITL adds its own sensor models and noise on top of that.

### ArduPlane settings

```json
"controller": {
  "id": "ArduPlane_Controller",
  "airframe-setup": "fixed-wing",
  "type": "arduplane-api",
  "arduplane-settings": {
    "local-host-ip": "0.0.0.0",
    "local-host-udp-port": 9002,
    "frame-timeout-ms": 1000,
    "send-airspeed": true,
    "launch": { "hold": true, "release-throttle": 0.5, "speed": 18.0, "duration": 0.5 },
    "actuator-order": [
      { "id": "Aileron_actuator" },
      { "id": "Elevator_actuator", "scale": -1.0 },
      { "id": "Throttle_actuator" }
    ]
  }
}
```

| Parameter | Type | Description |
| --- | --- | --- |
| `local-host-ip` | string | Address to listen on for ArduPilot's servo packets. Default `0.0.0.0`. |
| `local-host-udp-port` | integer | UDP port ArduPilot sends to: `9002 + 10 * instance` for `sim_vehicle.py -I <instance>`. Default `9002`. Replies go back to the address the packets came from. |
| `frame-timeout-ms` | integer | Once ArduPilot is connected, how long each tick waits for its next servo packet before treating it as disconnected. Default `1000`. |
| `wait-for-ardupilot` | boolean | Also wait for ArduPilot before it first connects, so the scene does not advance without it. Default `false`. |
| `send-airspeed` | boolean | Send the airspeed along the body x axis, relative to the scene's wind. Default `true`. |
| `launch` | object | Optional catapult launch for aircraft that spawn airborne; see below. |
| `actuator-order` | array | One entry per ArduPilot servo output; the first entry is `SERVO1`. `scale` multiplies the normalized output. `range` is `unipolar` (1000–2000 µs to 0–1, the default for rotor actuators) or `bipolar` (1000–2000 µs to −1–1 around 1500 µs, the default for everything else). |
| `gimbal` | object | Optional gimbal on ArduPilot's mount outputs. `id` names the robot's `gimbal` actuator. `roll-channel`, `pitch-channel` and `yaw-channel` are the `SERVOn` outputs. `<axis>-min-deg` and `<axis>-max-deg` give each axis's travel. Set `MNT1_TYPE 7` (BrushlessPWM, a self-stabilising gimbal): ArduPilot then sends roll and pitch relative to the horizon and yaw relative to the nose, which is what the gimbal actuator takes. Each output must span `SERVOn_MIN 1000` to `SERVOn_MAX 2000` across exactly that travel, which must equal `MNT1_<AXIS>_MIN/MAX`. ArduPilot is then the gimbal manager, so ground stations steer it with the MAVLink gimbal protocol. |

Actuators forward the normalized outputs to JSBSim through their `jsbsim-cmd` property. In ArduPilot, a high PWM means nose up or roll right. In JSBSim models, positive `fcs/elevator-cmd-norm` usually means nose down, so the elevator needs `"scale": -1.0`.

#### Catapult launch

An aircraft without landing gear can't start on the ground. With `launch.hold`, the controller freezes the JSBSim model at its spawn pose (`FGFDMExec::Hold`) and reports a stationary vehicle to ArduPilot. That gives ArduPilot time to boot, align its EKF and arm. When a throttle output (any `unipolar` entry) reaches `release-throttle`, the hold is released. JSBSim then accelerates the aircraft along its body x axis to `speed` (m/s) over `duration` (s), and the push shows up in the reported accelerometer reading. A `speed` of `0` releases the hold without the push.

### The Skywalker X8

The Skywalker X8 (JSBSim) flown by ArduPlane is part of the swarm simulation: the `x8_arduplane` aircraft model in [`swarm/aircraft/`](../../swarm/aircraft/). Its robot config is `swarm/aircraft/airframes/x8.jsonc` and its ArduPlane parameters `swarm/aircraft/params/x8_arduplane.param`; the swarm launcher starts ArduPlane SITL itself. See `swarm/docs/features/aircraft.md` for how to fly it.

The parameters map `SERVO1`–`SERVO3` to aileron, elevator and throttle. The X8 JSBSim model mixes those commands onto its elevons itself, so ArduPlane flies it as a conventional airframe without a rudder. Attitude control uses ArduPlane's default SITL gains. On this model they hold a loiter bank to within 0.2° RMS and reach 90% of a roll or pitch stick step in about 1 s or less. Two `AUTOTUNE` passes did not improve them overall, so the parameter file adds no tuned gains. Re-tune if you change the airframe model.

## Autotune a different vehicle

Vehicles other than the supplied examples should be autotuned for stable flight. Autotune can take hours.

1. Run `arm throttle`.
2. Run `rc3 1800` and wait until the vehicle is clear of obstacles.
3. Run `rc3 1500`.
4. Run `mode autotune`.

ArduPilot reports the tuned roll, pitch, and yaw values in its console. Copy them to the corresponding `ATC_RAT_*`, `ATC_ANG_*`, and `ATC_ACCEL_*` entries in a `.param` file, then start SITL with `--add-param-file=<file>`. The supplied `project-airsim-hexa.param` and `project-airsim-quad.param` files are reference examples.

It is safe to ignore an `AP: Auto-Tune: failing to level` message if it appears during the procedure.

---

Copyright (C) Microsoft Corporation.  
Copyright (C) 2025 IAMAI CONSULTING CORP

MIT License. All rights reserved.
