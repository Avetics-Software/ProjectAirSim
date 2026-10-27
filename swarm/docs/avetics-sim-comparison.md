# Avetics Sim v2.0.0 vs our ProjectAirSim (cesium-px4-gimbal)

Written 2026-10-27, from a read-only look at `~/Documents/package_avetics_sim_v2.0.0_shipping`.

## Context
`~/Documents/package_avetics_sim_v2.0.0_shipping/sim/Linux/Blocks.sh` launches a **packaged, prebuilt** fork of ProjectAirSim (UE 5.8, Avetics proprietary). We want its useful features in our sim (UE 5.7 + Cesium + PX4 swarm) without losing what we already have.

What the package actually contains:
- **Readable:** the Python and shell scripts in `scripts/` (`sim_profile.py`, `set_*.py`, `simctl.sh`, `launch_aircraft.sh`, `px4_sitl.sh`), the `.parm` files, `models.jsonc`, `SCHEMA.md`, `LAUNCHING.md` and the airframe `.jsonc` files.
- **Not readable:** all the C++. It is compiled into `Blocks-Linux-Shipping`, and the content is cooked into `.pak`.

I found no copy of the Avetics source repo on this machine. So **every C++ feature has to be rewritten, not copied.** The scores below assume that. If you can get the Avetics source, the C++ items each go up by about +1 to +2.

⚠ `models.jsonc` holds a live Cesium ion token in plain text. Do not copy it into our repo.

---

## 1. Features the Avetics build has that ours doesn't (or has only partly)

| # | Feature | How Avetics does it | What we have today |
|---|---|---|---|
| A1 | **One config file for every launch**: model × autopilot × physics × site × world × resolution, with an `--audit` cross-check | `models.jsonc` + `sim_profile.py` (2260 lines of Python) | Command-line flags on `run_cesium_sim.sh` / `px4_gimbal_cesium.py`, plus separate YAML files |
| A2 | **List of named sites** (Changi, Tuas, Marina Bay, Haneda, Zurich, SFO, Las Vegas, …), each with a time zone and its own spawn point | the `sites` block in `models.jsonc` | `--lat/--lon` with Seletar as the default; no named sites |
| A3 | **Time of day with per-site time zone and daylight saving**, set live from a script or an on-screen panel | C++ `UBlocksTimeSubsystem`; `TimeCommand.json` → `TimeState.json`; `set_time.py` | Upstream `set_time_of_day` API exists, but nothing uses it and it is untested on GISMap |
| A4 | **Wind plus gusts** (random gusts set by a strength and a time constant), at launch or live | `Scene::SetWindGust` in `core_sim`; a `wind{}` block in the scene config; `set_wind.py` | Upstream `set_wind_velocity` only; no gusts; no command-line option |
| A5 | **Map imagery switching between Google 3D, Bing and Esri, live** | C++ `UImagerySubsystem` + Cesium raster overlays; `set_imagery.py`; console command `Imagery.Set` | Google 3D Tiles only, chosen in the scene config |
| A6 | **Move the geo origin live** and report each vehicle's position and whether it is resting on the ground | C++ `UGeoOriginSubsystem`; `OriginCommand.json`/`OriginState.json`; `set_origin.py` | `goto LAT LON` reloads the scene and restarts PX4 |
| A7 | **Ground placement**: hold the vehicle until terrain tiles have streamed in, then seat it on the ground | C++ `[GROUNDPLACE]` code in `AUnrealRobot` | `--tile-wait` pauses the sim, and the ground altitude comes from Open-Meteo |
| A8 | **Swap the vehicle in about 1.7 s without restarting Unreal** | `simctl.sh` kills the SITL and camera client, then `/Sim/LoadScene` | Similar idea inside `goto`; no general model swap |
| A9 | **Camera image effects**: lens distortion, vibration (jitter), sensor noise | Python in the camera client (`cv2.remap`/`warpAffine`, frozen with PyInstaller) | Noise from `camera_properties`; no distortion or vibration |
| A10 | **Change camera settings live**: `SetResolution`, `SetAperture`, `SetNoiseParams`, `SetNoiseEnabled` | Robot service methods in the fork's C++ | Not available |
| A11 | **Per-frame pose log** (`PAS_FRAME_TIMES` CSV) | Python | Already covered, and done better, by `experiment_recorder.py` / `gt_capture.py` |
| A12 | **Propeller blur** at speed | C++ plus a `M_PropBlurDisc` material | No |
| A13 | **World picker, front-end menu and loading screen** (AsyncLoadingScreen plugin, MIT licence) | C++ `ABlocksMenuGameMode` + UMG + cooked art | No; the map is chosen with `SIM_MAP` |
| A14 | **More aircraft**: textured Alti VTOL (PX4/ArduPilot), A5 rebuilt from real motor and frame data, quad | Airframe `.jsonc` + `.parm` + cooked meshes + PX4 airframes 70001–70006 | One quad (PX4); VTOL refused; the X8 ArduPlane work is uncommitted in a worktree |
| A15 | **ArduPilot (Copter and Plane) next to PX4**, a parameter store per model, and the airspeed sensor connected over MSP | `ArduCopterApi` changes, `airspeed-msp-port`, `~/.projectairsim/vehicle_params/` | ArduCopter API from upstream only; the swarm is PX4 only |
| A16 | **Unreal (Chaos) physics option** with tuned sub-steps and inertia matching | `physics.unreal` + C++ timing changes | Fast physics only, in practice |
| A17 | **Second world: the authored BBDC map**, with safeguards that refuse impossible site/world combinations | Cooked level + world table | Only GISMap/Cesium |
| A18 | **Gimbal travel and slew limits checked against `MNT1_*` in the autopilot parameters**; gimbal emulation for ArduPilot | C++ `HoldWithinTravel` + `--audit` | We have gimbal `slew`/`travel` settings for PX4; no audit; no ArduPilot gimbal |
| A19 | Cooked content that is present but never wired in: CitySample crowd and traffic (`BP_VehicleAI`, `BP_CrowdCharacter`), `ObjectTrackingSimple` map | Cooked into the `.pak` | — |

## 2. Features we have and Avetics lacks: keep these, and don't let a port break them

Avetics is built around **one vehicle, one client** (an `nng_pair0` pair socket, one actor per scene, `simctl` kills by port). Port features into **our** architecture, not theirs.

- **Moving actors:**
  - `set_object_path` / `PathFollowerComponent` (C++)
  - people and cars on OpenStreetMap roads (`swarm/moving_actors.py`, `road_routes.py`): one-way streets, left-hand driving, convoys, start delays, bridges, cover under trees
  - browser actor editor (`actor_editor.py`)
  - scenario files (`swarm/missions/scenarios/*.yml`)
  - live commands `actors list|pause|resume|reload|delete`
- **Static object spawning** from built-in assets or our own `.glb` files (`spawned_objects.py`), and the **spotter** (a person walked by hand, with a first-person camera).
- **Mission planning:**
  - YAML missions converted to PX4 missions (`swarm/gcs/mission.py`) with steps `takeoff/goto/heading/gimbal/look_at/hold/wait/photo/track/land/rtl`
  - `mission load|upload|start|runs|compare`, with a 5 Hz CSV per run
  - recording and tracking steps (`mission_track.py`)
- **Swarm:**
  - N PX4 SITL instances with per-drone ports (`swarm/px4_sitl.py`)
  - `all takeoff`, goto in line formation, area search (polygon or box) split across drones
  - deconfliction: height layers, crossing check, 6 s predictive separation
- **Safety:** geofence uploaded to every PX4, HOLD ALL / RTL ALL, battery and lost-link responses, health dashboard and minimap.
- **One GCS for both sim and real drones** (`swarm/gcs/`, `run_real.sh`), companion computer integration, and QGC on UDP 14550 with one server per drone.
- **Video and payload:**
  - MAVLink camera with the PX4 gimbal manager v2 and zoom (`payload_mavlink.py`)
  - one RTSP and one WebRTC stream per drone through MediaMTX (`payload_video.py`)
  - optical zoom model (`camera_properties`)
- **Ground truth and evaluation:**
  - `--ground-truth` YOLO labels from segmentation, keeping identity across drones (`gt_capture.py`)
  - actor pose CSV with visible fraction and tree cover
  - `experiment_recorder.py`, `mot_eval.py`, `cmc_eval.py`, `stream_sim.py`, `live_align.py`
- **Cesium extras:** hidden Cesium World Terrain under the trees, tiles rendered as ID 0 in segmentation, the startup tile wait.

## 3. What can be ported, ranked easiest first (5 = easiest, 1 = hardest)

| Rank | Score | Feature | Why that score / how |
|---|---|---|---|
| 1 | **5** | A2 Named sites list | Add a `sites:` block (name, `home-geo-point`, heading, time zone) to `swarm/config.yml` and a `--site` option to `px4_gimbal_cesium.py`. Take the coordinates from `models.jsonc`. Note that Avetics altitudes are ellipsoid heights; we already add the geoid offset. |
| 2 | **5** | A4a Steady wind at launch and live | The C++ is already there (`Scene::SetWindVelocity`, used by fast physics). Add `wind:` to the config and a `sim wind <speed> <from_deg>` command. The direction maths can be read directly in `set_wind.py`. |
| 3 | **4** | A3a Time of day (setting from a script) | Upstream `World.set_time_of_day` / `set_sun_position_from_date_time` plus the SunSky in GISMap. Write our own `set_time` command using each site's time zone. Test first that the sun actually moves on GISMap. |
| 4 | **4** | A9 Lens distortion, vibration and noise on our stream | Pure OpenCV on frames in `payload_video.py`, before ffmpeg. The method is known (`remap` + `warpAffine` + numpy noise). Rewrite it rather than decompiling their code. |
| 5 | **4** | A15b ArduPilot parameter sets and SITL launch pattern | The `.parm` files and `px4_sitl.sh`/`simctl` launch logic are readable. Useful for the X8 ArduPlane worktree and roadmap item #29. A parameter store per model is optional. |
| 6 | **3** | A4b Wind gusts | A small `core_sim` change: a first-order random gust process added to the wind in `Scene` (`gust-std`, `gust-time-constant`, `seed`), plus a scene-config schema key and a client `set_wind_gust`. About 100 lines of C++ plus tests. |
| 7 | **3** | A1 Single launch file + `--audit` | Python only, but it touches all our CLI flags. Do it as a "scenario" YAML that combines site + wind + time + drones + actors + mission and reuses the existing loaders. Don't copy `models.jsonc`; it is single-vehicle. |
| 8 | **3** | A5a Choosing imagery at launch (Google / Bing / Esri) | Small C++ change in `UnrealSimLoader::ConfigureCesiumTileset`: when the scene key `imagery` is `bing` or `esri`, use Cesium World Terrain plus a `UCesiumBingMapsRasterOverlay` or a URL-template overlay. Changing it live is a separate, harder item (below). |
| 9 | **3** | A7 Proper ground placement | C++ in `UnrealRobot`: line-trace down until terrain tiles hit, then seat the vehicle. This replaces the Open-Meteo altitude guess. Moderate work, and touches PX4 startup timing. |
| 10 | **3** | A10 Runtime camera services (`SetResolution`, `SetNoiseParams`, `SetAperture`) | Add robot service methods in `UnrealCamera`. Resolution changes mean reallocating render targets. Useful for testing how zoom and resolution affect tracking. |
| 11 | **2** | A3b / A5b Time and imagery panel inside the sim, imagery switching live | New UMG widgets + subsystems + file or service protocol. Mostly UI work with little value for experiments; scripts cover it. |
| 12 | **2** | A12 Propeller blur | Material + C++ that swaps the blur disc in by rotor speed. Visual only, but it does affect detection of drones by other drones. |
| 13 | **2** | A13 World picker and loading screen | The AsyncLoadingScreen plugin is MIT licensed and can be added freely. The menu game mode, widgets and art have to be built from scratch. Low value for us. |
| 14 | **2** | A19 CitySample crowd and traffic | We can't extract it from their `.pak`. Get CitySample from Fab directly and connect `BP_VehicleAI` to our actor system. Big content, and it competes with our OSM path followers. |
| 15 | **2** | A16 Chaos physics option | Their timing and inertia fixes are C++ we can't see. Lock-step with PX4 is turned off in that mode, which would affect the swarm. |
| 16 | **1** | A14 A5 / Alti VTOL airframes | The meshes are cooked; we'd need the source FBX from Avetics. The airframe `.jsonc` files and PX4 airframes 70004/70006 are readable, so the physics and parameters can be copied once there are meshes. VTOL with PX4 is also missing on our side. |
| 17 | **1** | A15a Airspeed over MSP for ArduPilot | Only needed for their ArduPilot JSON-less path. Our X8 bridge uses ArduPilot's JSON SITL, which can carry airspeed directly, so **skip it**. |
| 18 | **1** | A17 Authored BBDC world | Cooked level; needs the source `.umap` and assets (perhaps from `~/Documents/package_bbdc_multiple_vehicles_5.7`?). |
| — | n/a | A11 pose log, A18 gimbal limits | We already have something equal or better. |
| — | n/a | UE 5.8 upgrade | Out of scope. |

## Suggested order (future work)
1. **Environment pack, in Python only (ranks 1–4):**
   - `sites:` + `wind:` + `time:` in `swarm/config.yml`
   - options `--site`, `--wind`, `--time`
   - live commands `sim wind …` and `sim time …` in `px4_gimbal_cesium.py`, using the `World` methods in `client/python/projectairsim/src/projectairsim/world.py`
   - image effects in `swarm/payload_video.py`
2. **Gusts in `core_sim/src/scene.cpp`** + schema + client method + gtest (rank 6).
3. **Scenario YAML** that combines site/env/actors/mission (rank 7). Extend the mission file's existing `actors:` key rather than adding a new format.
4. **Imagery at launch** (rank 8) and **ground placement** (rank 9) in `UnrealSimLoader.cpp` / `UnrealRobot.cpp`.

Files most affected: `swarm/px4_gimbal_cesium.py`, `swarm/config.yml`, `swarm/payload_video.py`, `swarm/run_cesium_sim.sh`, `core_sim/src/scene.cpp`, `unreal/Blocks/Plugins/ProjectAirSim/Source/ProjectAirSim/Private/UnrealSimLoader.cpp`, `swarm/docs/roadmap.md`.

## Verification
- Wind: run a mission with `--wind 8 270` and confirm PX4 shows drift and roll in its estimator and log.
- Gusts: a gtest on the gust statistics (standard deviation and time constant), plus the same flight check as wind.
- Time: `sim time 18:30` at two sites and confirm the sun angle in the GimbalCam screenshot.
- Image effects: record with `experiment_recorder.py` and compare MOT ID switches with and without them (`mot_eval.py`).
- No regressions: the existing `mot_convoy.yml` mission and `--drones 3` area search still run, and ground truth still lines up.
