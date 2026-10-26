"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Swarm ground station: the dashboard, swarm control and deconfliction, and the
link to each drone's X vision pipeline. It talks to the drones over MAVLink
only, so the same code flies the ProjectAirSim swarm (px4_gimbal_cesium.py) and
real PX4 drones (python -m gcs --fleet fleet.yml).

Nothing in this package imports projectairsim: copy the folder to a field
laptop and install requirements.txt.
"""
