"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

The ground station's logger. Whoever runs the ground station attaches the
handlers: the sim launcher reuses Project AirSim's log file and terminal, the
standalone ground station (gcs/__main__.py) sets up its own.
"""

import logging

log = logging.getLogger("gcs")
