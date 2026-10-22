"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Moved to gcs/road_routes.py, which the ground station's map shares; this keeps
`from road_routes import ...` and `python road_routes.py ...` working.
"""

from gcs.road_routes import *  # noqa: F401,F403
from gcs.road_routes import RoadNetwork, main, path_length_m  # noqa: F401

if __name__ == "__main__":
    main()
