"""
Copyright (C) 2025 IAMAI CONSULTING CORP
MIT License.

Demonstrates flying the JSBSim Skywalker X8 with an ArduPlane SITL controller.

Note: ArduPlane SITL must run with ArduPilot's JSON backend and the X8 parameters:

      cd ardupilot/Tools/autotest
      python sim_vehicle.py -v ArduPlane --model JSON \
          --home=29.4296741,-95.16384722,7.5,0 \
          --add-param-file=<path>/x8-arduplane.param

      The X8 waits on a virtual catapult until ArduPlane raises throttle. From the
      MAVProxy console (or a GCS such as QGroundControl or Mission Planner):

          mode TAKEOFF
          arm throttle

      After the climb, switch to LOITER, RTL or AUTO. Project AirSim Client API
      flight commands are not supported with arduplane-api.
"""

from projectairsim import ProjectAirSimClient, Drone, World
from projectairsim.utils import projectairsim_log
from projectairsim.image_utils import ImageDisplay


def main():
    # Create a Project AirSim client
    client = ProjectAirSimClient()

    # Initialize an ImageDisplay object to position the chase camera window
    image_display = ImageDisplay()

    try:
        # Connect to simulation environment
        client.connect()

        # Create a World object to interact with the sim world and load a scene
        world = World(client, "scene_ardu_x8_plane.jsonc", delay_after_load_sec=2)

        # Create a Drone object to interact with the X8 in the loaded sim world
        drone = Drone(client, world, "X8")

        # ------------------------------------------------------------------------------
        # Subscribe to chase camera sensor
        chase_cam_window = "ChaseCam"
        image_display.add_chase_cam(chase_cam_window)
        client.subscribe(
            drone.sensors["Chase"]["scene_camera"],
            lambda _, chase: image_display.receive(chase, chase_cam_window),
        )

        image_display.start()

        # ------------------------------------------------------------------------------
        # ArduPlane is controlled externally. Wait for a user input to stop.
        projectairsim_log().info(
            "X8 is held on the catapult. In MAVProxy: 'mode TAKEOFF' then 'arm throttle'."
        )
        input("Press Enter to stop viewing the X8's chase camera...")

        # ------------------------------------------------------------------------------

    except Exception as err:
        projectairsim_log().error(f"Exception occurred: {err}", exc_info=True)

    finally:
        # Always disconnect from the simulation environment to allow next connection
        client.disconnect()

        image_display.stop()


if __name__ == "__main__":
    main()
