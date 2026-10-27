"""The disarm command: refused in flight, sent once landed: python -m unittest discover -s tests."""

import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from pymavlink.dialects.v20 import common as M

from gcs.airframe import Airframe
from gcs.autopilot import dialect
from gcs.swarm import SwarmControl


class DisarmTests(unittest.IsolatedAsyncioTestCase):
    def make(self, height, speed=0.0):
        link = SimpleNamespace(name="Drone1", sysid=1, position=(1, 103, 50, height),
                               velocity=(speed, 0.0, 0.0), armed=True, mode="HOLD",
                               mode_name="HOLD", airframe=Airframe(), ap=dialect("px4"))
        link.disarm = Mock(return_value=M.MAV_RESULT_ACCEPTED)
        return link, SwarmControl([link])

    async def test_refused_in_the_air(self):
        link, swarm = self.make(height=40)
        with self.assertRaises(RuntimeError):
            await swarm._disarm(0, False)
        link.disarm.assert_not_called()

    async def test_refused_while_still_rolling(self):
        link, swarm = self.make(height=0.2, speed=12)
        with self.assertRaises(RuntimeError):
            await swarm._disarm(0, False)

    async def test_sent_once_landed(self):
        link, swarm = self.make(height=0.3)
        await swarm._disarm(0, False)
        link.disarm.assert_called_once_with(False)

    async def test_force_drops_it(self):
        link, swarm = self.make(height=40)
        await swarm._disarm(0, True)
        link.disarm.assert_called_once_with(True)


if __name__ == "__main__":
    unittest.main()
