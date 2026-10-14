"""Run with: python -m unittest delivery_sim.test_core"""

import unittest

from .core import DeliveryMission, STATIONS, run_batch, shortest_path


class DeliverySimulationTests(unittest.TestCase):
    def test_three_stations_wait_for_confirmation_and_return_home(self):
        mission = DeliveryMission()
        mission.start(["A", "B", "C"])
        waiting = []
        for _ in range(500):
            if mission.state == "WAITING":
                waiting.append(mission.target)
                mission.confirm()
            elif mission.state == "MOVING":
                mission.tick()
            else:
                break
        self.assertEqual(waiting, ["A", "B", "C"])
        self.assertEqual(mission.state, "DONE")
        self.assertEqual(mission.position, (2, 2))
        self.assertEqual(mission.visited, ["A", "B", "C"])
        self.assertGreater(mission.distance_cells, 0)

    def test_dynamic_obstacle_triggers_replan(self):
        mission = DeliveryMission()
        mission.start(["A"])
        blocked = mission.path[2]
        mission.toggle_obstacle(blocked)
        mission.tick()
        self.assertEqual(mission.state, "MOVING")
        self.assertEqual(mission.replans, 0)
        mission.tick()
        self.assertEqual(mission.replans, 1)
        self.assertNotEqual(mission.position, blocked)

    def test_blocked_station_fails_without_entering_it(self):
        mission = DeliveryMission()
        x, y = STATIONS["B"]
        mission.obstacles.update({(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)})
        mission.start(["B"])
        self.assertEqual(mission.state, "FAILED")
        self.assertEqual(mission.distance_cells, 0)
        self.assertIn("道路被堵", mission.reason)

    def test_cancel_stops_progress(self):
        mission = DeliveryMission()
        mission.start(["A"])
        mission.tick()
        position = mission.position
        mission.cancel()
        mission.tick()
        self.assertEqual(mission.state, "CANCELED")
        self.assertEqual(mission.position, position)

    def test_batch_ten_missions_record_success_and_failure(self):
        results = run_batch(10)
        self.assertEqual(len(results), 10)
        self.assertEqual(sum(item["state"] == "DONE" for item in results), 9)
        self.assertEqual(results[-1]["state"], "FAILED")
        self.assertIn("道路被堵", results[-1]["reason"])
        self.assertTrue(any(item["replans"] > 0 for item in results))
        self.assertTrue(all(item["trajectory"] for item in results))

    def test_shortest_path_rejects_blocked_goal(self):
        self.assertIsNone(shortest_path((0, 0), (1, 1), {(1, 1)}))


if __name__ == "__main__":
    unittest.main()
