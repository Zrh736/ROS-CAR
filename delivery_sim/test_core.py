"""Run with: python -m unittest delivery_sim.test_core"""

import unittest

from .core import (MAP_NAMES, DeliveryMission, RouteMemory, STATIONS,
                   compare_algorithms, obstacles_for_map, run_batch,
                   shortest_path)


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

    def test_memory_reuses_safe_path_and_repairs_blocked_path(self):
        memory = RouteMemory()
        walls = set()
        original = memory.path((0, 0), (4, 0), walls)
        self.assertEqual(memory.searches, 1)
        self.assertEqual(memory.path((0, 0), (4, 0), walls), original)
        self.assertEqual(memory.hits, 1)
        walls.add(original[1])
        repaired = memory.path((0, 0), (4, 0), walls)
        self.assertNotIn(original[1], repaired)
        self.assertEqual(memory.repairs, 1)
        self.assertEqual(memory.searches, 2)

    def test_auto_order_compares_nearest_neighbor_with_memory(self):
        rows = compare_algorithms(["A", "B", "D"], obstacles_for_map(MAP_NAMES[1]), True, 10)
        self.assertEqual(len(rows), 10)
        self.assertTrue(all(row["baseline"]["state"] == "DONE" for row in rows))
        self.assertTrue(all(row["memory"]["state"] == "DONE" for row in rows))
        self.assertLess(rows[0]["memory"]["distance_m"], rows[0]["baseline"]["distance_m"])
        self.assertEqual(rows[0]["baseline"]["ordering"], "auto")
        self.assertLess(sum(row["memory"]["path_searches"] for row in rows),
                        sum(row["baseline"]["path_searches"] for row in rows))
        self.assertEqual(rows[1]["memory"]["path_searches"], 0)
        self.assertGreater(rows[1]["memory"]["memory_hits"], 0)

    def test_optimized_mode_reports_unreachable_station(self):
        mission = DeliveryMission()
        x, y = STATIONS["B"]
        mission.obstacles.update({(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)})
        mission.start(["A", "B", "C"], optimize=True)
        self.assertEqual(mission.state, "FAILED")
        self.assertIsNone(mission.target)
        self.assertEqual(mission.visited, [])

    def test_current_comparison_uses_order_and_map(self):
        walls = obstacles_for_map(MAP_NAMES[0])
        first = compare_algorithms(["A", "B", "C"], walls, False)[0]
        changed = compare_algorithms(["A", "C", "B"], walls, False)[0]
        for algorithm in ("baseline", "memory"):
            self.assertEqual(first[algorithm]["order"], ["A", "B", "C"])
            self.assertEqual(changed[algorithm]["order"], ["A", "C", "B"])
            self.assertNotEqual(first[algorithm]["distance_m"], changed[algorithm]["distance_m"])
            self.assertEqual(first[algorithm]["ordering"], "fixed")
        blocked = set(walls)
        x, y = STATIONS["D"]
        blocked.update({(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)})
        self.assertEqual(compare_algorithms(["D"], blocked, False)[0]["baseline"]["state"], "FAILED")

    def test_each_example_map_supports_all_five_delivery_points(self):
        self.assertEqual(len(STATIONS), 5)
        for name in MAP_NAMES:
            result = compare_algorithms(list(STATIONS), obstacles_for_map(name), True)[0]
            self.assertEqual(result["baseline"]["state"], "DONE", name)
            self.assertEqual(result["memory"]["state"], "DONE", name)


if __name__ == "__main__":
    unittest.main()
