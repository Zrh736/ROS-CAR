"""Grid navigation and a delivery mission state machine for desktop simulation.

This module has no ROS or chassis connection. One grid cell represents 0.5 m.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from itertools import permutations

WIDTH = 25
HEIGHT = 17
CELL_METERS = 0.5
HOME = (2, 2)
STATIONS = {"A": (21, 2), "B": (21, 14), "C": (3, 14)}


def default_obstacles() -> set[tuple[int, int]]:
    walls = {(8, y) for y in range(1, 14) if y not in (4, 10)}
    walls |= {(16, y) for y in range(3, 16) if y not in (7, 13)}
    walls |= {(x, 8) for x in range(10, 15) if x != 12}
    return walls


def shortest_path(start, goal, obstacles):
    """Return the shortest four-neighbour path, or None if blocked."""
    if start in obstacles or goal in obstacles:
        return None
    queue = deque([start])
    previous = {start: None}
    while queue:
        point = queue.popleft()
        if point == goal:
            path = []
            while point is not None:
                path.append(point)
                point = previous[point]
            return list(reversed(path))
        x, y = point
        for neighbour in ((x + 1, y), (x, y + 1), (x - 1, y), (x, y - 1)):
            nx, ny = neighbour
            if (0 <= nx < WIDTH and 0 <= ny < HEIGHT and
                    neighbour not in obstacles and neighbour not in previous):
                previous[neighbour] = point
                queue.append(neighbour)
    return None


@dataclass
class RouteMemory:
    """Reuse previously found grid routes after checking them against new walls.

    This is a small teaching analogue of motion memory, not the MGMM model.
    """

    routes: dict[tuple[tuple[int, int], tuple[int, int]], tuple[tuple[int, int], ...]] = field(default_factory=dict)
    hits: int = 0
    searches: int = 0
    repairs: int = 0

    def path(self, start, goal, obstacles):
        key = (start, goal)
        cached = self.routes.get(key)
        if cached is not None and not any(point in obstacles for point in cached):
            self.hits += 1
            return list(cached)
        if cached is not None:
            self.repairs += 1
        self.searches += 1
        path = shortest_path(start, goal, obstacles)
        if path is not None:
            self.routes[key] = tuple(path)
            self.routes[(goal, start)] = tuple(reversed(path))
        return path

    def clear_after_removal(self):
        # Removing a wall may open a shorter route than a previously safe one.
        self.routes.clear()


def optimize_visit_order(start, stations, obstacles, memory):
    """Find the shortest complete visit order using remembered pairwise paths."""
    best = None
    best_cost = float("inf")
    for candidate in permutations(sorted(stations)):
        points = [start] + [STATIONS[name] for name in candidate] + [HOME]
        cost = 0
        for source, destination in zip(points, points[1:]):
            path = memory.path(source, destination, obstacles)
            if path is None:
                cost = float("inf")
                break
            cost += len(path) - 1
        if cost < best_cost:
            best = list(candidate)
            best_cost = cost
    return best


@dataclass
class DeliveryMission:
    obstacles: set[tuple[int, int]] = field(default_factory=default_obstacles)
    position: tuple[int, int] = HOME
    state: str = "IDLE"
    order: list[str] = field(default_factory=list)
    visited: list[str] = field(default_factory=list)
    target: str | None = None
    path: list[tuple[int, int]] = field(default_factory=list)
    trajectory: list[tuple[int, int]] = field(default_factory=lambda: [HOME])
    distance_cells: int = 0
    elapsed_seconds: int = 0
    replans: int = 0
    reason: str = ""
    events: list[str] = field(default_factory=list)
    memory: RouteMemory = field(default_factory=RouteMemory)
    optimize: bool = False
    requested_order: list[str] = field(default_factory=list)
    path_searches: int = 0
    memory_start_hits: int = 0
    memory_start_searches: int = 0
    memory_start_repairs: int = 0

    def _event(self, message):
        self.events.append(f"{self.elapsed_seconds:03d}s  {message}")

    def _plan(self):
        goal = HOME if self.target == "HOME" else STATIONS[self.target]
        if self.optimize:
            path = self.memory.path(self.position, goal, self.obstacles)
        else:
            self.path_searches += 1
            path = shortest_path(self.position, goal, self.obstacles)
        if path is None:
            self.state = "FAILED"
            self.reason = f"前往 {self.target} 的道路被堵"
            self.path = []
            self._event(self.reason + "，任务停止")
            return
        self.path = path
        self._event(f"规划前往 {self.target}：{len(path) - 1} 格")

    def start(self, order, optimize=False):
        if self.state not in ("IDLE", "DONE", "FAILED", "CANCELED"):
            raise ValueError("请先结束当前任务")
        order = [item.strip().upper() for item in order]
        if not order or len(set(order)) != len(order) or any(item not in STATIONS for item in order):
            raise ValueError("请输入不重复的配送点，例如 A,B,C")
        self.position = HOME
        self.state = "MOVING"
        self.requested_order = order.copy()
        self.optimize = optimize
        self.visited = []
        self.target = None
        self.path = []
        self.trajectory = [HOME]
        self.distance_cells = self.elapsed_seconds = self.replans = 0
        self.reason = ""
        self.events = []
        self.memory_start_hits = self.memory.hits
        self.memory_start_searches = self.memory.searches
        self.memory_start_repairs = self.memory.repairs
        self.path_searches = 0
        self.order = (optimize_visit_order(HOME, order, self.obstacles, self.memory)
                      if optimize else order)
        if self.order is None:
            self.order = order
            self.state = "FAILED"
            self.reason = "存在无法到达的配送点"
            self._event(self.reason + "，任务停止")
            return
        self.target = self.order[0]
        self._event("接单：" + " → ".join(order) + " → HOME")
        if optimize:
            self._event("路径记忆建议顺序：" + " → ".join(self.order) + " → HOME")
        self._plan()

    def tick(self):
        if self.state != "MOVING":
            return
        self.elapsed_seconds += 1
        if len(self.path) < 2 or self.path[0] != self.position or self.path[1] in self.obstacles:
            self.replans += 1
            self._event("路径变化，重新规划")
            self._plan()
            if self.state == "FAILED":
                return
        self.position = self.path[1]
        self.trajectory.append(self.position)
        self.path = self.path[1:]
        self.distance_cells += 1
        if len(self.path) == 1:
            if self.target == "HOME":
                self.state = "DONE"
                self._event("返回装载点，任务完成")
            else:
                self.state = "WAITING"
                self._event(f"到达 {self.target}，等待取货确认")

    def confirm(self):
        if self.state != "WAITING":
            raise ValueError("当前没有等待确认的配送点")
        self.visited.append(self.target)
        self._event(f"{self.target} 已确认取货")
        if self.optimize and len(self.visited) < len(self.order):
            remaining = self.order[len(self.visited):]
            revised = optimize_visit_order(self.position, remaining, self.obstacles, self.memory)
            if revised is None:
                self.state = "FAILED"
                self.reason = "剩余配送点无法全部到达"
                self.path = []
                self._event(self.reason + "，任务停止")
                return
            if revised != remaining:
                self.order = self.visited + revised
                self._event("剩余站点调整为：" + " → ".join(revised))
        self.target = (self.order[len(self.visited)]
                       if len(self.visited) < len(self.order) else "HOME")
        self.state = "MOVING"
        self._plan()

    def cancel(self):
        if self.state not in ("MOVING", "WAITING"):
            return
        self.state = "CANCELED"
        self.path = []
        self.reason = "人工取消"
        self._event("任务取消，小车停止")

    def toggle_obstacle(self, point):
        if point == self.position or point == HOME or point in STATIONS.values():
            return False
        x, y = point
        if not (0 <= x < WIDTH and 0 <= y < HEIGHT):
            return False
        if point in self.obstacles:
            self.obstacles.remove(point)
            self.memory.clear_after_removal()
            self._event(f"移除障碍 {point}")
        else:
            self.obstacles.add(point)
            self._event(f"放置障碍 {point}")
        return True

    def result(self):
        return {
            "state": self.state, "requested_order": self.requested_order,
            "order": self.order, "visited": self.visited,
            "position": self.position, "distance_m": self.distance_cells * CELL_METERS,
            "trajectory": self.trajectory,
            "elapsed_s": self.elapsed_seconds, "replans": self.replans,
            "planner": "memory_order" if self.optimize else "fixed_order",
            "path_searches": (self.memory.searches - self.memory_start_searches
                              if self.optimize else self.path_searches),
            "memory_hits": self.memory.hits - self.memory_start_hits if self.optimize else 0,
            "memory_repairs": self.memory.repairs - self.memory_start_repairs if self.optimize else 0,
            "reason": self.reason, "events": self.events,
        }


def run_batch(count=10):
    """Run reproducible normal, dynamic-block, and unreachable-goal scenarios."""
    if count < 1:
        raise ValueError("count must be positive")
    orders = list(permutations(STATIONS))
    results = []
    for number in range(count):
        mission = DeliveryMission()
        scenario = ("station_blocked" if number == 9 else
                    "dynamic_block" if number in (2, 5, 8) else "normal")
        if scenario == "station_blocked":
            x, y = STATIONS["B"]
            mission.obstacles.update({(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)})
        mission.start(orders[number % len(orders)])
        steps = 0
        injected = False
        while mission.state in ("MOVING", "WAITING") and steps < 500:
            if mission.state == "WAITING":
                mission.confirm()
            else:
                if scenario == "dynamic_block" and not injected and steps >= 7 and len(mission.path) > 2:
                    injected = mission.toggle_obstacle(mission.path[1])
                mission.tick()
            steps += 1
        if steps >= 500:
            mission.state = "FAILED"
            mission.reason = "仿真步数超限"
        results.append({"run": number + 1, "scenario": scenario, **mission.result()})
    return results


def run_comparison(count=10):
    """Compare fixed and memory-ordered tours on the same static grid."""
    if count < 1:
        raise ValueError("count must be positive")
    orders = list(permutations(STATIONS))
    shared_memory = RouteMemory()
    rows = []
    for index in range(count):
        requested = orders[index % len(orders)]
        pair = {}
        for label, optimized in (("fixed", False), ("memory_order", True)):
            mission = DeliveryMission(memory=shared_memory if optimized else RouteMemory())
            mission.start(requested, optimize=optimized)
            steps = 0
            while mission.state in ("MOVING", "WAITING") and steps < 500:
                if mission.state == "WAITING":
                    mission.confirm()
                else:
                    mission.tick()
                steps += 1
            pair[label] = mission.result()
        rows.append({"run": index + 1, "requested_order": requested, **pair})
    return rows
