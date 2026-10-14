"""Grid navigation and a delivery mission state machine for desktop simulation.

This module has no ROS or chassis connection. One grid cell represents 0.5 m.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from itertools import permutations
import random

WIDTH = 45
HEIGHT = 33
CELL_METERS = 0.5
HOME = (2, 2)
STATIONS = {"A": (41, 2), "B": (41, 29), "C": (3, 29),
            "D": (22, 16), "E": (22, 30)}
MAP_NAMES = ("经典障碍", "仓库通道", "空旷地图", "迷宫地图", "环形地图")


def default_obstacles() -> set[tuple[int, int]]:
    walls = {(12, y) for y in range(1, 32) if y not in (7, 8, 23, 24)}
    walls |= {(32, y) for y in range(1, 32) if y not in (10, 11, 26, 27)}
    walls |= {(x, 16) for x in range(13, 32) if x not in (21, 22, 23)}
    walls |= {(22, y) for y in range(4, 15) if y not in (8, 9)}
    return walls


def _rectangular_ring(left, top, right, bottom, gates):
    walls = {(x, top) for x in range(left, right + 1)}
    walls |= {(x, bottom) for x in range(left, right + 1)}
    walls |= {(left, y) for y in range(top, bottom + 1)}
    walls |= {(right, y) for y in range(top, bottom + 1)}
    return walls - gates


def obstacles_for_map(name: str) -> set[tuple[int, int]]:
    """Return a fresh obstacle set for a built-in example map."""
    if name == MAP_NAMES[0]:
        return default_obstacles()
    if name == MAP_NAMES[1]:
        return {(x, y) for y in (8, 16, 24) for x in range(2, 43)
                if x not in (6, 7, 21, 22, 23, 38, 39)}
    if name == MAP_NAMES[2]:
        return set()
    if name == MAP_NAMES[3]:
        gaps = {9: (4, 5, 6, 7), 18: (24, 25, 26, 27),
                27: (5, 6, 7, 8), 36: (23, 24, 25, 26)}
        walls = {(x, y) for x, opening in gaps.items() for y in range(1, 32)
                 if y not in opening}
        spurs = ((10, 2, 8), (20, 1, 7), (9, 11, 17), (19, 10, 16),
                 (10, 20, 26), (23, 19, 25), (12, 29, 35), (22, 28, 34),
                 (10, 39, 43), (20, 38, 42))
        walls |= {(x, y) for y, left, right in spurs for x in range(left, right)}
        return walls
    if name == MAP_NAMES[4]:
        outer_gates = {(x, 6) for x in (21, 22, 23)}
        outer_gates |= {(x, 26) for x in (21, 22, 23)}
        outer_gates |= {(11, y) for y in (15, 16, 17)}
        outer_gates |= {(33, y) for y in (15, 16, 17)}
        inner_gates = {(22, 11), (22, 21), (17, 16), (27, 16)}
        walls = _rectangular_ring(11, 6, 33, 26, outer_gates)
        walls |= _rectangular_ring(17, 11, 27, 21, inner_gates)
        return walls
    raise ValueError(f"未知地图：{name}")


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


def sequential_rrt_path(start, goal, obstacles, rng, max_samples=1500):
    """Grow a random tree one free grid cell at a time toward one goal.

    This is a 2D grid adaptation of sequential RRT, without vehicle dynamics.
    Return (path, samples); no shortest-path fallback is used on failure.
    """
    if start in obstacles or goal in obstacles:
        return None, 0
    parents = {start: None}
    vertices = [start]
    for sample_count in range(1, max_samples + 1):
        target = goal if rng.random() < 0.20 else (rng.randrange(WIDTH), rng.randrange(HEIGHT))
        nearest = min(vertices, key=lambda p: ((p[0] - target[0]) ** 2 + (p[1] - target[1]) ** 2,
                                               (p[0] - goal[0]) ** 2 + (p[1] - goal[1]) ** 2))
        dx, dy = target[0] - nearest[0], target[1] - nearest[1]
        directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        rng.shuffle(directions)
        directions.sort(key=lambda d: -(d[0] * dx + d[1] * dy))
        next_point = None
        for sx, sy in directions:
            candidate = (nearest[0] + sx, nearest[1] + sy)
            if (0 <= candidate[0] < WIDTH and 0 <= candidate[1] < HEIGHT
                    and candidate not in obstacles and candidate not in parents):
                next_point = candidate
                break
        if next_point is None:
            continue
        parents[next_point] = nearest
        vertices.append(next_point)
        if next_point == goal:
            path = []
            point = goal
            while point is not None:
                path.append(point)
                point = parents[point]
            return list(reversed(path)), sample_count
    return None, max_samples


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


def nearest_neighbor_order(start, stations):
    """Select the geometrically nearest next goal for sequential RRT."""
    remaining = set(stations)
    order = []
    current = start
    while remaining:
        name = min(remaining, key=lambda item: ((current[0] - STATIONS[item][0]) ** 2
                                                 + (current[1] - STATIONS[item][1]) ** 2, item))
        order.append(name)
        remaining.remove(name)
        current = STATIONS[name]
    return order


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
    algorithm: str = "sequential_rrt"
    auto_order: bool = False
    random_seed: int = 0
    rrt_samples: int = 0
    requested_order: list[str] = field(default_factory=list)
    path_searches: int = 0
    memory_start_hits: int = 0
    memory_start_searches: int = 0
    memory_start_repairs: int = 0

    def _event(self, message):
        self.events.append(f"{self.elapsed_seconds:03d}s  {message}")

    def _plan(self):
        goal = HOME if self.target == "HOME" else STATIONS[self.target]
        if self.algorithm == "mgmm_grid":
            path = self.memory.path(self.position, goal, self.obstacles)
        else:
            self.path_searches += 1
            path, samples = sequential_rrt_path(self.position, goal, self.obstacles,
                                                random.Random(self.random_seed + self.path_searches * 1009))
            self.rrt_samples += samples
        if path is None:
            self.state = "FAILED"
            self.reason = f"前往 {self.target} 的道路被堵"
            self.path = []
            self._event(self.reason + "，任务停止")
            return
        self.path = path
        self._event(f"规划前往 {self.target}：{len(path) - 1} 格")

    def _choose_order(self, start, stations):
        if self.algorithm == "mgmm_grid":
            return optimize_visit_order(start, stations, self.obstacles, self.memory)
        return nearest_neighbor_order(start, stations)

    def start(self, order, optimize=False, *, algorithm=None, auto_order=None):
        if self.state not in ("IDLE", "DONE", "FAILED", "CANCELED"):
            raise ValueError("请先结束当前任务")
        order = [item.strip().upper() for item in order]
        if not order or len(set(order)) != len(order) or any(item not in STATIONS for item in order):
            raise ValueError("请输入不重复的配送点，例如 A,B,C")
        self.position = HOME
        self.state = "MOVING"
        self.requested_order = order.copy()
        self.algorithm = algorithm or ("mgmm_grid" if optimize else "sequential_rrt")
        self.auto_order = optimize if auto_order is None else auto_order
        if self.algorithm not in ("mgmm_grid", "sequential_rrt"):
            raise ValueError("未知配送算法")
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
        self.rrt_samples = 0
        self.order = self._choose_order(HOME, order) if self.auto_order else order
        if self.order is None:
            self.order = order
            self.state = "FAILED"
            self.reason = "存在无法到达的配送点"
            self._event(self.reason + "，任务停止")
            return
        self.target = self.order[0]
        self._event("接单：" + " → ".join(order) + " → HOME")
        if self.auto_order:
            self._event("自动排序建议：" + " → ".join(self.order) + " → HOME")
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
        if self.auto_order and len(self.visited) < len(self.order):
            remaining = self.order[len(self.visited):]
            revised = self._choose_order(self.position, remaining)
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
            "planner": self.algorithm,
            "ordering": "auto" if self.auto_order else "fixed",
            "path_searches": (self.memory.searches - self.memory_start_searches
                              if self.algorithm == "mgmm_grid" else self.path_searches),
            "memory_hits": self.memory.hits - self.memory_start_hits if self.algorithm == "mgmm_grid" else 0,
            "memory_repairs": self.memory.repairs - self.memory_start_repairs if self.algorithm == "mgmm_grid" else 0,
            "rrt_samples": self.rrt_samples,
            "reason": self.reason, "events": self.events,
        }


def run_batch(count=10):
    """Run reproducible normal, dynamic-block, and unreachable-goal scenarios."""
    if count < 1:
        raise ValueError("count must be positive")
    orders = list(permutations(("A", "B", "C")))
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


def compare_algorithms(order, obstacles, auto_order, count=1):
    """Run both algorithms under identical map, goals and ordering policy.

    Repeated runs reuse memory only in the MGMM-grid arm. The map stays unchanged,
    so repetitions measure route reuse rather than varying scenarios.
    """
    if count < 1:
        raise ValueError("count must be positive")
    requested = [item.strip().upper() for item in order]
    if not requested or len(set(requested)) != len(requested) or any(item not in STATIONS for item in requested):
        raise ValueError("请输入不重复的配送点，例如 A,B,C")
    shared_memory = RouteMemory()
    rows = []
    for index in range(count):
        pair = {}
        for algorithm in ("sequential_rrt", "mgmm_grid"):
            mission = DeliveryMission(obstacles=set(obstacles),
                                      memory=shared_memory if algorithm == "mgmm_grid" else RouteMemory(),
                                      random_seed=index)
            mission.start(requested, algorithm=algorithm, auto_order=auto_order)
            steps = 0
            while mission.state in ("MOVING", "WAITING") and steps < 500:
                if mission.state == "WAITING":
                    mission.confirm()
                else:
                    mission.tick()
                steps += 1
            if steps >= 500:
                mission.state = "FAILED"
                mission.reason = "仿真步数超限"
            pair[algorithm] = mission.result()
        rows.append({"run": index + 1, "requested_order": requested, **pair})
    return rows
