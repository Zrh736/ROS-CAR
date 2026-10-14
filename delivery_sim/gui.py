"""Tkinter visualization for the hardware-free delivery mission."""

from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk

from .core import (CELL_METERS, HEIGHT, HOME, MAP_NAMES, STATIONS, WIDTH,
                   DeliveryMission, compare_algorithms, obstacles_for_map,
                   run_batch)

CELL = 28
COLORS = {"IDLE": "#526579", "MOVING": "#2176ae", "WAITING": "#bc761a",
          "DONE": "#238247", "FAILED": "#b43f40", "CANCELED": "#667085"}


class DeliveryApp:
    def __init__(self, root):
        self.root = root
        self.mission = DeliveryMission()
        self.paused = False
        self.last_event_count = 0
        root.title("ROS-CAR 多点配送 · Windows 二维仿真")
        root.minsize(1030, 650)
        root.configure(bg="#f4f6f9")

        body = ttk.Frame(root, padding=18)
        body.pack(fill="both", expand=True)
        ttk.Label(body, text="多点配送仿真", font=("Microsoft YaHei UI", 18, "bold")).pack(anchor="w")
        ttk.Label(body, text="每格 0.5 米｜点击地图可放置或移除障碍｜此界面不连接小车或 ROS",
                  foreground="#526579").pack(anchor="w", pady=(2, 12))

        content = ttk.Frame(body)
        content.pack(fill="both", expand=True)
        self.canvas = tk.Canvas(content, width=WIDTH * CELL, height=HEIGHT * CELL,
                                bg="white", highlightthickness=1, highlightbackground="#d5dce5")
        self.canvas.pack(side="left", anchor="n")
        self.canvas.bind("<Button-1>", self.on_map_click)

        panel = ttk.Frame(content, padding=(20, 0, 0, 0))
        panel.pack(side="left", fill="both", expand=True)
        ttk.Label(panel, text="示例地图", font=("Microsoft YaHei UI", 10, "bold")).pack(anchor="w")
        self.map_var = tk.StringVar(value=MAP_NAMES[0])
        map_picker = ttk.Combobox(panel, textvariable=self.map_var, values=MAP_NAMES, state="readonly")
        map_picker.pack(fill="x", pady=(5, 10))
        map_picker.bind("<<ComboboxSelected>>", self.select_map)
        ttk.Label(panel, text="配送点（A、B、C、D、E）", font=("Microsoft YaHei UI", 10, "bold")).pack(anchor="w")
        self.order_var = tk.StringVar(value="A,B,C")
        ttk.Entry(panel, textvariable=self.order_var).pack(fill="x", pady=(5, 10))
        ttk.Label(panel, text="自动排序只看选了哪些点；按输入顺序模式才遵守填写顺序。",
                  foreground="#526579", wraplength=290).pack(anchor="w", pady=(0, 6))
        self.ordering_var = tk.StringVar(value="auto")
        ttk.Radiobutton(panel, text="自动排序",
                        variable=self.ordering_var, value="auto").pack(anchor="w")
        ttk.Radiobutton(panel, text="按输入顺序配送",
                        variable=self.ordering_var, value="fixed").pack(anchor="w", pady=(0, 8))
        ttk.Label(panel, text="开始配送使用 MGMM 思路二维版；对比会运行 SequentialRRT 网格版和 MGMM 思路二维版。",
                  foreground="#526579", wraplength=290).pack(anchor="w", pady=(0, 6))
        ttk.Label(panel, text="均为二维教学实现，不是论文原版的车辆动力学或学习模型。",
                  foreground="#526579", wraplength=290).pack(anchor="w", pady=(0, 6))

        buttons = ttk.Frame(panel)
        buttons.pack(fill="x")
        for label, command in (("开始配送", self.start), ("确认取货", self.confirm),
                               ("暂停 / 继续", self.pause), ("取消任务", self.cancel),
                               ("重置地图", self.reset), ("当前条件对比 1 次", self.compare),
                               ("当前条件对比 10 次", self.compare_benchmark),
                               ("内置场景自动测试 10 次", self.batch)):
            ttk.Button(buttons, text=label, command=command).pack(fill="x", pady=2)

        ttk.Separator(panel).pack(fill="x", pady=12)
        self.state_var = tk.StringVar()
        self.metrics_var = tk.StringVar()
        ttk.Label(panel, textvariable=self.state_var, font=("Microsoft YaHei UI", 12, "bold")).pack(anchor="w")
        ttk.Label(panel, textvariable=self.metrics_var, justify="left").pack(anchor="w", pady=(5, 12))
        ttk.Label(panel, text="任务记录", font=("Microsoft YaHei UI", 10, "bold")).pack(anchor="w")
        self.log = tk.Text(panel, width=34, height=13, state="disabled", wrap="word",
                           font=("Consolas", 9), bg="#f9fbfd")
        self.log.pack(fill="both", expand=True, pady=(5, 0))
        self.render()
        root.after(180, self.loop)

    def _append_events(self):
        new = self.mission.events[self.last_event_count:]
        if not new:
            return
        self.log.configure(state="normal")
        for event in new:
            self.log.insert("end", event + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")
        self.last_event_count = len(self.mission.events)

    def render(self):
        self.canvas.delete("all")
        obstacles = self.mission.obstacles
        route = set(self.mission.path[1:])
        for y in range(HEIGHT):
            for x in range(WIDTH):
                point = (x, y)
                color = "#2d3748" if point in obstacles else "#cce6f5" if point in route else "#ffffff"
                self.canvas.create_rectangle(x * CELL, y * CELL, (x + 1) * CELL,
                                             (y + 1) * CELL, fill=color, outline="#e6ebf0")
        for name, point in (("H", HOME), *STATIONS.items()):
            x, y = point
            fill = "#3e6289" if name == "H" else "#238247" if name in self.mission.visited else "#8d56b6"
            self.canvas.create_oval(x * CELL + 2, y * CELL + 2, (x + 1) * CELL - 2,
                                    (y + 1) * CELL - 2, fill=fill, outline="")
            self.canvas.create_text(x * CELL + CELL / 2, y * CELL + CELL / 2,
                                    text=name, fill="white", font=("Arial", 10, "bold"))
        x, y = self.mission.position
        self.canvas.create_oval(x * CELL + 5, y * CELL + 5, (x + 1) * CELL - 5,
                                (y + 1) * CELL - 5, fill="#ed7a28", outline="#8d410d", width=2)
        state = self.mission.state
        suffix = "（已暂停）" if self.paused and state == "MOVING" else ""
        self.state_var.set(f"状态：{state}{suffix}    目标：{self.mission.target or '无'}")
        order = " → ".join(self.mission.order) if self.mission.order else "尚未开始"
        counts = self.mission.result()
        self.metrics_var.set(
            f"配送顺序：{order}\n"
            f"已送达：{'、'.join(self.mission.visited) or '无'}\n"
            f"路程：{self.mission.distance_cells * CELL_METERS:.1f} 米\n"
            f"仿真时间：{self.mission.elapsed_seconds} 秒\n"
            f"重新规划：{self.mission.replans} 次\n"
            f"寻路计算：{counts['path_searches']} 次｜记忆复用：{counts['memory_hits']} 次"
        )
        self._append_events()

    def loop(self):
        if not self.paused:
            self.mission.tick()
        self.render()
        self.root.after(180, self.loop)

    def start(self):
        try:
            order = self._entered_order()
            self.mission.start(order, algorithm="mgmm_grid", auto_order=self.ordering_var.get() == "auto")
        except ValueError as exc:
            messagebox.showerror("无法开始", str(exc))
        else:
            self.log.configure(state="normal")
            self.log.delete("1.0", "end")
            self.log.configure(state="disabled")
            self.last_event_count = 0
            self.paused = False
            self.render()

    def confirm(self):
        try:
            self.mission.confirm()
        except ValueError as exc:
            messagebox.showinfo("提示", str(exc))
        self.render()

    def pause(self):
        if self.mission.state == "MOVING":
            self.paused = not self.paused
            self.render()

    def cancel(self):
        self.mission.cancel()
        self.paused = False
        self.render()

    def reset(self):
        self.mission = DeliveryMission(obstacles=obstacles_for_map(self.map_var.get()))
        self.paused = False
        self.last_event_count = 0
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")
        self.render()

    def select_map(self, _event=None):
        self.reset()

    def _entered_order(self):
        order = [item.strip().upper() for item in self.order_var.get().replace("，", ",").split(",")]
        if not order or len(set(order)) != len(order) or any(item not in STATIONS for item in order):
            raise ValueError("请输入不重复的配送点，例如 A,B,C 或 E,D,A")
        return order

    def on_map_click(self, event):
        point = (event.x // CELL, event.y // CELL)
        if self.mission.toggle_obstacle(point):
            self.render()

    def batch(self):
        results = run_batch(10)
        report = {
            "kind": "desktop_grid_simulation",
            "cell_meters": CELL_METERS,
            "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "successes": sum(item["state"] == "DONE" for item in results),
            "runs": results,
        }
        directory = Path(__file__).resolve().parents[1] / "artifacts" / "delivery_sim"
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / f"batch_{datetime.now():%Y%m%d_%H%M%S}.json"
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        messagebox.showinfo("自动测试完成", f"成功 {report['successes']}/10 次\n记录已保存：\n{path}")

    def compare(self):
        self._compare(1)

    def compare_benchmark(self):
        self._compare(10)

    def _compare(self, count):
        try:
            order = self._entered_order()
        except ValueError as exc:
            messagebox.showerror("无法对比", str(exc))
            return
        auto_order = self.ordering_var.get() == "auto"
        rows = compare_algorithms(order, self.mission.obstacles, auto_order, count)
        baseline = [row["sequential_rrt"] for row in rows]
        memory = [row["mgmm_grid"] for row in rows]
        report = {
            "kind": "sequential_rrt_vs_mgmm_grid_demo",
            "note": "SequentialRRT is a random-tree grid adaptation. MGMM-grid is an independently implemented path-cache and tour analogy, without the paper's trained predictor, guided roadmap, or vehicle dynamics. These are not the original paper implementations. Repeated runs use the same static map and order; memory carries across runs.",
            "map": self.map_var.get(),
            "requested_order": order,
            "ordering": "auto" if auto_order else "fixed",
            "obstacles": sorted(self.mission.obstacles),
            "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "runs": rows,
        }
        directory = Path(__file__).resolve().parents[1] / "artifacts" / "delivery_sim"
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / f"comparison_{datetime.now():%Y%m%d_%H%M%S_%f}.json"
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        first_baseline, first_memory = baseline[0], memory[0]
        policy = "自动排序" if auto_order else "按输入顺序"
        baseline_distance = sum(item["distance_m"] for item in baseline)
        memory_distance = sum(item["distance_m"] for item in memory)
        baseline_searches = sum(item["path_searches"] for item in baseline)
        rrt_samples = sum(item["rrt_samples"] for item in baseline)
        memory_searches = sum(item["path_searches"] for item in memory)
        memory_hits = sum(item["memory_hits"] for item in memory)
        explanation = "10 次使用同一地图和订单，路径记忆可跨次复用。" if count > 1 else "两种算法在相同条件下各运行一次。"
        messagebox.showinfo(
            "算法对比完成",
            f"地图：{self.map_var.get()}｜配送方式：{policy}\n"
            f"配送点：{' → '.join(order)}｜重复 {count} 次\n{explanation}\n\n"
            f"SequentialRRT（网格版）：{' → '.join(first_baseline['order'])} → H\n"
            f"合计 {baseline_distance:.1f} 米，RRT 规划 {baseline_searches} 次、采样 {rrt_samples} 次，完成 {sum(r['state'] == 'DONE' for r in baseline)}/{count}\n\n"
            f"MGMM 思路二维版：{' → '.join(first_memory['order'])} → H\n"
            f"合计 {memory_distance:.1f} 米，寻路 {memory_searches} 次，复用 {memory_hits} 次，完成 {sum(r['state'] == 'DONE' for r in memory)}/{count}\n\n"
            f"详细记录：\n{path}",
        )


def main():
    root = tk.Tk()
    DeliveryApp(root)
    root.mainloop()
