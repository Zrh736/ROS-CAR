"""Tkinter visualization for the hardware-free delivery mission."""

from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import messagebox, ttk

from .core import CELL_METERS, HEIGHT, HOME, STATIONS, WIDTH, DeliveryMission, run_batch

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
        ttk.Label(panel, text="配送顺序（A、B、C）", font=("Microsoft YaHei UI", 10, "bold")).pack(anchor="w")
        self.order_var = tk.StringVar(value="A,B,C")
        ttk.Entry(panel, textvariable=self.order_var).pack(fill="x", pady=(5, 10))

        buttons = ttk.Frame(panel)
        buttons.pack(fill="x")
        for label, command in (("开始配送", self.start), ("确认取货", self.confirm),
                               ("暂停 / 继续", self.pause), ("取消任务", self.cancel),
                               ("重置地图", self.reset), ("自动测试 10 次", self.batch)):
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
        self.metrics_var.set(
            f"已送达：{'、'.join(self.mission.visited) or '无'}\n"
            f"路程：{self.mission.distance_cells * CELL_METERS:.1f} 米\n"
            f"仿真时间：{self.mission.elapsed_seconds} 秒\n"
            f"重新规划：{self.mission.replans} 次"
        )
        self._append_events()

    def loop(self):
        if not self.paused:
            self.mission.tick()
        self.render()
        self.root.after(180, self.loop)

    def start(self):
        try:
            order = self.order_var.get().replace("，", ",").split(",")
            self.mission.start(order)
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
        self.mission = DeliveryMission()
        self.paused = False
        self.last_event_count = 0
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")
        self.render()

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


def main():
    root = tk.Tk()
    DeliveryApp(root)
    root.mainloop()
