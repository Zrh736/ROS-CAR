"""Run the original MGMM repository's C++ planners through Ubuntu on WSL.

This command deliberately does not use the local Tk grid or Python planners.
The author's scene, instance, prediction data, and compiled Runner are required.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from datetime import datetime
from pathlib import Path


SCENES = ("Random", "Curves", "Maze", "Storage")
PLANNERS = ("Sequential", "Dromos")
METRIC_PATTERNS = {
    "solved": re.compile(r"\[solved\s*=\s*(-?\d+)\]"),
    "planning_seconds": re.compile(r"\[TimeSolve\s*=\s*([\d.eE+-]+)\]"),
    "distance": re.compile(r"\[SolDistance\s*=\s*([\d.eE+-]+)\]"),
}


def runner_command(scene: str, grid: int, instance: int, planner: str, result_file: str):
    """Make argv for the author's Runner, without shell interpolation."""
    if scene not in SCENES or grid not in (2, 3, 4) or not 0 <= instance <= 400:
        raise ValueError("场景、网格或样例编号无效")
    if planner not in PLANNERS:
        raise ValueError("未知原版规划器")
    return [
        "./bin/Runner", "RunPlanner", f"data/ParamsScene{scene}ForCar.txt",
        "ParamsExtraFile", f"data/Instances/{scene}CarNrGrids{grid}/{instance}.txt",
        "MGMMPredictLabel", "data/PredictLabel/", "UseMP", planner,
        "UseMap", scene, "PlannerStatsFile", result_file,
        "UseGrid", str(grid), "UseObstacle", str(instance), "TopNumber", "5",
    ]


def parse_runner_output(output: str):
    values = {}
    for name, pattern in METRIC_PATTERNS.items():
        match = pattern.search(output)
        if match is None:
            raise ValueError(f"作者程序未输出 {name}，请查看保存的原始日志")
        values[name] = int(match.group(1)) if name == "solved" else float(match.group(1))
    return values


def run_author_planners(scene: str, grid: int, instance: int, distro: str,
                        linux_repo: str, timeout: int):
    """Run both original planners on the same author-provided instance."""
    probe = subprocess.run(["wsl.exe", "--list", "--quiet"], capture_output=True,
                           text=True, errors="replace", check=False)
    if probe.returncode != 0:
        raise RuntimeError("未找到可运行的 WSL Linux 发行版；请先安装 Ubuntu/WSL。")

    report = {
        "source": "yuanjielu-64/MGMM_ICRA2025 original C++ Runner",
        "repository": "https://github.com/yuanjielu-64/MGMM_ICRA2025",
        "scene": scene, "grid": grid, "instance": instance,
        "wsl_distribution": distro, "linux_repository": linux_repo,
        "runs": {},
    }
    # The shell receives all variable values as positional arguments. Nothing
    # from the CLI is interpolated into the shell program.
    shell_program = (
        'set -e; case "$1" in /*) cd "$1";; *) cd "$HOME/$1";; esac; '
        'test -x ./bin/Runner || { echo "缺少 bin/Runner：请先在 Ubuntu 编译作者项目" >&2; exit 2; }; '
        'test -f "$2" || { echo "缺少作者场景样例：$2" >&2; exit 2; }; '
        'test -f "$3" || { echo "缺少作者预测数据：$3" >&2; exit 2; }; '
        'mkdir -p data/Results; git rev-parse HEAD; shift 3; "$@"'
    )
    instance_file = f"data/Instances/{scene}CarNrGrids{grid}/{instance}.txt"
    prediction_file = f"data/PredictLabel/{scene}_{grid}/predict_label_{instance}.txt"
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    for planner in PLANNERS:
        stats_file = f"data/Results/codex_original_{planner}_{scene}_{grid}_{instance}_{stamp}.txt"
        command = runner_command(scene, grid, instance, planner, stats_file)
        process = subprocess.run(
            ["wsl.exe", "--distribution", distro, "--exec", "bash", "-lc",
             shell_program, "author-runner", linux_repo, instance_file,
             prediction_file, *command],
            capture_output=True, text=True, errors="replace", check=False,
            timeout=timeout,
        )
        output = process.stdout + "\n" + process.stderr
        report["runs"][planner] = {
            "command": command, "stats_file_in_author_repo": stats_file,
            "returncode": process.returncode, "raw_output": output,
        }
        if process.returncode != 0:
            raise RuntimeError(f"作者 {planner} 程序运行失败：\n{output[-3000:]}")
        # The first stdout line is the commit emitted by the shell preamble.
        lines = process.stdout.splitlines()
        if lines:
            report["author_commit"] = lines[0].strip()
        report["runs"][planner]["metrics"] = parse_runner_output(output)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description="使用作者原版 C++ 程序对比 MGMM 和 Sequential")
    parser.add_argument("--scene", choices=SCENES, default="Maze")
    parser.add_argument("--grid", type=int, choices=(2, 3, 4), default=2)
    parser.add_argument("--instance", type=int, default=0)
    parser.add_argument("--distro", default="Ubuntu-20.04", help="WSL 发行版名称")
    parser.add_argument("--linux-repo", default="MGMM_ICRA2025",
                        help="Ubuntu 内作者仓库的路径，默认为 ~/MGMM_ICRA2025")
    parser.add_argument("--timeout", type=int, default=180)
    args = parser.parse_args(argv)
    if not 0 <= args.instance <= 400 or args.timeout < 1:
        parser.error("样例编号须为 0–400，超时须大于 0")
    try:
        report = run_author_planners(args.scene, args.grid, args.instance,
                                     args.distro, args.linux_repo, args.timeout)
    except (RuntimeError, ValueError, subprocess.TimeoutExpired, FileNotFoundError) as exc:
        print(f"作者原版对比未完成：{exc}", file=sys.stderr)
        return 1
    output_dir = Path(__file__).resolve().parents[1] / "artifacts" / "delivery_sim"
    output_dir.mkdir(parents=True, exist_ok=True)
    output_file = output_dir / f"author_original_{datetime.now():%Y%m%d_%H%M%S}.json"
    output_file.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    for planner in PLANNERS:
        metrics = report["runs"][planner]["metrics"]
        print(f"{planner}: 完成={metrics['solved'] == 1}  "
              f"规划时间={metrics['planning_seconds']:.3f}秒  路程={metrics['distance']:.2f}")
    print(f"作者提交：{report['author_commit']}\n完整记录：{output_file}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
