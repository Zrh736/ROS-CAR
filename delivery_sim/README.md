# Windows 多点配送二维仿真

这是不连接 ROS、底盘或实车的桌面演示。使用纯 Python 标准库，地图为 55 × 39 格，每格代表 0.5 米；虚拟车从 H（装载点）出发，到所选配送点 A/B/C/D/E 送货，每站等待人工确认，最后返回 H。可切换“经典障碍”“仓库通道”“空旷地图”“迷宫地图”“环形地图”五种基础结构。每次选图都会叠加少量随机障碍，仓库使用成组货物障碍；空旷地图也有少量散落障碍。迷宫地图包含交替通道和岔路，环形地图包含三层带入口的环形障碍。生成时会检查 H 到五个配送点仍可到达。地图中深色格为障碍，可在运行中点击增删；路径被挡时尝试重新规划，无路可走则停止并记录原因。地图界面显示计划路线和已行轨迹。

## 运行

在 Windows 中双击仓库根目录的 `start_delivery_sim.cmd`。也可以在仓库根目录打开 PowerShell：

```powershell
python -m delivery_sim
```

输入 `A,B,C`、`E,D,A` 等不重复的配送点，点击“开始配送”。到站后点击“确认取货”；可以暂停、取消或在地图上点击增删障碍。切换地图会生成新的随机场景并重置任务。“恢复本次地图”会清除手动修改，保留当前场景编号；“换随机障碍”会生成新的场景。场景编号及完整障碍清单会随对比结果保存，便于复查。选择“按输入顺序配送”会严格遵循输入顺序；选择“自动排序”会自行安排这些点。交互演示使用 MGMM 思路二维版。“内置场景自动测试 10 次”仍使用固定示例，不读取输入框或当前地图，包含一个预期失败场景，正常结果是 9/10 成功。

## 两种配送算法对比

配送方式与算法分开选择。配送方式是“自动排序”或“按输入顺序”；对比按钮会在**相同的当前地图、手动障碍、配送点和配送方式**下运行这两种算法：

- **SequentialRRT（二维网格版）**：自动排序时先选几何距离最近的未配送点；每一段单独运行随机探索树（RRT），在空格上采样并逐格扩展，找到目标后沿树回溯路径。不使用 BFS 作为失败时的隐藏替代。按输入顺序时只改变站点顺序，仍逐段运行 RRT。
- **MGMM 思路二维版**：缓存已算过的站点间网格路径，复用前检查障碍；自动排序时枚举所选站点的顺序，选择总网格路程最短的路线。按输入顺序时保留输入顺序，只复用路径。这里使用 BFS 建立和修复缓存，并未运行论文中的学习模型。

“当前条件对比 1 次”分别运行两种算法一次。“当前条件对比 10 次”在同一静态地图和同一订单上重复十次；SequentialRRT 每次使用新的随机种子重新探索，MGMM 思路二维版可跨次复用缓存。这十次是重复试验，不是十张不同地图。弹窗分别显示实际访问顺序、合计路程、RRT 采样次数、寻路次数和记忆命中次数；每次轨迹保存在 `artifacts/delivery_sim/comparison_*.json`。两种算法在固定站点顺序下仍可能产生不同路程，因为 RRT 不保证最短路径；自动排序还可能选出不同站点顺序。这些数字只反映本程序的网格仿真，不代表论文的性能结果。

核心逻辑检查：

```powershell
python -m unittest delivery_sim.test_core
```

## 验证范围

### 作者原版 C++ 算法（独立实验入口）

要使用论文作者的**原始代码**，在 Ubuntu 虚拟机内运行 `python3 -m delivery_sim.author_original --backend local --scene Maze --grid 2 --instance 0`；也可以在装有 WSL 的 Windows 上运行 `python -m delivery_sim.author_original --backend wsl --scene Maze --grid 2 --instance 0`。它会调用[作者仓库](https://github.com/yuanjielu-64/MGMM_ICRA2025)编译出来的 `bin/Runner`，在作者提供的**同一个场景样例**上分别使用 `UseMP Sequential` 和 `UseMP Dromos`；后者是作者 README 中 MGMM 的入口。脚本会记录作者仓库提交号、原始程序输出、求解状态、规划时间和路程，保存到 `artifacts/delivery_sim/author_original_*.json`。原版运行失败时直接报错，不退回到本项目的 Python 算法。

已在 Ubuntu 20.04 虚拟机用作者提交 `107a309f787f354ec74e47084008b8b007bc0091` 编译、运行 Maze／网格 2／样例 0：原版 `Sequential` 和 `Dromos` 都得到 `[solved = 1]`。结果文件在 `artifacts/delivery_sim/author_original_*.json`；规划含随机性，一次运行的数值不是论文性能结论。虚拟机内的本项目入口位于 `~/ROS-CAR-delivery-sim`，作者源码位于 `~/MGMM_ICRA2025`。

作者的图形窗口 `GRunMP` 也已在虚拟机桌面打开过。要再次打开迷宫样例，在 Ubuntu 终端进入 `~/MGMM_ICRA2025` 后运行：

```bash
./bin/Runner GRunMP data/ParamsSceneMazeForCar.txt ParamsExtraFile data/Instances/MazeCarNrGrids2/0.txt MGMMPredictLabel data/PredictLabel/ UseMP Dromos UseMap Maze PlannerStatsFile data/Results/gui_Maze_2_0.txt UseGrid 2 UseObstacle 0 TopNumber 5
```

窗口内按作者 README 的操作：右键打开 “Motion Planner”，选择 “solve repeat”，再选择 “animate solution” 观看路径。切换到 `UseMP Sequential` 可看原版顺序规划器。图形窗口和上面的命令行统计入口使用同一作者场景，但各次运行随机采样会使结果不同。

重新安装时需要克隆作者仓库、检出其记录的 Treelite 子模块版本并编译，确保 `bin/Runner`、车体图形资源 `gdata/`、对应的 `data/Instances/`、`data/Obstacles/`、`data/PredictLabel/`、`data/goalPosition/`、`data/path/` 都齐全。此次虚拟机使用 CMake 3.27.9；作者 `CMakeLists.txt` 中硬编码的 `/usr/local/lib/lib_lightgbm.so` 在虚拟机不存在，构建前只从该文件中移除了这一条链接路径，算法 C++ 源码未改。Windows WSL 模式可用 `--distro` 指定发行版名称；两种模式均可用 `--linux-repo` 指定 Ubuntu 内的仓库路径。作者项目依赖 C++、OpenGL/GLUT、PNG、Treelite 和 LKH，不能直接运行 Windows `.exe`。

原版实验使用作者的 Random、Curves、Maze、Storage 场景与目标点，**不会读取本窗口的地图、随机障碍或 A–E 订单**。窗口中的“当前条件对比”仍是本项目的网格演示。若要让作者算法规划窗口里的自定义地图，还需实现地图/目标格式转换，并生成与新场景对应的 MGMM 预测数据；只替换算法名称无法做到等价运行。

### Windows 网格演示

论文把 SequentialRRT 作为逐个访问最近目标的随机树基线；MGMM 则使用学习得到的轨迹和目标间距离，指导运动地图、访问顺序及运动树扩展。这里的两个版本是独立编写的**二维网格演示**，**没有使用或复制[作者 C++ 源码](https://github.com/yuanjielu-64/MGMM_ICRA2025)**。MGMM 思路二维版没有[论文](https://arxiv.org/abs/2407.11399)的训练模型、运动地图采样或车辆动力学约束，不能称为 MGMM 原算法复现；SequentialRRT 网格版也没有论文的车辆动力学。

本程序验证订单顺序、到站停等、人工确认、网格寻路、改道和停止等任务流程。地图是示意图，位置、激光、里程计和运动均为软件模型；它**没有调用 Nav2，也不能证明真实小车的定位、避障或制动性能**。接入 ROS 2 时，需要用 Nav2 导航反馈替换网格移动，并单独设计配送模式的运动授权，不得直接绕过 `motion_guard`。
