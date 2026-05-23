# EGO-Planner 项目总览

## 项目坐标
- 仓库：`compare/ego-planner/`（即开源仓库 `ZJU-FAST-Lab/ego-planner`）
- 论文：*"EGO-Planner: An ESDF-Free Gradient-Based Local Planner for Quadrotors"* (RA-L 2020)
- 许可证：GPL-3.0
- 关键信息：本仓库与 `Fast-Drone-250/src/planner` 是同一份代码（前者是单独的 EGO 包，后者把 EGO 封装到了 Fast-Drone-250 框架里），因此 `docs/annotated/` 中已有的 7 篇分析直接适用于本仓库，本目录把行号、路径、上下文统一对齐到 `compare/ego-planner/src/planner/...`。

## 核心设计哲学
EGO 的最大特色是 **"ESDF-Free"**：

> 大部分基于 B-spline 的优化器（Fast-Planner、Bubble Planner 等）都需要先把占用栅格转换为欧几里得有符号距离场（ESDF），ESDF 计算昂贵且更新慢；EGO 用一个"碰撞反弹（collision rebound）"的方法，**只在控制点撞障碍物时才用 A* 搜一段绕障路径，把交点存为 `base_point` 与方向 `direction`**，从而把对障碍物的梯度信息"按需懒计算"——这就回避了 ESDF。

## 模块划分（ROS 包）

```
src/planner/
├── plan_manage/        ← 规划入口和 FSM
│   ├── ego_planner_node.cpp     主程序（ros::spin 而已）
│   ├── ego_replan_fsm.cpp       7 状态有限状态机（核心）
│   ├── planner_manager.cpp      reboundReplan 三步：INIT → OPT → REFINE
│   └── traj_server.cpp          B-spline → PositionCommand
│
├── bspline_opt/        ← 局部 B-spline 优化器
│   ├── uniform_bspline.cpp      三次均匀 B-spline 数据结构与求导
│   └── bspline_optimizer.cpp    L-BFGS + 5 个 cost + collision rebound（核心引擎）
│
├── plan_env/           ← 地图（占用栅格 + 膨胀）
│   ├── grid_map.cpp             概率栅格 + 射线投射 + 膨胀（无 ESDF）
│   └── raycast.cpp              Bresenham 射线投射
│
├── path_searching/     ← 仅在 collision rebound 时使用
│   └── dyn_a_star.cpp           轻量级 A*
│
└── traj_utils/
    ├── polynomial_traj.cpp      多项式（minSnap、one_segment_traj_gen）—— 用于初始路径
    └── planning_visualization.cpp  RViz 可视化
```

## 顶层调用链

```
ros::init  →  EGOReplanFSM::init  →  ros::spin
                              │
                              ├─ Timer(100Hz) execFSMCallback ─ 7状态 switch
                              │      └── 当 EXEC 超时 → REPLAN_TRAJ
                              │             └── callReboundReplan(false,false)
                              │                     ├── getLocalTarget   （在全局轨迹上找 planning_horizon 处的点）
                              │                     ├── reboundReplan
                              │                     │    ├── INIT   多项式 / 当前轨迹延伸
                              │                     │    ├── OPT    BsplineOptimizeTrajRebound（L-BFGS）
                              │                     │    │           ├ smooth + distance + feasibility + swarm + terminal
                              │                     │    │           └ 后期触发 check_collision_and_rebound（A* 找绕障）
                              │                     │    └── REFINE refineTrajAlgo（fitness cost）
                              │                     └── 发布 traj_utils::Bspline 到 planning/bspline
                              │                              └── traj_server 订阅 → 100 Hz 解算 → /position_cmd
                              │
                              └─ Timer(20Hz) checkCollisionCallback
                                   ├── map.getOdomDepthTimeout → EMERGENCY_STOP
                                   └── 沿轨迹采样 → 撞障 → REPLAN_TRAJ / EMERGENCY_STOP
```

## 各模块深度解读索引

| 序号 | 文件 | 重点内容 |
|---|---|---|
| 1 | `1_ego_planner_node.md` | main 函数与节点骨架 |
| 2 | `2_ego_replan_fsm_init.md` | FSM 参数读取、订阅/发布建立、目标源选择 |
| 3 | `3_ego_replan_fsm_execfsm.md` | execFSMCallback 7 状态详解 + getLocalTarget + checkCollisionCallback |
| 4 | `4_traj_server.md` | B-spline → PositionCommand + yaw 限幅 |
| 5 | `5_planner_manager_rebound.md` | reboundReplan 三步法 + 时间重分配 |
| 6 | `6_grid_map.md` | 概率栅格 + 射线投射 + 膨胀 + 碰撞查询 |
| 7 | `7_bspline_optimizer_cost.md` | 5 个 cost 数学形式 + collision rebound + L-BFGS |

> 注：本仓库与 `docs/annotated/` 中的内容一致。若你之前读过 `docs/annotated/` 的版本，本目录可作为对照——把所有 `official/Fast-Drone-250/src/planner/...` 路径替换为 `compare/ego-planner/src/planner/...` 即可。
