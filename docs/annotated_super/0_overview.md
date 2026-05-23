# SUPER 项目总览

## 项目坐标
- 仓库：`compare/SUPER/`，HKU MaRS Lab 出品
- 论文：*"Safety-assured high-speed navigation for MAVs"* (Sci. Robotics 2025)
- 配套依赖：自带 `rog_map`（ROG-Map）、`mars_uav_sim`（仿真器）、`mission_planner`（航点任务）
- 许可证：GPL-3.0
- 整体定位：**针对未知环境高速避障** 的层次化运动规划框架，强调"已知-未知"边界处理与 **应急备份轨迹**。

## 核心设计哲学

SUPER 的三个关键词：**ROG-Map / 凸多面体 / 双轨迹**

### ROG-Map（占用栅格 + 滑窗 + 可选 ESDF）
- 与 EGO 的"无 ESDF + 概率占用图"不同，ROG-Map 同时维护：概率占用图 (`prob_map`)、膨胀图 (`inf_map`)、ESDF (`esdf_map`，可选)、自由空间计数图 (`free_cnt_map`，前沿提取用)。
- 全部继承 `SlidingMap` —— 当机器人飞远时地图整体平移，而不是每次重建（参见 `rog_map/include/rog_map/rog_map_core/sliding_map.h`）。

### 凸多面体走廊（Safe Flight Corridor, SFC）
- 用 **CIRI** 算法（`super_planner/src/super_core/ciri.cpp`，419 行）从一段种子线段（seed line）膨胀出一个凸多面体（通过迭代生成椭球切平面）。
- `CorridorGenerator::SearchPolytopeOnPath`（`corridor_generator.cpp`）把前端 A* 给出的折线切成多段，每段都生成一个 SFC，相邻多面体必须有足够大的 overlap 才能接续。

### 双轨迹策略（Exp + Backup）
SUPER 最具特色的设计——**每次 replan 同时输出两条轨迹**：

- **Exp Trajectory（探索轨迹）**：从当前状态飞向全局目标，会延伸进 **未知区域**。允许"赌博式"飞快。
- **Backup Trajectory（备份轨迹）**：当 Exp 轨迹飞到某一时刻（"接管时刻"）发现前方不可见，自动衔接备份轨迹，**让无人机减速并停留在已知自由空间内**。
- 两条轨迹都用 **MINCO**（**MIN**imum **CO**ntrol，s=4 即 7 阶多项式段）表示，飞控按时间执行 `cmd_traj_info_`，自动从 Exp 切到 Backup。

## 模块划分

```
SUPER/
├── super_planner/         ← 规划主模块
│   ├── Apps/
│   │   ├── fsm_node_ros1.cpp / fsm_node_ros2.cpp     ROS1 / ROS2 入口
│   │   ├── read_replan_log.cpp                       离线分析工具
│   │   └── traj_opt_tuning.cpp                       轨迹优化器调参
│   │
│   ├── include/fsm/ + src/super_core/fsm.cpp         有限状态机（6 状态）
│   ├── include/super_core/ + src/super_core/         核心规划器
│   │   ├── super_planner.cpp                         规划主类（PlanFromRest / ReplanOnce）
│   │   ├── corridor_generator.cpp                    SFC 生成（连续凸多面体）
│   │   ├── ciri.cpp                                   CIRI 凸多面体生成器
│   │   ├── astar.cpp                                  前端 A*
│   │   └── fov_checker.h                              视场剪裁
│   │
│   ├── include/data_structure/ + src/utils/          数据结构 & 工具
│   │   ├── exp_traj.h / backup_traj.h / cmd_traj.h    三类轨迹数据结构
│   │   ├── trajectory.cpp / piece.cpp                  MINCO 多项式片段
│   │   ├── polytope.cpp / ellipsoid.cpp                凸多面体 / 椭球
│   │   └── geometry_utils.cpp                          几何工具
│   │
│   ├── src/traj_opt/                                 轨迹优化（MINCO）
│   │   ├── exp_traj_optimizer_s4.cpp                  探索轨迹（s=4，含 attitude/thrust 约束）
│   │   ├── backup_traj_optimizer_s4.cpp               备份轨迹（含接管时刻可变优化）
│   │   └── yaw_traj_opt.cpp                            航向轨迹（独立 1D 优化）
│   │
│   ├── include/utils/optimization/                   底层优化算子
│   │   ├── minco.h                                    MINCO（带 banded system 求解）
│   │   ├── lbfgs.h                                    L-BFGS
│   │   ├── sdlp.h / sdqp.hpp / mvie.h                  辅助求解器
│   │   └── waypoint_trajectory_optimizer.h            通用轨迹优化器
│   │
│   └── include/ros_interface/                         ROS1/ROS2 抽象层
│
├── rog_map/               ← 地图模块（独立可复用）
│   ├── include/rog_map/
│   │   ├── rog_map.h                                  主类（继承 ProbMap）
│   │   ├── prob_map.h, inf_map.h, esdf_map.h, free_cnt_map.h
│   │   └── rog_map_core/sliding_map.h, raycaster.h, counter_map.h
│   └── src/rog_map/                                   各子地图实现
│
├── mission_planner/       ← 航点任务（外层调用 super_planner）
│   ├── Apps/ros1_waypoint_mission.cpp                 ROS1 入口
│   └── include/waypoint_mission/                       任务管理类
│
└── mars_uav_sim/          ← 内置仿真器（用于 closed-loop 测试）
```

## 顶层调用链

```
ros::init  →  FsmRos1::init  →  ros::AsyncSpinner spin
                          │
                          ├─ Timer / Click：setGoalPosiAndYaw → started_=true, gi_.new_goal=true
                          │
                          ├─ Timer(replan_rate Hz) callMainFsmOnce
                          │      switch(machine_state_)
                          │        INIT       → 等待 odom → WAIT_GOAL
                          │        WAIT_GOAL  → 收到目标 → GENERATE_TRAJ
                          │        GENERATE_TRAJ → planner_ptr_->PlanFromRest() → FOLLOW_TRAJ
                          │        FOLLOW_TRAJ  → publishCurPoseToPath
                          │        EMER_STOP    → WAIT_GOAL
                          │
                          └─ Timer(replan_rate Hz) callReplanOnce  （只在 FOLLOW_TRAJ 触发）
                                  └── planner_ptr_->ReplanOnce()
                                          ├── generateExpTraj
                                          │     ├── 1) 从上一段轨迹热启动（碰撞检查、receding）
                                          │     ├── 2) 在 inf_map 上跑 A* PathSearch
                                          │     ├── 3) CorridorGenerator::SearchPolytopeOnPath（多段 SFC）
                                          │     ├── 4) ExpTrajOpt::optimize（MINCO + L-BFGS）
                                          │     └── 5) YawTrajOpt::optimize（1D 航向）
                                          │
                                          └── generateBackupTrajectory
                                                ├── 1) 找"接管时刻"（line-of-sight 失效处）
                                                ├── 2) 沿当前位置→ seed_point 生成单个 SFC
                                                ├── 3) BackupTrajOpt::optimize（MINCO + L-BFGS，接管时刻可变）
                                                └── 4) 拼接 cmd_traj_info_：[exp 头段 + 新 exp + backup]
```

## 各模块深度解读索引

| 序号 | 文件 | 重点 |
|---|---|---|
| 1 | `1_fsm_node.md` | 程序入口，配置加载 |
| 2 | `2_fsm.md` | 6 状态状态机（INIT/WAIT_GOAL/YAWING/GENERATE_TRAJ/FOLLOW_TRAJ/EMER_STOP） |
| 3 | `3_super_planner_main.md` | SuperPlanner 类构造 / PlanFromRest / ReplanOnce |
| 4 | `4_generate_exp_traj.md` | 探索轨迹生成全流程（A* + SFC + MINCO 优化） |
| 5 | `5_generate_backup_traj.md` | 备份轨迹与接管时刻 |
| 6 | `6_corridor_ciri.md` | 安全飞行走廊（CIRI 凸多面体 + 走廊串联） |
| 7 | `7_minco_trajopt.md` | MINCO 数据结构 + Exp/Backup 优化器 cost 函数 |
| 8 | `8_rog_map.md` | ROG-Map：ProbMap/InfMap/ESDFMap/SlidingMap |
| 9 | `9_astar.md` | A* 前端搜索（支持 inf/prob 两种模式 + escape） |

