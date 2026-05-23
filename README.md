# EGO-Planner vs SUPER 对比研究

> 一份针对两款开源四旋翼运动规划器的源码精读 + 横向对比 + 实机部署的研究归档。

| 项目 | 仓库 | 论文 | 年份 |
|---|---|---|---|
| **EGO-Planner** | [ZJU-FAST-Lab/ego-planner](https://github.com/ZJU-FAST-Lab/ego-planner) | *EGO-Planner: An ESDF-Free Gradient-Based Local Planner for Quadrotors* | RA-L 2020 |
| **SUPER** | [hku-mars/SUPER](https://github.com/hku-mars/SUPER) | *Safety-assured high-speed navigation for MAVs* | Sci. Robotics 2025 |

两者都来自 HKU/ZJU 系，但选择了相反的工程取舍——EGO 做减法（无 ESDF + B-spline + 5 个 cost），SUPER 做加法（ROG-Map + MINCO + CIRI + 双轨迹）。本仓库把它们放到同一坐标系下读懂、对比、上机。

---

## 仓库内容

```
.
├── ego-planner/              EGO 源码（与 ZJU-FAST-Lab 主干一致）
├── SUPER/                    SUPER 源码（与 hku-mars 主干一致，含 mars_uav_sim 仿真器）
└── docs/
    ├── annotated/            EGO 7 篇源码精读（function/L行 级注释）
    │   ├── 1_ego_planner_node.md
    │   ├── 2_ego_replan_fsm_init.md
    │   ├── 3_ego_replan_fsm_execfsm.md
    │   ├── 4_traj_server.md
    │   ├── 5_planner_manager_rebound.md
    │   ├── 6_grid_map.md
    │   └── 7_bspline_optimizer_cost.md
    │
    ├── annotated_ego/        EGO 项目总览索引
    │   └── 0_overview.md
    │
    ├── annotated_super/      SUPER 9 篇源码精读
    │   ├── 0_overview.md
    │   ├── 1_fsm_node.md
    │   ├── 2_fsm.md                       6 状态状态机
    │   ├── 3_super_planner_main.md       SuperPlanner 类构造 / PlanFromRest / ReplanOnce
    │   ├── 4_generate_exp_traj.md        探索轨迹生成
    │   ├── 5_generate_backup_traj.md     备份轨迹与接管时刻
    │   ├── 6_corridor_ciri.md             CIRI 凸多面体走廊
    │   ├── 7_minco_trajopt.md             MINCO 7 阶轨迹优化与代价函数
    │   ├── 8_rog_map.md                   ROG-Map 占用栅格 / 膨胀 / ESDF
    │   └── 9_astar.md                     A* 前端搜索
    │
    ├── comparison.md         ⭐ 横向对比报告（15 章，一文看完两者差异）
    ├── deploy_ego_realworld.md     EGO 实机部署 + PX4 桥接
    └── deploy_super_realworld.md   SUPER 实机部署 + PX4 桥接
```

---

## 阅读路径建议

### 想了解差异，5 分钟入门
直接读 [`docs/comparison.md`](docs/comparison.md) 的「一句话摘要」和「关键算法对比」章节。

### 想精读源码
按"FSM → 主规划循环 → 优化器 → 地图 → 前端"的顺序，对照阅读：

| 模块 | EGO 文档 | SUPER 文档 |
|---|---|---|
| 程序入口 | [annotated/1](docs/annotated/1_ego_planner_node.md) | [annotated_super/1](docs/annotated_super/1_fsm_node.md) |
| 状态机 | [annotated/2](docs/annotated/2_ego_replan_fsm_init.md)、[annotated/3](docs/annotated/3_ego_replan_fsm_execfsm.md) | [annotated_super/2](docs/annotated_super/2_fsm.md) |
| 主规划循环 | [annotated/5](docs/annotated/5_planner_manager_rebound.md) | [annotated_super/3](docs/annotated_super/3_super_planner_main.md)、[4](docs/annotated_super/4_generate_exp_traj.md)、[5](docs/annotated_super/5_generate_backup_traj.md) |
| 走廊（仅 SUPER） | — | [annotated_super/6](docs/annotated_super/6_corridor_ciri.md) |
| 轨迹优化 | [annotated/7](docs/annotated/7_bspline_optimizer_cost.md) | [annotated_super/7](docs/annotated_super/7_minco_trajopt.md) |
| 地图 | [annotated/6](docs/annotated/6_grid_map.md) | [annotated_super/8](docs/annotated_super/8_rog_map.md) |
| 前端搜索 | 见 EGO 7 中 collision_rebound 节 | [annotated_super/9](docs/annotated_super/9_astar.md) |
| 控制接口 | [annotated/4](docs/annotated/4_traj_server.md) | 见 SUPER 3 中 getOneCommandFromTraj 节 |

### 想上机
直接查 [`docs/deploy_ego_realworld.md`](docs/deploy_ego_realworld.md) 或 [`docs/deploy_super_realworld.md`](docs/deploy_super_realworld.md)，里面有：
- 硬件清单、ROS 软件依赖
- 编译与多 terminal 启动顺序
- 完整输入输出话题表
- 12+ 关键参数调参表
- **完整可编译的 PX4 桥接节点 C++ 代码**（含 OFFBOARD 切换、ARM 自动化、悬停保护）

---

## 核心结论速查

### 一句话定位

| 项目 | 一句话定位 |
|---|---|
| **EGO-Planner** | "ESDF-Free 的 B-spline 局部规划器"——直接在概率占用栅格上做基于梯度的轨迹优化，碰撞反弹机制按需调 A* 找绕障路径 |
| **SUPER** | "双轨迹高速安全规划"——MINCO 7 阶多项式 + CIRI 凸多面体走廊 + 探索/备份双轨迹策略，专为未知环境 5-10 m/s 高速避障设计 |

### 关键算法差异

| 维度 | EGO | SUPER |
|---|---|---|
| 轨迹表示 | 三次均匀 B-spline | 7 阶 minimum-snap (MINCO_S4NU) |
| 时间是否优化 | ❌（仅 refine 阶段时间重分配） | ✅（每段时长是优化变量） |
| 避障方式 | 软约束 + collision rebound | 硬约束（凸多面体）+ smoothL1 软化 |
| 是否需要 ESDF | ❌（论文核心卖点） | ❌（默认关闭，预留接口） |
| 走廊 | 无 | 多段 CIRI 凸多面体 |
| 动力学约束 | 速度 + 加速度（软） | 速度 + 加速度 + jerk + 角速度 + 推力箱（软） |
| 单次规划耗时 | 5-15 ms | 30-90 ms |
| 主输出 | PVA + yaw | **PVAJ** + yaw |
| 急停机制 | 实时生成全零 B-spline | 走已 committed 的 backup_traj |
| 多机 | ✅ SEQUENTIAL_START | ❌ |

### 选型建议

| 场景 | 推荐 |
|---|---|
| 教学/入门、低速（≤3 m/s） | **EGO** |
| 仿真森林/走廊、Fast-Drone-250 实机 | **EGO** |
| 多机编队 | **EGO**（或 EGO-Swarm） |
| 深度相机 + VIO + 低算力机载机 | **EGO** |
| 高速户外 5-10 m/s | **SUPER** |
| 未知大空间探索 | **SUPER** |
| 激光雷达 + LIO | **SUPER** |
| ROS2 工程 | **SUPER**（双 ROS 支持） |
| 需要精确动力学约束 | **SUPER** |

详见 [`docs/comparison.md`](docs/comparison.md) 第 13 章。

---

## 实机部署一览

```
EGO:
RealSense ─► VINS-Fusion ─► EGO ─► traj_server ─► ego_px4_bridge ─► MAVROS ─► PX4
                  (PVA + yaw 100Hz)

SUPER:
Mid-360 LiDAR ─► FAST-LIO2 ─► ROG-Map ─► SuperPlanner ─► super_px4_bridge ─► MAVROS ─► PX4
                                                       (PVAJ + yaw 100Hz)
```

两份部署文档都给了**完整的桥接节点 C++ 源码**——直接复制粘贴到自己的 catkin 工作空间就能编译。

---

## 上游与许可证

- `ego-planner/` 来自 [ZJU-FAST-Lab/ego-planner](https://github.com/ZJU-FAST-Lab/ego-planner)，沿用其 GPL-3.0 许可证
- `SUPER/` 来自 [hku-mars/SUPER](https://github.com/hku-mars/SUPER)，沿用其 GPL-3.0 许可证
- `docs/` 下的分析文档为本仓库原创，遵循同款 GPL-3.0

如果你的研究/项目用到了这两份代码，请引用各自上游论文：

```bibtex
@article{zhou2020egoplanner,
  title={EGO-Planner: An ESDF-Free Gradient-Based Local Planner for Quadrotors},
  author={Zhou, Xin and Wang, Zhepei and Ye, Hongkai and Xu, Chao and Gao, Fei},
  journal={IEEE Robotics and Automation Letters},
  year={2020}
}

@article{ren2025super,
  title={Safety-assured high-speed navigation for MAVs},
  author={Ren, Yunfan and others},
  journal={Science Robotics},
  year={2025}
}
```

---

## 反馈

源码分析有错漏、参数有更优配置、桥接代码上机有 bug，欢迎开 issue 或 PR。
