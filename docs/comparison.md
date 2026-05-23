# EGO-Planner 与 SUPER 对比分析报告

> 范围：仅基于 `compare/ego-planner/`（ZJU-FAST-Lab/ego-planner）与 `compare/SUPER/`（HKU-MARS/SUPER）两个仓库的源代码。两者都是开源四旋翼运动规划框架，但定位、设计哲学、算法栈差异显著。

---

## 1. 一句话摘要

| 项目 | 一句话定位 |
|---|---|
| **EGO-Planner**（2020, RA-L） | "**ESDF-Free** 的 B-spline 局部规划器"——直接在概率占用栅格上做基于梯度的轨迹优化，碰撞反弹（collision rebound）机制按需调 A* 找绕障路径。 |
| **SUPER**（2025, Sci. Robotics） | "**双轨迹高速安全规划**"——MINCO 7 阶多项式 + 凸多面体走廊 + 探索/备份双轨迹策略，专门针对未知环境 5-10 m/s 高速避障。 |

---

## 2. 顶层架构差异

### EGO-Planner

```
ego_planner_node
       └─ EGOReplanFSM（7 状态）
              ├─ Timer 100Hz execFSMCallback —— 状态机切换
              └─ Timer 20Hz checkCollisionCallback —— 安全监视

EGOReplanFSM::callReboundReplan
       └─ EGOPlannerManager::reboundReplan
              ├─ STEP 1 INIT：多项式初始路径 → B-spline 控制点
              ├─ STEP 2 OPT：BsplineOptimizeTrajRebound（L-BFGS）
              │      cost = λ1·smooth + λ2·distance + λ3·feasibility + λ2·swarm + λ2·terminal
              │      迭代后期触发 check_collision_and_rebound（A* 找绕障 → 更新 base_point/direction）
              └─ STEP 3 REFINE：refineTrajAlgo（fitness cost 替代 distance cost）

输出：traj_utils::Bspline → traj_server → /position_cmd
```

### SUPER

```
fsm_node_ros1
       └─ FsmRos1（基类 fsm::Fsm，6 状态）
              ├─ Timer callMainFsmOnce —— 状态机切换
              └─ Timer callReplanOnce —— 飞行中重规划（在 FOLLOW_TRAJ 持续触发）

SuperPlanner::PlanFromRest / ReplanOnce
       ├─ generateExpTraj
       │     ├─ A* PathSearch（inf_map + prob_map 双视图）
       │     ├─ CorridorGenerator::SearchPolytopeOnPath（多段 SFC，CIRI 凸多面体）
       │     └─ ExpTrajOpt::optimize（MINCO + L-BFGS）
       │             cost = energy + pos + vel + acc + jer + attractor + omg + thrust
       │
       └─ generateBackupTrajectory
             ├─ 沿 exp 找首个 line-of-sight 失效点 → seed_point
             ├─ CorridorGenerator::GeneratePolytopeFromLine（单段 SFC）
             └─ BackupTrajOpt::optimize（MINCO + 接管时刻 ts 自由度）

输出：CmdTraj（exp + backup 拼接） → MPC 控制器
```

---

## 3. 关键算法对比

| 维度 | EGO-Planner | SUPER |
|---|---|---|
| **轨迹表示** | 三次均匀 B-spline（cubic uniform） | 7 阶 minimum-snap（MINCO_S4NU） |
| **控制变量** | 控制点（30-100 个，3D） | 段路点 + 段时长（5-30 段，含 ts 自由度） |
| **时间是否优化** | 否（仅 refine 阶段做时间重分配） | 是（每段时长是优化变量） |
| **避障方式** | 软约束 + collision rebound（梯度方向由 A* 给出） | 硬约束（轨迹必须在凸多面体内）+ smoothL1 软化 |
| **是否需要 ESDF** | **否**（ESDF-Free，论文核心卖点） | 否（SUPER 默认关闭 ESDF；ESDF 是预留接口） |
| **走廊** | 无（直接在控制点上算避障梯度） | 多段凸多面体 SFC（CIRI 算法） |
| **动力学约束** | 速度 + 加速度（软约束） | 速度 + 加速度 + jerk + 角速度 + 推力箱（软约束） |
| **优化器** | L-BFGS（Lewis-Overton） | L-BFGS（同款 lbfgs.h） |
| **求解器单次耗时** | 5-15 ms | 10-40 ms（exp） + 5-20 ms（backup） |
| **重规划频率** | ~2.5 Hz（每 0.4s 一次） | ~3.3 Hz（replan_forward_dt = 0.3s） |

---

## 4. 地图模块对比

| 维度 | EGO grid_map | SUPER ROG-Map |
|---|---|---|
| **概率更新** | logit 累加（hit/miss） | logit 累加（hit/miss） |
| **输入数据** | 深度图 + Pose / 点云 | 点云 + Pose |
| **滑动窗** | 一次性清边界（O(N)） | SlidingMap 整体平移（O(平移量)） |
| **膨胀** | 单层 buffer（每帧重新做） | 独立 InfMap（增量更新） |
| **ESDF** | **不支持**（核心设计哲学） | 可选（默认关闭，留作扩展接口） |
| **前沿提取** | 不支持 | 可选（FreeCntMap） |
| **多视图查询** | `getInflateOccupancy` | `isOccupied / isOccupiedInflate / isLineFree / getNearestCellNot` |
| **线段碰撞** | 沿轨迹采样查询 | `isLineFree` + 球形邻域（自带机体半径检查） |
| **代码规模** | 约 1000 行 | 约 2000 行（多个子地图） |

---

## 5. 状态机与失败容错

### EGO 7 状态

```
INIT → WAIT_TARGET → SEQUENTIAL_START（多机）/ GEN_NEW_TRAJ
   ↓                           ↓
   EMERGENCY_STOP ← REPLAN_TRAJ ⇄ EXEC_TRAJ
```

### SUPER 6 状态

```
INIT → WAIT_GOAL → GENERATE_TRAJ → FOLLOW_TRAJ ⇄ EMER_STOP
                                       (callReplanOnce 在此持续触发)
```

| 项目 | EGO | SUPER |
|---|---|---|
| 重规划是否切状态 | 是（EXEC_TRAJ ↔ REPLAN_TRAJ 来回切） | 否（FOLLOW_TRAJ 持续状态，独立 Timer 触发） |
| 急停轨迹 | callEmergencyStop 实时生成全零控制点 B-spline | 直接走已 committed 的 backup_traj |
| 失败响应 | 立即重试 + 必要时急停 | 留旧 cmd_traj 继续飞，靠 backup 兜底 |
| 多机协调 | SEQUENTIAL_START + 广播 Bspline | **不支持**多机 |
| 安全检查频率 | 独立 20Hz checkCollisionCallback | 由 ReplanOnce 内部对当前 cmd_traj 做检查 |
| Goal 处理 | RViz 2D Nav Goal z=1.0 硬编码 | 用 click_height + getNearestInfCellNot 自动避开占据格 |

> SUPER 的最大设计差异：**安全闭环依赖 backup_traj 而非急停**。一旦 ReplanOnce 失败，飞机仍然在已 committed 的 cmd_traj 上飞，cmd_traj 的尾部就是 backup_traj，会自动让飞机减速到悬停在已知自由空间内。EGO 没有这个保险机制——ReplanOnce 失败时只能依赖 EMERGENCY_STOP 重新生成全零急停轨迹。

---

## 6. 双轨迹（Backup Trajectory）—— SUPER 独有

> 这是 SUPER 区别于 EGO 最核心的设计点。

**问题**：当 exp 轨迹延伸进未知/不可见区域时，飞机如果一路按 exp 飞过去，可能撞上未观测障碍。

**SUPER 的解法**（在每次 replan 时同时生成）：
- **Exp Trajectory**：从当前状态飞向全局目标，可以延伸进未知区域；
- **Backup Trajectory**：从某个"接管时刻"`opt_ts` 开始，让飞机在已知自由空间内减速到悬停；
- 飞控按 cmd_traj.getState(t) 自动从 exp 切到 backup；
- 下次 replan 把接管时刻顺延，感知跟得上时正常飞，跟不上时自动刹车。

**关键创新**：`opt_ts`（接管时刻）是 BackupTrajOpt 的优化变量之一，会被自适应调整。

EGO 没有这个机制——它依赖前端 A* 总是能搜到无碰路径，依赖 collision rebound 总能修正避障梯度。当感知有延迟或地图存在死区时容易出现失稳。

---

## 7. 前端搜索

### EGO `dyn_a_star`
- 仅在 `check_collision_and_rebound` 触发时按需调用
- 搜索范围小（2-3 m，绕一个障碍物）
- 26 邻居，无球形线段检查
- 启发式：欧氏距离

### SUPER `astar`
- 每次 generateExpTraj 都调用一次
- 搜索范围 = planning_horizon（10-20 m）
- 双视图：先 inf_map 快速搜，失败回退 prob_map
- Unknown 处理可配置：UNKNOWN_AS_OCCUPIED（保守）/ UNKNOWN_AS_FREE（依赖 backup 兜底）
- 球形邻域 + 起点逃逸（escapePathSearch）
- 启发式：DIAG / MANH / EUCL 三选一
- 时间预算控制（time_out 参数）

---

## 8. 代价函数对比

### EGO `combineCostRebound`（5 项）
```
total = λ1·smooth          (jerk² 平滑)
      + λ2·distance         (碰撞反弹 + 三次/二次惩罚)
      + λ3·feasibility       (速度/加速度软约束)
      + λ2·swarm            (多机距离)
      + λ2·terminal         (终点收敛到 local_target_pt)
```

`refine` 阶段把 distance 替换为 fitness（贴合参考路径）。

### SUPER `costFunctional`（7+1 项）
```
total = energy             (∫|jerk|² dt 闭式能量，由 MINCO 给出)
      + ρ_T · ΣT             (总时间长度)
      + λ_pos·smoothL1(SFC violation)  (位置必须在多面体内)
      + λ_att·smoothL1(attractor violation)  (路点吸引)
      + λ_vel/acc/jer·smoothL1(动力学违规)
      + λ_omg·smoothL1(|ω|² - ωmax²)
      + λ_thr·smoothL1((thr-mean)² - radi²)
```

通过 `flatness::FlatnessMap` 把（vel, acc, jer, ψ）映射到（thrust, quat, ω），让"姿态/推力约束"也能直接对位置高阶导写出梯度。

---

## 9. 性能与适用场景

| 维度 | EGO | SUPER |
|---|---|---|
| **设计速度** | 1-3 m/s（常用） | 5-10 m/s（论文实测 18 m/s） |
| **典型 replan 总耗时** | 10-30 ms | 30-90 ms |
| **典型环境** | 仿真森林、走廊、Fast-Drone-250 实机 | 高速户外、复杂室内、未知大空间 |
| **传感器** | 深度相机（VINS）/ 激光 | 激光雷达（FAST-LIO） |
| **对感知延迟容忍度** | 中（依赖 EMERGENCY_STOP 兜底） | 高（依赖 backup_traj，能在感知死区下安全减速） |
| **多机** | 支持（SEQUENTIAL_START + Bspline 广播） | 不支持 |
| **代码量** | ~3500 行（plan_manage + bspline_opt + plan_env） | ~7000 行（super_planner + rog_map） |
| **依赖** | Eigen + nlopt-like lbfgs | Eigen + LBFGS + cereal + fmt + backward-cpp + flatness |
| **ROS 兼容** | 仅 ROS1 | ROS1 / ROS2 双支持（通过 ros_interface 适配层） |

---

## 10. 代码风格 / 工程化对比

| 维度 | EGO | SUPER |
|---|---|---|
| **入口风格** | `ros::spin()` 单线程 | `ros::AsyncSpinner(0)` 多线程 |
| **配置加载** | `nh.param("...", ...)` 参数服务器 | YAML 文件（cereal-yaml-loader） |
| **错误处理** | 主要靠 ROS_ERROR + 状态机重试 | 详细 RET_CODE 枚举 + backward-cpp 崩溃回溯 |
| **日志** | std::cout | fmt::print + 文件 CSV log（耗时分模块统计） |
| **抽象层** | ROS 紧耦合 | RosInterface 抽象（同份代码跑 ROS1/ROS2） |
| **可视化** | PlanningVisualization 直接发 Marker | RosInterface::vizXxx 系列方法（11 种轨迹/SFC/路径） |
| **调试工具** | 仅 RViz | Apps/read_replan_log + traj_opt_tuning + LogOneReplan 序列化 |
| **测试** | 仿真 launch | 自带 mars_uav_sim（PerfectDroneSim + MARS LiDAR 渲染） |

---

## 11. 数学形式对照

### B-spline vs MINCO 多项式

EGO（cubic B-spline）：
```
轨迹：r(t) = Σ q_i · B_{i,3}(t)              （控制点 q_i）
速度：v(t) = (q_{i+1} - q_i) / Δt
加速度：a(t) = (q_{i+2} - 2q_{i+1} + q_i) / Δt²
jerk：j(t) = (q_{i+3} - 3q_{i+2} + 3q_{i+1} - q_i) / Δt³
```

SUPER（7 阶 minimum-snap，每段独立）：
```
段内：r_i(t) = Σ_{k=0..7} c_{i,k} · t^k       （8×3 系数矩阵 c_i）
段间连续性：P/V/A/J 直至 4 阶导都连续
能量：∫(d⁴r/dt⁴)² dt → 闭式（minimum-snap）
```

MINCO 的关键性质：给定段时间 T 和段间路点 q，**c 由 banded 线性方程组在 O(N) 内闭式求解**——这是它能高效优化的基础。

### 避障约束

EGO（软约束）：
```
对每个控制点 q_i，对每个障碍物 j，定义：
  dist_ij = (q_i - base_point[i][j]) · direction[i][j]
  err = clearance - dist_ij
  cost = {
    0,                                if err < 0
    err³,                              if 0 ≤ err < demarcation
    a·err² + b·err + c,               if err ≥ demarcation
  }
```
其中 `base_point` 和 `direction` 由 collision rebound 中的 A* 搜索得到。

SUPER（硬约束 + smoothL1 软化）：
```
对轨迹每个采样点 r(t)，对所属 SFC 的每个半空间 (n_k, d_k)：
  viola = n_k · r(t) + d_k        （>0 表示违规）
  cost = smoothL1(viola, μ)        （μ = smooth_eps）
```

---

## 12. 各自的优劣势

### EGO 的优势
1. **无 ESDF**：地图更新极轻，深度相机 + VINS 即可，硬件门槛低；
2. **代码简洁**：~3500 行，状态机直观，二开门槛低；
3. **多机支持**：SEQUENTIAL_START + Bspline 广播，是 EGO-Swarm 的基础；
4. **实时性好**：单次规划 10-30 ms，2.5 Hz 重规划完全够用；
5. **被验证的可靠**：Fast-Drone-250 已成为高校实机基线。

### EGO 的劣势
1. **B-spline 阶数低**：cubic 限制了"完整 PVAJ + 姿态/推力约束"的施加，对高动态飞行能力有限；
2. **避障是软约束**：collision rebound 需要 A* 介入，复杂场景下迭代次数多；
3. **失败兜底依赖急停**：感知延迟时容易撞墙；
4. **不优化时间**：refine 阶段才做时间重分配，整体最优性较弱；
5. **未知空间处理不显式**：把 unknown 视为 free，没有专门保护机制。

### SUPER 的优势
1. **双轨迹策略**：感知失效或 replan 失败时有 backup 自动接管，安全裕度高；
2. **MINCO 高阶**：能精确施加 jerk/姿态/推力约束，支持 5-10 m/s 高速飞行；
3. **凸走廊硬约束**：避障由凸约束保证，优化迭代收敛快；
4. **ROG-Map 工程化**：增量膨胀 + 滑动窗 + ESDF 可选，扩展性好；
5. **ROS1/ROS2 双支持**：抽象层做得彻底；
6. **未知/已知两种语义**：A* 可显式选择 UNKNOWN_AS_OCCUPIED 或 _FREE。

### SUPER 的劣势
1. **代码量大、依赖重**：~7000 行 + cereal + fmt + flatness + backward-cpp，编译/二开成本高；
2. **不支持多机**：纯单机，没有 swarm 协调机制；
3. **CIRI 走廊计算贵**：单次 SFC 生成 5-15 ms，对低速场景是浪费；
4. **MINCO 优化耗时较高**：30-90 ms 单帧，对 10Hz 控制频率紧；
5. **目标只支持 click_height 锁定**：与 EGO 的 PRESET_TARGET 多航点列表相比少功能；
6. **MPC 配套**：cmd 输出包含 jerk，要求下游必须用 NLMPC（不能直接给 PX4 PositionTarget）。

---

## 13. 选型建议

| 场景 | 推荐 | 原因 |
|---|---|---|
| 教学/入门 | **EGO** | 代码量少、状态机直观、Fast-Drone-250 成熟 |
| 仿真森林/走廊 1-3 m/s | **EGO** | 速度域内 EGO 完全够用，且二开容易 |
| 多机编队 | **EGO**（或 EGO-Swarm） | SUPER 不支持多机 |
| 实机深度相机 + 低算力 | **EGO** | 不需要 ESDF，深度图直接用 |
| 高速户外 5-10 m/s | **SUPER** | MINCO + backup 是为高速设计的 |
| 未知大空间探索 | **SUPER** | UNKNOWN_AS_FREE + backup_traj 安全闭环 |
| 激光雷达 + FAST-LIO | **SUPER** | ROG-Map 原生支持点云，ESDF 可选 |
| ROS2 工程 | **SUPER** | EGO 是 ROS1-only |
| 需要精确动力学约束 | **SUPER** | jerk + 角速度 + 推力箱完整约束 |

---

## 14. 设计哲学的根本差异

EGO 和 SUPER 都是 HKU/ZJU 系一脉相承的产物，但选择了相反的工程取舍：

- **EGO**：减法。砍掉 ESDF、砍掉走廊、砍掉复杂动力学、砍掉双轨迹。剩下"概率栅格 + B-spline + 5 个 cost"，简单到极致。换来低门槛和高重用性。
- **SUPER**：加法。MINCO + CIRI + ROG-Map + 双轨迹 + ROS1/ROS2 抽象 + flatness + ESDF 可选 + 完整日志。每个模块都为"高速 + 未知 + 安全"这一目标服务。换来高速极限性能。

> 一个朴素的判断：**如果你的飞机不会超过 3 m/s，EGO 是完美的；如果你必须 8 m/s 在未知空间高速穿梭，SUPER 才是对的工具**。

---

## 15. 进一步阅读

| 想了解什么 | EGO 文档 | SUPER 文档 |
|---|---|---|
| 状态机 | `annotated/2_ego_replan_fsm_init.md`、`annotated/3_ego_replan_fsm_execfsm.md` | `annotated_super/2_fsm.md` |
| 主规划循环 | `annotated/5_planner_manager_rebound.md` | `annotated_super/3_super_planner_main.md`、`4_generate_exp_traj.md`、`5_generate_backup_traj.md` |
| 代价函数 | `annotated/7_bspline_optimizer_cost.md` | `annotated_super/7_minco_trajopt.md` |
| 地图 | `annotated/6_grid_map.md` | `annotated_super/8_rog_map.md` |
| 走廊（仅 SUPER） | — | `annotated_super/6_corridor_ciri.md` |
| 前端搜索 | `annotated/7_bspline_optimizer_cost.md` 中 collision_rebound 节 | `annotated_super/9_astar.md` |
| 控制接口 | `annotated/4_traj_server.md` | `annotated_super/3_super_planner_main.md` 中 getOneCommandFromTraj 节 |
