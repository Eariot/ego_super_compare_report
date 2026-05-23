# super_planner.cpp（三）—— generateBackupTrajectory 备份轨迹

## 文件路径
`super_planner/src/super_core/super_planner.cpp` L828-1085

## 整体设计

备份轨迹（Backup Trajectory）是 SUPER 与 EGO 最核心的差异化设计。它解决一个具体问题：

> **当 exp 轨迹延伸进未知/不可见区域时，如果飞机一路按 exp 轨迹飞过去，可能会撞上还没观测到的障碍物。** 因此 SUPER 在每次 replan 时同时生成一条"刹车轨迹"——它从某个"接管时刻"（`opt_ts`）开始，让飞机减速并停在已知自由空间中。当感知更新跟得上时，下次 replan 把接管时刻顺延；跟不上时飞机自然走 backup 而不会失控。

## 核心数据流

```
generateBackupTrajectory(ref_exp_traj)
    │
    ├── 1) 沿 exp 采样：找到第一个"line-of-sight"失效点 invisible_p
    │       (机体到该点的连线在地图上撞障)
    │
    ├── 2) 沿 exp 往回退一小段（一个 robot_r）→ seed_point
    │       这是接管时刻的"目标位置"——backup 轨迹要在此停下
    │
    ├── 3) CorridorGenerator::GeneratePolytopeFromLine
    │       从 robot 当前位置到 seed_point 拉一条线 → CIRI 膨胀成单个凸多面体
    │       （备份轨迹只有 1 个 SFC，所以叫 single-corridor backup）
    │
    ├── 4) 可选剪裁：FOV / sensing_horizon
    │       cutPolyByFov、cutPolyBySensingHorizon
    │
    ├── 5) 估算"接管时刻"启发式 heu_ts
    │       heu_ts = max((t0 + te) / 2, te - vel_e / max_acc)
    │       即"中点 vs 反推刹车起点"二选一靠后那个
    │
    └── 6) BackupTrajOpt::optimize
              输入：ref_exp_traj、t0、te、heu_ts、heu_p、heu_dur、SFC
              输出：temp_pos_traj（多段 MINCO）+ opt_ts（被优化器调整后的接管时刻）
```

> 关键创新：`opt_ts` 在 BackupTrajOpt 内部被当作可微优化变量——当物理可行性更好的接管时刻能让代价更低，优化器会把它从 `heu_ts` 调到 `opt_ts`。这是 SUPER backup 的"自适应接管时刻"特色。

---

## 阶段 1：早退检查 + 沿 exp 找接管区段（L828-895）

```cpp
RET_CODE SuperPlanner::generateBackupTrajectory(ExpTraj &ref_exp_traj,
                                                BackupTraj &back_traj_info) {
    drone_state_mutex_.lock();
    back_traj_info.setRobotPos(robot_state_.p);   // 当前机体位置
    drone_state_mutex_.unlock();

    TimeConsuming t_back_frontend("t_back_frontend", false);

    double total_dur = ref_exp_traj.getTotalDuration();
    double start_t = ros_ptr_->getSimTime() - ref_exp_traj.getStartWallTime();

    /* ---- 早退 1：exp 已经飞完 ---- */
    if (start_t > total_dur - 0.01) return NO_NEED;

    Vec3f temp_point;
    double out_t;
    bool all_traj_visible{true};
    vector<double> min_stop_dis;       // 每个采样点理论最短刹车距离 v²/(2a)
    vector<TimePosPair> eval_ps;       // 已采样的 (TT, pos) 对
    Vec3f temp_vel;

    /* ---- 沿 exp 向前采样，记录可见性 ---- */
    Vec3f last_pos = ref_exp_traj.getPos(start_t);
    for (out_t = start_t; out_t < total_dur; out_t += cfg_.sample_traj_dt) {
        temp_point = ref_exp_traj.getPos(out_t);
        if ((last_pos - temp_point).norm() < cfg_.resolution * 0.8) continue;

        last_pos = temp_point;
        temp_vel = ref_exp_traj.getVel(out_t);
        double v_norm = temp_vel.norm();
        min_stop_dis.push_back(v_norm * v_norm / 2.0 / cfg_.exp_traj_cfg.max_acc);
        eval_ps.push_back({out_t, temp_point});

        // 检查 robot 到 temp_point 的连线是否在 prob_map 上"自由"
        // min_dis：最大允许检查长度（min(sensing_horizon, safe_corridor_line_max_length)）
        const double min_dis = cfg_.sensing_horizon > 0
                ? std::min(cfg_.sensing_horizon, cfg_.safe_corridor_line_max_length)
                : cfg_.safe_corridor_line_max_length;
        if (!map_ptr_->isLineFree(back_traj_info.getRobotPos(),
                                   temp_point,
                                   min_dis,
                                   cfg_.seed_line_neighbour)) {
            all_traj_visible = false;
            break;        // 找到第一个不可见点 → 跳出
        }
    }

    /* ---- 早退 2：整段 exp 都"可见"（=确认安全） ---- */
    if (all_traj_visible) {
        back_traj_info.setEmpty();
        // 还是为可视化生成一个 SFC（围住 exp 末端）
        Vec3f seed_pt = ref_exp_traj.getPos(ref_exp_traj.getTotalDuration());
        Line line{back_traj_info.getRobotPos(), seed_pt};
        Polytope temp_poly;
        if (cg_ptr_->GeneratePolytopeFromLine(line, temp_poly)) {
            back_traj_info.setSFC(temp_poly);
            ros_ptr_->vizBackupSfc(temp_poly);
        }
        return FINISH;     // 不需要 backup
    }
```

> "可见性"在 SUPER 里 = `isLineFree`（机体到目标点连线 + 半径邻域不撞障）。如果飞机离 exp 末端 5m，但中间所有点都能"看到"（连线无障），则 `all_traj_visible = true`。

---

## 阶段 2：找 seed_point（L895-905）

```cpp
Vec3f invisible_p = eval_ps.back().second;     // 第一个不可见点

// 从 invisible_p 往回退，直到距离它超过一个 robot_r
// 这样 seed_point 就在"安全可见区"的最末端
while (out_t > start_t) {
    out_t -= cfg_.sample_traj_dt;
    Vec3f out_p = ref_exp_traj.getPos(out_t);
    if ((out_p - invisible_p).norm() > cfg_.robot_r) break;
}

double seed_point_t = std::max(start_t, out_t);
Vec3f seed_point = ref_exp_traj.getPos(seed_point_t);
```

> 退一个 `robot_r`（机体半径）的目的是给 backup 留点缓冲——直接在不可见点制动会有撞墙风险。

---

## 阶段 3：生成 backup SFC（L919-955）

```cpp
// 把机体起点 shift 到自由空间（同 PlanFromRest 的处理）
// 优先使用上一帧 exp 计算时存的 shifted_sfc_start_pt_
Vec3f shifted_robot_p = shifted_sfc_start_pt_.norm() > 999
                            ? robot_state_.p
                            : shifted_sfc_start_pt_;
if (!map_ptr_->getNearestCellNot(GridType::OCCUPIED, shifted_robot_p, shifted_robot_p, 3.0)) {
    return FAILED;
}

// CIRI: 从 [robot_pos → seed_point] 这条线段膨胀出一个凸多面体
Line line{shifted_robot_p, seed_point};
Polytope temp_poly;
if (!cg_ptr_->GeneratePolytopeFromLine(line, temp_poly)) return FAILED;

// 检查多面体内部点是否存在
Eigen::Vector3d inner;
if (!geometry_utils::findInterior(temp_poly.GetPlanes(), inner)) {
    return FAILED;     // 退化多面体，规划失败
}

// 可选：FOV 剪裁（让 backup SFC 不超出相机视场）
if (cfg_.use_fov_cut) {
    if (!fov_checker_->cutPolyByFov(robot_state_.p, robot_state_.q, seed_point, temp_poly))
        return FAILED;
}

// 可选：传感距离剪裁（避免使用未观测远区域）
if (cfg_.sensing_horizon > 0 &&
    !fov_checker_->cutPolyBySensingHorizon(robot_state_.p, seed_point,
                                            cfg_.sensing_horizon, temp_poly)) {
    return FAILED;
}

back_traj_info.setSFC(temp_poly);
ros_ptr_->vizBackupSfc(temp_poly);
```

---

## 阶段 4：扩展接管时刻区间（L967-985）

backup_traj 不一定要正好停在 seed_point；只要落在 SFC 内部就算合法。所以下面这段把 `eval_ps` 继续往前延伸——只要接下来的 exp 点仍然在 SFC 内，就把它作为候选 seed_point。

```cpp
double eval_t = eval_ps.back().first + cfg_.sample_traj_dt;
last_pos = eval_ps.back().second;
while (temp_poly.PointIsInside(eval_ps.back().second) && eval_t < total_dur) {
    Vec3f cur_pos = ref_exp_traj.getPos(eval_t);
    if ((cur_pos - last_pos).norm() < cfg_.resolution * 0.8) {
        eval_t += cfg_.sample_traj_dt;
        continue;
    }
    temp_vel = ref_exp_traj.getVel(out_t);
    double v_norm = temp_vel.norm();
    min_stop_dis.push_back(v_norm * v_norm / 2.0 / cfg_.exp_traj_cfg.max_acc);
    eval_ps.emplace_back(eval_t, cur_pos);
    last_pos = cur_pos;
    eval_t += cfg_.sample_traj_dt;
}
eval_ps.pop_back();
seed_point = eval_ps.back().second;
seed_point_t = eval_ps.back().first;
```

---

## 阶段 5：启发式接管时刻（L989-998）

```cpp
double t0 = ros_ptr_->getSimTime() - ref_exp_traj.getStartWallTime() + 0.01;
double te = seed_point_t;
double vel_e_n = ref_exp_traj.getVel(te).norm();
// 启发式：取"中点"和"end - 刹车时间"两者的较大值
// "刹车时间" = vel_e / max_acc（用最大减速度从 vel_e 减速到 0 需要的时间）
double heu_ts = std::max((t0 + te) / 2, te - vel_e_n / cfg_.back_traj_cfg.max_acc);
double heu_dur = te - heu_ts;
Vec3f heu_p = seed_point;
```

直观解释：
- `(t0+te)/2`：把 backup 接管时刻设在中点（保守）；
- `te - vel_e/max_acc`：在"恰好够刹住"的时刻接管（激进）；
- 取较大值意味着"如果激进刹车不够空间，那么提早接管"。

---

## 阶段 6：MINCO 优化（L1000-1037）

```cpp
TimeConsuming t_back_opt("t_back_opt", false);
double opt_ts = heu_ts;
Trajectory temp_pos_traj;
auto sfc0 = back_traj_info.getSFC();

bool temp_ret = back_traj_opt_->optimize(
        ref_exp_traj.posTraj(),     // 参考的 exp 轨迹（用于追踪 attractor）
        t0,                          // 当前时刻在 exp 上的偏移
        te,                          // 接管时刻最晚不超过 te
        heu_ts,                      // 接管时刻初值
        heu_p,                       // backup 终点位置
        heu_dur,                     // backup 持续时间
        back_traj_info.getSFC(),     // SFC（凸多面体约束）
        temp_pos_traj,               // [输出] 优化后的多段 MINCO 轨迹
        opt_ts                       // [输出] 被优化的接管时刻
);
time_consuming_[BACK_TRAJ_OPT] = t_back_opt.stop();
```

> 优化变量除了控制点和段时间，还包括 `ts`（接管时刻）。具体的代价函数（位置/速度/加速度/jerk/姿态/推力 + 与 exp 的 attractor）见 `7_minco_trajopt.md`。

```cpp
if (!temp_ret) {
    back_traj_info.setEmpty();
    return OPT_FAILED;
}

/* ---- yaw 优化 + 拼接 ---- */
Vec4f yaw_init_vec = ref_exp_traj.getYawState(opt_ts).row(0);
Vec4f yaw_goal{0, 0, 0, 0};
bool free_end{true};
if (cfg_.goal_yaw_en && !isnan(gi_.goal_yaw)) {
    free_end = false;
    yaw_goal[0] = gi_.goal_yaw;
}
Trajectory temp_yaw_traj;
if (!yaw_traj_opt_->optimize(yaw_init_vec, yaw_goal, temp_pos_traj,
                              temp_yaw_traj, 3, false, free_end)) {
    return OPT_FAILED;
}

/* ---- 一致性检查 ---- */
// 接管时刻在新轨迹中不能比当前 cmd_traj 已 committed 的接管时刻早
double new_ts_WT = ref_exp_traj.getStartWallTime() + opt_ts;
const auto &committed_ts_WT = cmd_traj_info_.getBackupTrajStartTT();
if (committed_ts_WT < cmd_traj_info_.getTotalDuration() && new_ts_WT < committed_ts_WT) {
    return OPT_FAILED;       // 时间倒退 → 拒绝
}

/* ---- 设置 BackupTraj ---- */
back_traj_info.setTrajectory(new_ts_WT, opt_ts, temp_pos_traj, temp_yaw_traj);
return SUCCESS;
```

---

## 拼接到 cmd_traj 的语义

```
cmd_traj 时间轴（从 cmd_start_WT 开始）：

|<-- exp 头段（已 committed，不能改）-->|<-- exp 新段 -->|<-- backup -->|
0                                  replan_state_TT      backup_start_TT  total_dur
                                                          ^
                                                          opt_ts
```

飞控读取 `cmd_traj.getState(eval_t)` 时：
- `eval_t < backup_start_TT`：在 exp 上，按 exp 多项式插值；
- `eval_t >= backup_start_TT`：自动切换到 backup 多项式。

完全无缝，因为 `BackupTrajOpt` 在 `opt_ts` 处的 P/V/A/J 都被 attractor 项约束到 = exp 轨迹在 `opt_ts` 处的 P/V/A/J。

---

## 状态机的"急停"如何依赖 backup

回顾 FSM 状态机（见 `2_fsm.md`）：当 ReplanOnce 返回 EMER 时，FSM 直接切到 EMER_STOP → 然后回 WAIT_GOAL，等用户重点。**FSM 不会再发任何控制指令**——但飞机仍然有运动学指令，因为：

1. 上一次 replan 已 committed 的 `cmd_traj_info_` 还在执行；
2. 该 cmd_traj 的尾部就是 backup_traj，会自动让飞机减速到悬停；
3. 用户重点之后，新的 PlanFromRest 从悬停状态接管。

这是 SUPER 的"安全闭环"：

> **飞机进入 backup_traj 时，必然不会撞上未知区域**——因为 backup 的 SFC 完全在已知自由空间内。

EGO 没有这个机制：当 EGO 触发 EMERGENCY_STOP 时，它必须 **当场** 生成一段全零控制点的 B-spline 急停轨迹（`callEmergencyStop`），如果当前速度太大或当前位置离障碍物太近，这一急停可能会失败。

---

## 时间预算分配

`replan_forward_dt = 0.3 s` 是 SUPER 的一个核心参数。它表示飞控提前消费当前 cmd_traj 多久。各个模块在这 0.3s 中分配：

| 模块 | 典型耗时 |
|---|---|
| EXP 前端（A* + SFC） | 5-15 ms |
| EXP 优化（MINCO） | 10-40 ms |
| BACKUP 前端（line search + SFC） | 3-8 ms |
| BACKUP 优化（MINCO） | 5-20 ms |
| Yaw 优化 | 1-3 ms |
| 可视化 | 2-5 ms |
| **总计** | **~30-90 ms** ≪ 300 ms |

剩下的 ~210 ms 是飞控真正消费当前 cmd_traj 的时间，保证轨迹平滑切换。
