# super_planner.cpp（二）—— generateExpTraj 探索轨迹生成

## 文件路径
`super_planner/src/super_core/super_planner.cpp` L379-826

## 整体设计

`generateExpTraj` 是 SUPER 最长（约 450 行）也最复杂的函数。它负责把"上一段轨迹 + 新目标"翻译为一条新的探索轨迹，分七步：

```
1. 入口与参数声明
2. 处理 last_exp_traj 是否为空
   2a. 空 → 从静止点出发（PlanFromRest 分支）
   2b. 非空 → 沿当前 cmd_traj 前向 replan_forward_dt 取状态作为新起点
              + 收集"剩余有效轨迹片段"作为热启动的 guide_path 部分
3. 用 A* 把 guide_path 末端搜到目标点（PathSearch）
4. 时间分配：给 guide_path 每个点打上时间戳
5. CorridorGenerator::SearchPolytopeOnPath → 多段 SFC
6. ExpTrajOpt::optimize → MINCO + L-BFGS
7. 拼接 [当前到 replan_state 的旧轨迹 + 优化得到的新轨迹]
   + YawTrajOpt::optimize（独立 yaw）
```

> SUPER 的核心思想：**轨迹的前 `replan_forward_dt` 秒不动**（让飞控有时间消费），新轨迹只从 `replan_state_TT` 时刻开始拼接进来。这与 EGO 的 `planFromCurrentTraj`（取当前时刻状态作为起点）有本质差别——EGO 直接覆盖正在跑的轨迹，SUPER 在飞控的"消费窗口"之外才覆盖。

---

## 关键变量

```cpp
// 时间戳的两种坐标系：
// WT (Wall Time) = ROS 系统时间，全局一致
// TT (Trajectory Time) = 相对于轨迹起点（start_WT）的偏移量
// 二者关系：WT = start_WT + TT

double replan_process_start_WT;     // 这次 replan 启动时的 ROS 时间
double replan_process_start_TT;     // 启动时刻在旧轨迹中的相对时间
double replan_state_TT;             // 实际作为新轨迹起点的时间（= start_TT + replan_forward_dt）

StatePVAJ pos_init_state, pos_fina_state;  // 4×3：起点/终点的 P/V/A/J
PolytopeVec sfc;                            // SFC 数组
vec_Vec3f guide_path;                       // 引导路径（若干 3D 点）
vector<double> guide_stamp;                 // 引导路径的时间戳（相对值）
double guide_path_end_vel;                  // 引导路径末端的速度
```

---

## 阶段 1：last_exp_traj 为空（PlanFromRest 分支）

```cpp
// L406-412
if (last_exp_traj_info.empty()) {
    pos_init_state.setZero();           // P=local_start_p, V=A=J=0
    pos_init_state.col(0) = local_start_p_;
    replan_process_start_TT = -1;
    replan_state_TT = -1;
}
```

起点取的是 PlanFromRest 中已经 shift 过的 `local_start_p_`，速度/加速度/jerk 全部为零。

---

## 阶段 2：非空时的早退条件（L413-493）

热启动时 SUPER 做了大量"看是否完全不需要重规划"的检查，能够省掉一次完整的优化。

```cpp
// L425-442：如果 replan_state_TT 已经超过当前 cmd 的总时长 → 等飞机自然飞完
if (replan_state_TT >= cmd_traj_info_.getTotalDuration()) {
    out_exp_traj_info = last_exp_traj_info;
    if (robot_on_backup_traj_) return FAILED;  // 已在 backup 上还想等？不行
    return NO_NEED;
}

// L444-458：替代检查（针对 last_exp_traj 而非 cmd_traj）
// L461-475：上一段轨迹只有 1 个 SFC 且已经连接到 goal → 不需要再重规划
if (!gi_.new_goal && last_exp_traj_info.getSFCSize() == 1
                  && last_exp_traj_info.connectedToGoal()) {
    return NO_NEED;
}

// L478-491：在 replan_state 处的轨迹位置已经 < 3 格的距离到目标 → 不需要规划
if (!gi_.new_goal &&
    (gi_.goal_p - last_exp_traj.getPos(replan_state_TT)).norm() < cfg_.resolution * 3) {
    return NO_NEED;
}
```

> 这一组早退检查在长距离平直飞行中节省了大量算力——EGO 没有类似机制，每个 0.4s 都强制重规划一次。

---

## 阶段 3：碰撞检查 + 收集 guide_path（L502-584）

```cpp
// L502-531：沿 cmd_traj 从 replan_state_TT 开始往后采样
// 每隔 sample_traj_dt 秒取一个点，检查 inf_map 上是否被占据
// 同时记录"整段轨迹是否完全在 known free 内"
last_exp_traj_info.setWholeTrajKnownFreeFlag(true);
for (eval_t = replan_state_TT + sample_traj_dt;
     eval_t < guide_pos_traj_total_time;
     eval_t += cfg_.sample_traj_dt) {
    temp_pt = guide_pos_traj.getPos(eval_t);
    rog_map::GridType temp_grid = map_ptr_->getInfGridType(temp_pt);
    if (temp_grid == GridType::OCCUPIED || temp_grid == GridType::OUT_OF_MAP) {
        last_exp_traj_info.setWholeTrajKnownFreeFlag(false);
        break;       // 只要碰到一次撞障，就不再继续
    }
    last_exp_traj_time_pos.emplace_back(eval_t, temp_pt);
    last_exp_traj_vel.emplace_back(guide_pos_traj.getVel(eval_t).norm());
}

// L537-540：决定 receding 距离
// receding = "把上一段轨迹的多大一段保留作为 guide_path"
// 若整段安全 + 没有新目标 → 保留全部（split_dis = INF）
// 若发现碰撞或新目标 → 只保留 receding_dis 米
double split_dis = cfg_.receding_dis;
if (last_exp_traj_info.wholeTrajKnownFree() && !gi_.new_goal && cfg_.receding_dis > 0.0) {
    split_dis = std::numeric_limits<double>::max();
}

// L542-547：取 replan_state_TT 处的状态作为新轨迹起点
if (!guide_pos_traj.getState(replan_state_TT, pos_init_state)) {
    return FAILED;
}

// L552-584：把保留的 guide_path 段抽出来，附带时间戳
// 注意要把"撞障的尾部"或"距离起点超过 split_dis 的部分"丢掉
while (map_ptr_->isOccupiedInflate(temp_pt) ||
       (temp_pt - pos_init_state.col(0)).norm() > split_dis) {
    last_exp_traj_time_pos.pop_back();
    last_exp_traj_vel.pop_back();
    if (last_exp_traj_time_pos.empty()) break;
    temp_pt = last_exp_traj_time_pos.back().second;
}
```

---

## 阶段 4：用 A* 延伸 guide_path 到目标（L587-698）

```cpp
double guide_path_length = geometry_utils::computePathLength(guide_path);
double temp_horizon = cfg_.planning_horizon - guide_path_length;

// 如果 guide_path 已经长够 planning_horizon，就不需要再走 A*
if (temp_horizon > cfg_.resolution * 2) {
    if ((guide_path.back() - gi_.goal_p).norm() < cfg_.resolution * 5) {
        // 已经接近目标 → 直接连一段到 goal
        guide_path.push_back(gi_.goal_p);
        guide_stamp.push_back(... + dist / max_vel);
    } else {
        // 否则用 A* 搜一段补上去
        vec_Vec3f new_path;
        if (!PathSearch(guide_path.back(), gi_.goal_p, temp_horizon, new_path)) {
            return FAILED;
        }

        // L651-678：用"加速→匀速→减速"的 PM (Pontryagin Maximum) 时间分配
        // simplePMTimeAllocator(max_acc, max_vel, end_vel, total_dis, dis_i, stamp_i, vel_i)
        // 给 new_path 上每个点打时间戳
        ...
        for (long unsigned int i = 1; i < new_path.size(); i++) {
            time_stamp += dt[i];
            guide_path.emplace_back(new_path[i]);
            guide_stamp.emplace_back(time_stamp);
        }
    }
}
```

> PathSearch 内部（L1107-1201）的细节：先在 `inf_map` 上搜（更快），失败再回退到 `prob_map`（更精确）。同时支持 `frontend_in_known_free` 选项 —— true 时把 unknown 当作 occupied，让前端只搜已知自由区域。

---

## 阶段 5：CorridorGenerator → SFC 多面体序列（L703-720）

```cpp
sfc.clear();
ros_ptr_->vizFrontendPath(guide_path);
shifted_sfc_start_pt_ = Vec3f(9999,9999,9999);   // sentinel

bool bool_ret_code = cg_ptr_->SearchPolytopeOnPath(
                          guide_path, sfc,
                          shifted_sfc_start_pt_,    // 输出：第一段 SFC 的实际起点
                          cfg_.use_fov_cut          // 是否用相机视场剪裁第一段
                       );

if (!bool_ret_code) {
    return FAILED;     // 无法生成连续走廊
}
ros_ptr_->vizExpSfc(sfc);
```

`shifted_sfc_start_pt_` 的妙用：当起点本身在 OCCUPIED 格上（罕见但可能），`SearchPolytopeOnPath` 会自动平移起点到第一个自由格，并把这个新起点写回这个变量，后续 backup_traj 也以它为锚点。

---

## 阶段 6：MINCO 优化（L725-756）

```cpp
// 终点状态：默认 V=A=J=0（停在最后一个 SFC 末端）
pos_fina_state.setZero();
pos_fina_state.col(0) = guide_path.back();

// 例外：如果开启了 goal_vel 且距离目标超过 horizon/2，给一个朝向目标的速度
if (cfg_.goal_vel_en && (gi_.goal_p - robot_state_.p).norm() > cfg_.planning_horizon / 2) {
    pos_fina_state.col(1) = (gi_.goal_p - robot_state_.p).normalized()
                          * cfg_.exp_traj_cfg.max_vel / 2;
}
// 如果 guide_path.back() ≈ goal，就硬把终点钉在 goal 上、速度归零
if ((pos_fina_state.col(0) - gi_.goal_p).norm() < cfg_.resolution * 2) {
    pos_fina_state.col(1).setZero();
    pos_fina_state.col(0) = gi_.goal_p;
}

// 调用 MINCO 优化器（核心，详见 7_minco_trajopt.md）
Trajectory out_traj;
TimeConsuming t_exp_opt("t_exp_opt", false);
auto original_sfc = sfc;
temp_ret = exp_traj_opt_->optimize(pos_init_state,    // 起点（4×3）
                                    pos_fina_state,    // 终点（4×3）
                                    guide_path,        // 引导点（用作 attractor）
                                    guide_stamp,       // 引导点时间戳
                                    sfc,                // SFC 序列（位置约束）
                                    out_traj);         // 输出轨迹
time_consuming_[EXP_TRAJ_OPT] = t_exp_opt.stop();

if (!temp_ret) return FAILED;

// 时间预算：超过 replan_forward_dt 直接 abort
if ((ros_ptr_->getSimTime() - replan_process_start_WT) > cfg_.replan_forward_dt) {
    return FAILED;
}
```

---

## 阶段 7：与旧轨迹拼接 + Yaw 优化（L763-825）

```cpp
double new_traj_WT = replan_process_start_WT;
replan_process_start_TT = replan_process_start_WT - guide_pos_traj.start_WT;

// 7.1) 截取旧轨迹 [replan_process_start_TT, replan_state_TT] 段
//      注意这一段是飞控正在消费的，不能动
Trajectory temp_exp_traj;
if (!last_exp_traj_info_.empty() &&
    !guide_pos_traj.getPartialTrajectoryByTime(replan_process_start_TT,
                                                replan_state_TT,
                                                temp_exp_traj)) {
    return FAILED;
}

// 7.2) 拼接 = [旧头段] + [新优化轨迹]
out_exp_traj_info.setSFC(sfc);
temp_exp_traj = temp_exp_traj + out_traj;
temp_exp_traj.start_WT = new_traj_WT;

// 7.3) Yaw：从旧 yaw 轨迹的 replan_state_TT 处取值作为新 yaw 起点
Vec4f init_yaw{robot_state_.yaw, 0, 0, 0};
if (!last_exp_traj_info.empty()) {
    StatePVAJ yaw_replan_state;
    guide_yaw_traj.getState(replan_state_TT, yaw_replan_state);
    init_yaw = yaw_replan_state.row(0);
}

// 7.4) Yaw 终点：可选锁定到 goal_yaw
Vec4f fina_yaw{0, 0, 0, 0};
bool free_end{true};
if (cfg_.goal_yaw_en && !isnan(gi_.goal_yaw) && connected_goal) {
    free_end = false;
    fina_yaw[0] = gi_.goal_yaw;
}

// 7.5) Yaw 优化（独立的 1D 多项式）
//      out_traj：位置轨迹（已得到），用于估算 dpsi/dt 的边界
//      参数 3 表示阶数，free_end 决定终点是否自由
Trajectory new_traj, old_traj;
if (!yaw_traj_opt_->optimize(init_yaw, fina_yaw, out_traj,
                              new_traj, 3, false, free_end)) {
    return FAILED;
}

// 7.6) Yaw 旧段截取并拼接
if (!last_exp_traj_info.empty()) {
    guide_yaw_traj.getPartialTrajectoryByTime(replan_process_start_TT,
                                                replan_state_TT, old_traj);
}
const auto temp_yaw_traj = old_traj + new_traj;

// 7.7) 标记当前 exp 轨迹是否包含"上一次 backup 的一部分"
//      （ReplanOnce 时如果 last cmd 已经切到 backup 上，这部分需要保留时间窗）
double on_backup_end_TT{-1}, on_backup_start_TT{-1};
if (!last_exp_traj_info.empty() && replan_state_TT > cmd_traj_info_.getBackupTrajStartTT()) {
    on_backup_start_TT = cmd_traj_info_.getBackupTrajStartTT() - replan_process_start_TT;
    on_backup_end_TT = replan_state_TT - replan_process_start_TT;
}

// 7.8) 设置最终 ExpTraj
out_exp_traj_info.setTrajectory(new_traj_WT, temp_exp_traj, temp_yaw_traj,
                                 on_backup_start_TT, on_backup_end_TT);

return SUCCESS;
```

---

## 流程图

```
┌─────────────────────┐
│ generateExpTraj 入口 │
└──────────┬──────────┘
           │
           ▼
   last_exp_traj_info empty?
       ├─ 是 ─▶ pos_init = local_start_p_, replan_state_TT = -1
       └─ 否 ─▶ ┌─────────────────────────────────┐
                │ 早退检查（NO_NEED 五连）         │
                └─────────────────────────────────┘
                          │ 没满足
                          ▼
                ┌────────────────────────────────┐
                │ 沿旧轨迹采样 + 碰撞检查         │
                │  → wholeTrajKnownFree?          │
                │  → split_dis = INF or receding  │
                └────────────────────────────────┘
                          │
                          ▼
                ┌────────────────────────────────┐
                │ 保留 guide_path（旧轨迹有效段） │
                └────────────────────────────────┘
                          │
                          ▼
        ┌─────────────────────────────────────────┐
        │ A* PathSearch：guide_path.back() → goal │
        │  (inf_map 优先，prob_map 兜底)            │
        └─────────────────────────────────────────┘
                          │
                          ▼
        ┌─────────────────────────────────────────┐
        │ simplePMTimeAllocator：给 A* 路径打时间 │
        └─────────────────────────────────────────┘
                          │
                          ▼
        ┌─────────────────────────────────────────┐
        │ CorridorGenerator::SearchPolytopeOnPath │
        │   → SFC 序列（凸多面体走廊）             │
        └─────────────────────────────────────────┘
                          │
                          ▼
        ┌─────────────────────────────────────────┐
        │ ExpTrajOpt::optimize（MINCO + L-BFGS）   │
        │   cost = pos + vel + acc + jer +         │
        │          attractor + omg + thrust        │
        └─────────────────────────────────────────┘
                          │
                          ▼
        ┌─────────────────────────────────────────┐
        │ 拼接 [旧头段 + 新优化轨迹] + yaw 优化     │
        └─────────────────────────────────────────┘
                          │
                          ▼
                       SUCCESS
```

---

## 与 EGO `planFromCurrentTraj` 的对比

| 项目 | EGO `planFromCurrentTraj` | SUPER `generateExpTraj` |
|---|---|---|
| 起点 | 当前时刻轨迹位置 | replan_state_TT（前向 0.3s 后） |
| 旧轨迹是否拼接 | 否，覆盖式生成 | 是，前 0.3s 头段保留 |
| 引导路径 | 仅多项式 / 当前轨迹延伸 | 旧轨迹采样 + A* 延伸 |
| 走廊 | 无（直接在控制点上做避障） | 多段凸多面体 SFC |
| 优化方法 | 三次 B-spline + L-BFGS + collision rebound | 7 阶 MINCO + L-BFGS + smoothL1 |
| 早退检查 | 无 | 5 种 NO_NEED 早退 |
| 时间预算 | 每帧规划但不严格检查 | 强制 < 0.9 × replan_forward_dt |
