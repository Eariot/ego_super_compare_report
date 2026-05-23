# ego_replan_fsm.cpp（续）—— execFSMCallback 与 getLocalTarget

## 文件路径
`official/Fast-Drone-250/src/planner/plan_manage/src/ego_replan_fsm.cpp`

> 本篇覆盖 execFSMCallback（状态机主循环）和 getLocalTarget（局部规划目标选取）两部分。

---

## execFSMCallback —— FSM 的心脏（L431-616）

### 整体流程概览

`execFSMCallback` 每 0.01 秒（100Hz）执行一次，是 EGO 状态机真正的调度中心。它通过一个 `switch` 语句处理 7 种状态，决定何时规划、何时重规划、何时急停。

```
每帧执行：停止定时器 → 判断当前状态 → 执行对应逻辑 → 发布调试数据 → 重启定时器
```

---

### L431-445：定时器防阻塞 + 状态打印

```cpp
// EGOReplanFSM::execFSMCallback —— FSM 主循环，每 0.01s 执行一次
void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
{
    // 【重要】先停止定时器，防止回调执行时间超过 0.01s 导致定时器堆积
    // 这是原版 EGO 避免阻塞的唯一手段——如果 stop/start 之间逻辑太重，会丢帧
    exec_timer_.stop();

    // 每 100 帧（约 1 秒）打印一次状态
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
        printFSMExecState();  // 打印当前状态名
        if (!have_odom_) cout << "no odom." << endl;
        if (!have_target_) cout << "wait for goal or trigger." << endl;
        fsm_num = 0;
    }

    // 以下进入 switch，根据当前状态执行不同逻辑
    // ...
```

---

### L447-458：INIT 状态处理

```cpp
switch (exec_state_)
{
case INIT:  // 初始化状态
{
    // 没有里程计数据就什么都不做，直接返回
    if (!have_odom_)
    {
        goto force_return;
    }
    // 有里程计了 → 进入 WAIT_TARGET，等待目标点或触发信号
    changeFSMExecState(WAIT_TARGET, "FSM");
    break;
}
```

---

### L460-477：WAIT_TARGET 状态处理

```cpp
case WAIT_TARGET:  // 等待目标状态
{
    // 需要同时具备：目标点（have_target_）+ 触发信号（have_trigger_）
    // 两者缺一不可：没有目标点不知道去哪，没有触发信号不能起飞
    if (!have_target_ || !have_trigger_)
        goto force_return;

    // 两项都满足 → 进入 SEQUENTIAL_START（单机直接生成轨迹）
    changeFSMExecState(SEQUENTIAL_START, "FSM");
    break;
}
```

---

### L479-506：SEQUENTIAL_START 状态处理（多机顺序启动）

```cpp
case SEQUENTIAL_START:  // 多机模式：等待前机轨迹
{
    // 条件：单机 drone_id<=0 → 直接满足
    //       多机 drone_id>=1 → 必须 have_recv_pre_agent_=true（收到前机轨迹）
    if (planner_manager_->pp_.drone_id <= 0 ||
        (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
    {
        if (have_odom_ && have_target_ && have_trigger_)
        {
            // 调用 planFromGlobalTraj(10)：最多尝试 10 次生成全局轨迹
            bool success = planFromGlobalTraj(10);
            if (success)
            {
                changeFSMExecState(EXEC_TRAJ, "FSM");  // 生成成功 → 执行轨迹
                publishSwarmTrajs(true);               // 发布轨迹给后一架无人机
            }
            else
            {
                ROS_ERROR("Failed to generate the first trajectory!!!");
                changeFSMExecState(SEQUENTIAL_START, "FSM");  // 失败 → 留在本状态重试
            }
        }
        else
        {
            ROS_ERROR("No odom or no target! have_odom_=%d, have_target_=%d",
                      have_odom_, have_target_);
        }
    }
    break;
}
```

---

### L508-527：GEN_NEW_TRAJ 状态处理（生成新轨迹）

```cpp
case GEN_NEW_TRAJ:  // 生成全新轨迹（第一次规划或急停恢复后）
{
    // planFromGlobalTraj(10)：从当前无人机位置到目标点，生成一段完整的新轨迹
    // 最多尝试 10 次
    bool success = planFromGlobalTraj(10);

    if (success)
    {
        changeFSMExecState(EXEC_TRAJ, "FSM");  // 成功 → 执行轨迹
        flag_escape_emergency_ = true;         // 标记：已退出急停模式
        publishSwarmTrajs(false);               // 发布轨迹（非启动模式）
    }
    else
    {
        // 失败 → 留在 GEN_NEW_TRAJ 状态，下一帧继续尝试
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
    }
    break;
}
```

---

### L529-543：REPLAN_TRAJ 状态处理（重规划）

```cpp
case REPLAN_TRAJ:  // 重规划（在已有轨迹基础上修正）
{
    // planFromCurrentTraj(1)：基于当前轨迹的剩余部分进行重规划
    // 不从头生成，而是从当前位置继续，效率更高
    if (planFromCurrentTraj(1))
    {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        publishSwarmTrajs(false);
    }
    else
    {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
    }
    break;
}
```

---

### L545-591：EXEC_TRAJ 状态处理（轨迹执行中）—— 最重要

```cpp
case EXEC_TRAJ:  // 执行轨迹中（正常飞行状态）
{
    // 核心逻辑：判断"是否需要重规划"
    // 取当前时刻无人机沿轨迹的位置
    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();
    t_cur = min(info->duration_, t_cur);  // 不超过轨迹总时长
    Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

    /* ---- 条件1：预设航点序列模式，自动切换下一个航点 ---- */
    // 如果当前航点距离足够近（< no_replan_thresh_），且还有下一个航点
    if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
        (wp_id_ < waypoint_num_ - 1) &&
        (end_pt_ - pos).norm() < no_replan_thresh_)
    {
        wp_id_++;                                      // 切换到下一个航点
        planNextWaypoint(wps_[wp_id_]);               // 触发新规划
    }
    /* ---- 条件2：接近全局终点（local_target == 全局终点）---- */
    else if ((local_target_pt_ - end_pt_).norm() < 1e-3)
    {
        if (t_cur > info->duration_ - 1e-2)  // 轨迹快执行完了
        {
            have_target_ = false;
            have_trigger_ = false;

            if (target_type_ == TARGET_TYPE::PRESET_TARGET)
            {
                // 预设模式：回到第一个航点，重新开始
                wp_id_ = 0;
                planNextWaypoint(wps_[wp_id_]);
            }
            changeFSMExecState(WAIT_TARGET, "FSM");
            goto force_return;
        }
        else if ((end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_)
        {
            // 还没到终点，但离终点足够远且执行够久 → 重规划
            changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
    }
    /* ---- 条件3：执行够久（t_cur > replan_thresh_）→ 重规划 ---- */
    else if (t_cur > replan_thresh_)
    {
        // 每隔 replan_thresh_ 秒强制触发一次重规划
        // 用于更新轨迹、应对新出现的障碍
        changeFSMExecState(REPLAN_TRAJ, "FSM");
    }
    break;
}
```

---

### L593-608：EMERGENCY_STOP 状态处理（急停）

```cpp
case EMERGENCY_STOP:  // 急停状态
{
    if (flag_escape_emergency_)  // 第一次进入急停
    {
        // 发布急停轨迹：所有控制点都是当前位置，速度、加速度为零
        callEmergencyStop(odom_pos_);
    }
    else
    {
        // 已经在急停中：速度小于 0.1 m/s → 尝试重新生成轨迹
        if (enable_fail_safe_ && odom_vel_.norm() < 0.1)
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
    }

    flag_escape_emergency_ = false;  // 重置标记
    break;
}
```

---

### L611-616：收尾

```cpp
    // 发布调试数据（状态信息发送到 RViz）
    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);

force_return:
    exec_timer_.start();  // 重启定时器，准备下一帧
}
```

---

## getLocalTarget —— 局部规划目标如何选取（L894-949）

### 整体设计思想

EGO 是"全局参考 + 局部优化"的架构。`getLocalTarget` 的作用是：
- 从全局轨迹上找到一个点，使得该点到**当前无人机位置**的距离 ≈ `planning_horizon_`
- 这个点就是局部 B-spline 优化的"局部终点"，EGO 只优化从当前位置到该点的这段轨迹

```
当前无人机位置 ────(≈ planning_horizon_)────▶ [local_target_pt]
                    ↑
              从全局轨迹上找
```

### 详细逻辑

```cpp
// EGOReplanFSM::getLocalTarget —— 在全局轨迹上找距离为 planning_horizon 的点
void EGOReplanFSM::getLocalTarget()
{
    double t;

    // L898：采样步长
    // = planning_horizon / 20 / max_vel
    // 目的是在一个 planning_horizon 距离内均匀采样约 20 个点
    // 步长不宜太小（计算量大），不宜太大（目标选择不精确）
    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;

    // dist_min：记录已遍历路径中，距离无人机最近的累积距离
    // dist_min_t：对应的全局轨迹时间参数
    double dist_min = 9999, dist_min_t = 0.0;

    // L900-934：从 last_progress_time_ 开始沿全局轨迹向前遍历
    for (t = planner_manager_->global_data_.last_progress_time_;
         t < planner_manager_->global_data_.global_duration_;
         t += t_step)
    {
        // 取出全局轨迹上 t 时刻的位置
        Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);

        // 计算该点到当前无人机位置的距离
        double dist = (pos_t - start_pt_).norm();

        /* ---- 角落情况处理：避免 local_target 落在无人机身后 ---- */
        // 如果在起点附近（last_progress_time_+微小量），距离就已经 > planning_horizon
        // 说明无人机处于轨迹的一个弯曲处，此时应该继续找直到距离 < planning_horizon
        if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 &&
            dist > planning_horizen_)
        {
            // 从起点开始继续前进，直到找到距离首次小于 planning_horizon 的点
            for (; t < planner_manager_->global_data_.global_duration_; t += t_step)
            {
                Eigen::Vector3d pos_t_temp = planner_manager_->global_data_.getPosition(t);
                double dist_temp = (pos_t_temp - start_pt_).norm();
                if (dist_temp < planning_horizen_)
                {
                    pos_t = pos_t_temp;
                    dist = (pos_t - start_pt_).norm();
                    cout << "Escape conor case \"getLocalTarget\"" << endl;
                    break;
                }
            }
        }

        // 记录目前为止距离最小的时刻（用于恢复 last_progress_time_）
        if (dist < dist_min)
        {
            dist_min = dist;
            dist_min_t = t;
        }

        // 关键：当距离首次 >= planning_horizon 时，停止搜索
        // 此时 pos_t 就是最近的、距离超过前视窗口的点
        // 但实际上我们想要的是"恰好超过"的那个点，所以保存它
        if (dist >= planning_horizen_)
        {
            local_target_pt_ = pos_t;                           // 设置局部目标点
            planner_manager_->global_data_.last_progress_time_ = dist_min_t;  // 记录进度
            break;
        }
    }

    // L935-939：如果遍历完整个全局轨迹都没找到距离 >= planning_horizon 的点
    // 说明剩余轨迹不够长，直接把终点设为局部目标
    if (t > planner_manager_->global_data_.global_duration_)
    {
        local_target_pt_ = end_pt_;  // 使用全局终点
        planner_manager_->global_data_.last_progress_time_ =
            planner_manager_->global_data_.global_duration_;
    }

    // L941-948：根据到终点的距离决定局部目标速度
    // 如果距离终点足够近（能在一个 braking 距离内停下），速度设为 0
    // 否则使用全局轨迹在 t 时刻的速度
    // 公式：braking_dist = v²/2a，即 (max_vel)² / (2 * max_acc)
    double braking_dist = (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_)
                          / (2 * planner_manager_->pp_.max_acc_);
    if ((end_pt_ - local_target_pt_).norm() < braking_dist)
    {
        local_target_vel_ = Eigen::Vector3d::Zero();  // 终点速度为零
    }
    else
    {
        local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
    }
}
```

### getLocalTarget 与 planning_horizon_ 的关系

| 参数 | 含义 | 典型值 | 说明 |
|---|---|---|---|
| `planning_horizon_` | 前视距离（米） | 7.5 | EGO 向前看多远设局部目标 |
| `planning_horizen_time_` | 前视时间窗口（秒） | - | planning_horizon 的时间版本（参数存在但代码未使用） |

**为什么用空间前视而非时间前视？**
- 空间前视更直观：障碍物距离是空间的，安全边界也是空间的
- 时间前视在速度变化时前视窗口会变化，不如空间稳定

---

## planFromGlobalTraj 与 planFromCurrentTraj

### planFromGlobalTraj（L618-638）—— 从头生成轨迹

```cpp
// planFromGlobalTraj —— 从当前位置到目标点，生成全新轨迹
// trial_times=10：最多尝试 10 次
bool EGOReplanFSM::planFromGlobalTraj(const int trial_times)
{
    start_pt_ = odom_pos_;     // 起点 = 当前无人机位置
    start_vel_ = odom_vel_;    // 起点速度 = 当前速度
    start_acc_.setZero();       // 起点加速度 = 0

    // 第一次调用用确定性多项式生成初始路径
    // 后续重调用用随机多项式（增加探索性）
    bool flag_random_poly_init;
    if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
    else
        flag_random_poly_init = true;

    for (int i = 0; i < trial_times; i++)
    {
        // callReboundReplan(true, flag_random_poly_init)
        //   true = flag_use_poly_init：用多项式生成初始路径
        if (callReboundReplan(true, flag_random_poly_init))
            return true;
    }
    return false;
}
```

### planFromCurrentTraj（L640-675）—— 基于当前轨迹重规划

```cpp
// planFromCurrentTraj —— 从当前轨迹的剩余部分继续优化
// trial_times=1：只尝试 1 次
bool EGOReplanFSM::planFromCurrentTraj(const int trial_times)
{
    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();

    // 从当前轨迹上取"当前位置"作为规划起点（而非 odom_pos_）
    // 这样可以利用已生成的轨迹信息，减少突变
    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    // 1. 先尝试基于当前轨迹继续优化（效率最高）
    bool success = callReboundReplan(false, false);
    //   false = flag_use_poly_init：不重新生成初始路径，直接用当前轨迹

    if (!success)
    {
        // 2. 失败 → 用当前轨迹位置但重新生成多项式初始路径
        success = callReboundReplan(true, false);

        if (!success)
        {
            // 3. 仍然失败 → 多次随机重试
            for (int i = 0; i < trial_times; i++)
            {
                success = callReboundReplan(true, true);
                if (success) break;
            }
            if (!success) return false;
        }
    }
    return true;
}
```

---

## checkCollisionCallback —— 安全守护（L677-753）

### 整体设计

`safety_timer_` 每 0.05 秒（20Hz）执行一次。核心功能：
1. 检查里程计/深度图是否丢失 → EMERGENCY_STOP
2. 检查当前轨迹是否撞障碍物 → REPLAN_TRAJ 或 EMERGENCY_STOP

```cpp
void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
{
    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    // 还没开始规划，不用检查
    if (exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)
        return;

    /* ---- 检查1：深度图/里程计丢失 ---- */
    // 如果 odom 或 depth 超过 mp_.odom_depth_timeout_ 秒没有更新
    // 地图不可信，必须急停
    if (map->getOdomDepthTimeout())
    {
        ROS_ERROR("Depth Lost! EMERGENCY_STOP");
        enable_fail_safe_ = false;
        changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    }

    /* ---- 检查2：轨迹碰撞 ---- */
    // 每 0.01 秒检查一次轨迹上的点
    constexpr double time_step = 0.01;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    Eigen::Vector3d p_cur = info->position_traj_.evaluateDeBoorT(t_cur);

    // 多机避障：安全距离 = swarm_clearance
    const double CLEARANCE = 1.0 * planner_manager_->getSwarmClearance();

    double t_cur_global = ros::Time::now().toSec();
    double t_2_3 = info->duration_ * 2 / 3;  // 只检查前 2/3 轨迹

    for (double t = t_cur; t < info->duration_; t += time_step)
    {
        // 只检查轨迹前 2/3（后 1/3 是缓冲区，不强制要求）
        if (t_cur < t_2_3 && t >= t_2_3)
            break;

        // 2a. 检查是否撞障碍物
        bool occ = map->getInflateOccupancy(
            info->position_traj_.evaluateDeBoorT(t));

        // 2b. 检查是否与其他无人机太近（多机模式）
        for (size_t id = 0; id < planner_manager_->swarm_trajs_buf_.size(); id++)
        {
            if ((planner_manager_->swarm_trajs_buf_.at(id).drone_id == (int)id) &&
                (planner_manager_->swarm_trajs_buf_.at(id).drone_id != planner_manager_->pp_.drone_id))
            {
                double t_X = t_cur_global -
                             planner_manager_->swarm_trajs_buf_.at(id).start_time_.toSec();
                Eigen::Vector3d swarm_pridicted =
                    planner_manager_->swarm_trajs_buf_.at(id).position_traj_.evaluateDeBoorT(t_X);
                double dist = (p_cur - swarm_pridicted).norm();
                if (dist < CLEARANCE)
                {
                    occ = true;
                    break;
                }
            }
        }

        if (occ)
        {
            // 发现障碍物！先尝试一次基于当前轨迹重规划
            if (planFromCurrentTraj())
            {
                changeFSMExecState(EXEC_TRAJ, "SAFETY");  // 成功 → 继续执行
                publishSwarmTrajs(false);
                return;
            }
            else
            {
                // 重规划失败
                if (t - t_cur < emergency_time_)  // 距离障碍物 < 0.8s
                {
                    ROS_WARN("Suddenly discovered obstacles. emergency stop!");
                    changeFSMExecState(EMERGENCY_STOP, "SAFETY");
                }
                else
                {
                    // 障碍物在前方较远处，可以触发重规划
                    changeFSMExecState(REPLAN_TRAJ, "SAFETY");
                }
                return;
            }
        }
    }
}
```

---

## callReboundReplan —— 规划调度的包装器（L755-805）

```cpp
// EGOReplanFSM::callReboundReplan —— 调度 planner_manager_->reboundReplan()
// 并负责发布 Bspline 轨迹
bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
{
    // 1. 获取局部目标点（从全局轨迹上找到 planning_horizon 处的点）
    getLocalTarget();

    // 2. 调用核心优化函数 reboundReplan
    bool plan_and_refine_success =
        planner_manager_->reboundReplan(
            start_pt_, start_vel_, start_acc_,
            local_target_pt_, local_target_vel_,
            (have_new_target_ || flag_use_poly_init),  // 是否用多项式初始路径
            flag_randomPolyTraj                          // 是否用随机初始路径
        );
    have_new_target_ = false;

    if (plan_and_refine_success)
    {
        auto info = &planner_manager_->local_data_;

        // 3. 构建 Bspline 消息（控制点 + 节点向量）
        traj_utils::Bspline bspline;
        bspline.order = 3;
        bspline.start_time = info->start_time_;
        bspline.traj_id = info->traj_id_;

        Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
        for (int i = 0; i < pos_pts.cols(); ++i)
        {
            geometry_msgs::Point pt;
            pt.x = pos_pts(0, i);
            pt.y = pos_pts(1, i);
            pt.z = pos_pts(2, i);
            bspline.pos_pts.push_back(pt);
        }

        Eigen::VectorXd knots = info->position_traj_.getKnot();
        for (int i = 0; i < knots.rows(); ++i)
            bspline.knots.push_back(knots(i));

        // 4. 发布三个方向：
        //    a) traj_server（控制无人机飞）
        bspline_pub_.publish(bspline);
        //    b) 后一架无人机（多机协调）
        //    c) RViz 可视化（显示优化后的控制点）
        visualization_->displayOptimalList(
            info->position_traj_.get_control_points(), 0);
    }

    return plan_and_refine_success;
}
```

---

## 状态机完整状态转换图

```
                    ┌─────────────────────────────────────┐
                    │  仿真模式: have_trigger_=true      │
                    │  实机模式: 收到 /traj_start_trigger  │
                    └──────────────┬──────────────────────┘
                                   ▼
┌──────────┐  have_odom_   ┌──────────────┐  have_target_&have_trigger_   ┌────────────────────┐
│   INIT   │──────────────▶│  WAIT_TARGET │────────────────────────────────▶│  SEQUENTIAL_START  │
└──────────┘               └──────────────┘                                  │   (多机顺序启动)    │
                                                                              └────────┬───────────┘
                                                                                       │ planFromGlobalTraj()
                                                                                       ▼
┌──────────────────┐                                           ┌─────────────┐
│  EMERGENCY_STOP  │◀─────────────────────────────────────────│  EXEC_TRAJ  │◀──┐
│   (急停状态)      │   EMERGENCY_STOP                         │  (执行轨迹)  │    │
└────────┬─────────┘   or planFromCurrentTraj()                └──────┬──────┘    │
         │                                                              │           │
         │  enable_fail_safe_ & vel<0.1                                │ t_cur>    │ REPLAN_TRAJ
         ▼                                                              │ replan_thresh_   │
┌──────────────────┐                                                     │           │
│   GEN_NEW_TRAJ   │────────────────────────────────────────────────────┘           │
│   (生成新轨迹)    │                                                                   │
└──────────────────┘                                                                   │
                                                                                       │
                                    REPLAN_TRAJ ◀────────────────────────────────────────┘
                                  (planFromCurrentTraj())
```

---

## execFSMCallback 7 状态速查表

| 状态 | 入口条件 | 核心动作 | 出口条件 |
|---|---|---|---|
| `INIT` | 程序启动 | 等待 have_odom_ | have_odom_=true → WAIT_TARGET |
| `WAIT_TARGET` | INIT 完成 | 等待目标+触发 | have_target_+have_trigger_ → SEQUENTIAL_START |
| `SEQUENTIAL_START` | 多机等待前机 | planFromGlobalTraj(10) | 成功 → EXEC_TRAJ |
| `GEN_NEW_TRAJ` | 急停恢复/第一次 | planFromGlobalTraj(10) | 成功 → EXEC_TRAJ |
| `REPLAN_TRAJ` | EXEC_TRAJ 超时/安全触发 | planFromCurrentTraj(1) | 成功 → EXEC_TRAJ |
| `EXEC_TRAJ` | 轨迹生成成功 | 正常飞行，判断是否重规划 | 满足条件 → REPLAN_TRAJ 或 WAIT_TARGET |
| `EMERGENCY_STOP` | 碰撞/深度丢失 | callEmergencyStop | 速度<0.1 → GEN_NEW_TRAJ |
