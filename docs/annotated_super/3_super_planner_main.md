# super_planner.cpp（一）—— 类结构、PlanFromRest、ReplanOnce

## 文件路径
- 头文件：`super_planner/include/super_core/super_planner.h`
- 实现：`super_planner/src/super_core/super_planner.cpp`（1208 行）

## 核心理解

`SuperPlanner` 是 SUPER 的"中央调度器"，它负责把以下模块串起来：
- ROG-Map（地图）
- A*（前端搜索，`astar_ptr_`）
- CorridorGenerator + CIRI（凸多面体走廊）
- ExpTrajOpt / BackupTrajOpt / YawTrajOpt（三个 MINCO 优化器）
- FOVChecker（视场剪裁）

它对外只暴露两个核心接口：

| 接口 | 触发场景 | 作用 |
|---|---|---|
| `PlanFromRest(goal, yaw, new_goal)` | FSM GENERATE_TRAJ | 从静止状态生成第一段轨迹 |
| `ReplanOnce(goal, yaw, new_goal)` | FSM FOLLOW_TRAJ（持续触发） | 在飞行过程中持续重规划 |

两者内部都调用同一组 `generateExpTraj` + `generateBackupTrajectory`，差别只是初始状态来源。

---

## 类成员速查（super_planner.h L59-99）

```cpp
class SuperPlanner {
private:
    super_planner::Config cfg_;            // 自家参数（planning_horizon、robot_r 等）
    rog_map::ROGMapROS::Ptr map_ptr_;       // 共享 ROG-Map（外部传入）
    CorridorGenerator::Ptr cg_ptr_;        // SFC 生成器
    path_search::Astar::Ptr astar_ptr_;    // 前端搜索器
    ros_interface::RosInterface::Ptr ros_ptr_; // ROS1/ROS2 抽象

    traj_opt::ExpTrajOpt::Ptr exp_traj_opt_;     // 探索轨迹优化器（s=4）
    traj_opt::BackupTrajOpt::Ptr back_traj_opt_; // 备份轨迹优化器（含 ts 自由度）
    traj_opt::YawTrajOpt::Ptr yaw_traj_opt_;     // 1D yaw 优化器

    CIRI::Ptr ciri_;                              // 凸多面体生成核心
    super_utils::RobotState robot_state_;        // 当前机体状态（位置、速度、姿态）

    std::mutex drone_state_mutex_;
    std::mutex replan_lock_;                      // 整个 PlanFromRest/ReplanOnce 加锁

    Vec3f local_start_p_;                          // 起点平移到自由空间的结果
    bool robot_on_backup_traj_{false};             // 当前是否已经在 backup 轨迹上

    struct GoalInfo {
        Vec3f goal_p{0, 0, 0};
        double goal_yaw{0};
        bool new_goal{true};
        bool goal_valid{true};
    } gi_;

    FOVChecker::Ptr fov_checker_;                 // 视场剪裁（可选）
    CmdTraj cmd_traj_info_;                        // 当前 committed 轨迹（含 exp+backup）
    ExpTraj last_exp_traj_info_;                  // 上一次 exp 轨迹（用于热启动）

    vector<double> time_consuming_;               // 各模块耗时 log
};
```

> 关键数据结构 `CmdTraj`（`include/data_structure/cmd_traj.h`）：把 exp + backup 拼成一条"切换式"轨迹，按时间 `t` 自动选 exp 或 backup 段，对外只暴露 `getState(t)`。

---

## 构造函数（super_planner.cpp L32-66）

```cpp
SuperPlanner::SuperPlanner(const std::string &cfg_path,
                           const ros_interface::RosInterface::Ptr &ros_ptr,
                           const rog_map::ROGMapROS::Ptr &map_ptr)
    : cfg_(Config(cfg_path)), ros_ptr_(ros_ptr), map_ptr_(map_ptr) {

    // 1) 把分辨率/可视化开关同步给 ROS interface
    ros_ptr_->setResolution(cfg_.resolution);
    ros_ptr_->setVisualizationEn(cfg_.visualization_en);

    // 2) 创建 3 个轨迹优化器（exp / backup / yaw）
    exp_traj_opt_  = std::make_shared<traj_opt::ExpTrajOpt>(cfg_.exp_traj_cfg, ros_ptr_);
    back_traj_opt_ = std::make_shared<traj_opt::BackupTrajOpt>(cfg_.back_traj_cfg, ros_ptr_);
    yaw_traj_opt_  = std::make_shared<traj_opt::YawTrajOpt>(cfg_.yaw_dot_max);

    // 3) A* + CorridorGenerator
    const auto &rog_map_cfg = map_ptr_->getMapConfig();
    astar_ptr_ = std::make_shared<path_search::Astar>(cfg_path, ros_ptr_, map_ptr_);
    cg_ptr_    = std::make_shared<CorridorGenerator>(
            ros_ptr_, map_ptr_, cfg_.corridor_bound_dis,
            cfg_.corridor_line_max_length,
            cfg_.resolution,
            rog_map_cfg.virtual_ground_height,
            rog_map_cfg.virtual_ceil_height,
            cfg_.robot_r,
            cfg_.obs_skip_num,
            cfg_.iris_iter_num);
    cg_ptr_->SetLineNeighborList(cfg_.seed_line_neighbour);  // 球形邻域，用于"线膨胀"碰撞查询

    time_consuming_.resize(8);   // 8 个耗时插槽：见 fsm.h log_time_str
    robot_state_.rcv = false;

    // 4) FOV Checker：当前默认全向 + ±35° 范围。可改为针对特定深度相机的视场
    fov_checker_ = std::make_shared<FOVChecker>(FOVType::OMNI, -1.0, -35.0, 35.0);

    // 5) 计算 A* 的精细邻域（无人机半径覆盖几格）
    const int neighbor_step = floor(cfg_.robot_r / cfg_.resolution);
    astar_ptr_->setFineInfNeighbors(neighbor_step);
}
```

---

## PlanFromRest —— 静止状态规划（L68-161）

```cpp
RET_CODE SuperPlanner::PlanFromRest(const Vec3f &goal_p,
                                    const double &goal_yaw,
                                    const bool &new_goal) {
    std::lock_guard<std::mutex> guard(replan_lock_);
    latest_replan.reset();
    latest_replan.setGoal(goal_p, goal_yaw, robot_state_);

    /* ---- 1) 必要前置条件 ---- */
    if (robot_state_.rcv == false) {
        latest_replan.setRetCode(SUPER_NO_ODOM);
        return FAILED;
    }
    gi_.goal_p = goal_p; gi_.goal_yaw = goal_yaw; gi_.new_goal = new_goal;
    gi_.goal_valid = true;

    // 可视化：从机体到目标的直线（橙色）
    ros_ptr_->vizGoalPath(vec_Vec3f{goal_p, robot_state_.p});

    /* ---- 2) 起点 shift 到自由空间 ---- */
    // 直接从 robot_state_.p 开始可能就在 OCCUPIED 格子上（如刚起飞）
    // 在 prob_map 上搜半径 3m 找最近的非占据格
    Vec3f local_star_pt;
    if (!map_ptr_->getNearestCellNot(GridType::OCCUPIED,
                                      robot_state_.p, local_star_pt, 3.0)) {
        ros_ptr_->error(" -- [SUPER] Local start point is deeply occupied.");
        latest_replan.setRetCode(SUPER_NO_START_POINT);
        return FAILED;
    }
    latest_replan.setLocalStartP(local_star_pt);

    /* ---- 3) 生成探索轨迹 (Exp) ---- */
    ExpTraj exp_traj_info;
    BackupTraj back_traj_info;
    last_exp_traj_info_.setEmpty();      // 没有"上一段"用于热启动
    local_start_p_ = local_star_pt;

    RET_CODE exp_ret_code = generateExpTraj(last_exp_traj_info_, exp_traj_info);
    if (exp_ret_code == FAILED) return FAILED;

    /* ---- 4) 生成备份轨迹 (Backup) ---- */
    back_traj_info.setEmpty();
    RET_CODE back_ret_code = generateBackupTrajectory(exp_traj_info, back_traj_info);

    /* ---- 5) 合并并提交 ---- */
    if (back_ret_code == SUCCESS) {
        // 5a) Exp + Backup 双轨迹模式（最常见）
        cmd_traj_info_.setTrajectory(exp_traj_info, back_traj_info);
        last_exp_traj_info_ = exp_traj_info;
        robot_on_backup_traj_ = false;
        gi_.new_goal = false;
        ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), cmd_traj_info_.getBackupTrajStartTT());
        latest_replan.setRetCode(SUPER_SUCCESS_WITH_BACKUP);
        return SUCCESS;
    }
    else if (back_ret_code == FINISH || back_ret_code == NO_NEED) {
        // 5b) Exp 全部在已知自由空间内 → 不需要 backup
        robot_on_backup_traj_ = false;
        cmd_traj_info_.setTrajectory(exp_traj_info);   // 只设置 exp
        last_exp_traj_info_ = exp_traj_info;
        gi_.new_goal = false;
        ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1);  // -1 表示无 backup
        latest_replan.setRetCode(SUPER_SUCCESS_NO_BACKUP);
        return SUCCESS;
    }
    return FAILED;
}
```

### 返回码对照（`super_core/super_ret_code.hpp`）

| 返回码 | 出现位置 | 含义 |
|---|---|---|
| SUPER_NO_ODOM | 入口检查 | 还没有 odom 数据 |
| SUPER_NO_START_POINT | shift 失败 | 半径 3m 内找不到自由格（撞墙了） |
| SUPER_SUCCESS_WITH_BACKUP | 双轨迹成功 | 最常见的成功路径 |
| SUPER_SUCCESS_NO_BACKUP | 不需要 backup | exp 完全在已知自由空间，安全无虞 |

---

## ReplanOnce —— 飞行中重规划（L164-295）

PlanFromRest 与 ReplanOnce 的核心区别：

| 项目 | PlanFromRest | ReplanOnce |
|---|---|---|
| 起点 | 当前 odom（shift 到自由空间） | 沿当前 cmd_traj 前 `replan_forward_dt` 秒后的状态 |
| 上一段 exp | 空 | 用作热启动 |
| 失败响应 | 直接返回 FAILED，FSM 留在 GENERATE_TRAJ | 保留旧 cmd_traj 继续飞，FSM 留在 FOLLOW_TRAJ |

```cpp
RET_CODE SuperPlanner::ReplanOnce(const Vec3f &goal_p,
                                  const double &goal_yaw,
                                  const bool &new_goal) {
    TimeConsuming replan_total_t("ReplanOnce", false);
    std::lock_guard<std::mutex> guard(replan_lock_);

    gi_.goal_p = goal_p; gi_.goal_yaw = goal_yaw; gi_.new_goal = new_goal;
    gi_.goal_valid = true;
    latest_replan.reset();
    latest_replan.setGoal(goal_p, goal_yaw, robot_state_);

    ros_ptr_->vizGoalPath(vec_Vec3f{goal_p, robot_state_.p});

    /* ---- 1) 重生成 exp 轨迹（基于 last_exp_traj_info_ 热启动） ---- */
    ExpTraj exp_traj_info;
    TimeConsuming t_exp("t_exp", false);
    RET_CODE exp_ret_code = generateExpTraj(last_exp_traj_info_, exp_traj_info);
    time_consuming_[GENERATE_EXP_TRAJ] = t_exp.stop();

    /* exp 返回码处理（不同于 PlanFromRest 的简单 if-else） */
    if (exp_ret_code == FAILED) return FAILED;
    if (exp_ret_code == NEW_TRAJ) return NEW_TRAJ;   // 当前轨迹已用完 → 让 FSM 切回 GENERATE_TRAJ
    if (exp_ret_code == EMER)     return EMER;         // 已在 backup 上仍失败 → 急停
    // SUCCESS / NO_NEED 继续

    // 可视化更新后的 yaw + pos 轨迹
    ros_ptr_->vizYawTraj(exp_traj_info.posTraj(), exp_traj_info.yawTraj());

    /* ---- 2) 重生成 backup 轨迹 ---- */
    BackupTraj back_traj_info;
    TimeConsuming t_back("t_back", false);
    RET_CODE back_ret_code = generateBackupTrajectory(exp_traj_info, back_traj_info);
    time_consuming_[GENERATE_BACK_TRAJ] = t_back.stop();

    /* ---- 3) 时间预算检查 ---- */
    // 整个 ReplanOnce 必须在 0.9 * replan_forward_dt 内完成
    // 否则下一次 cmdCallback 可能会跑到旧轨迹的尾部，导致控制不连续
    double replan_dt = replan_total_t.stop();
    if (replan_dt > cfg_.replan_forward_dt * 0.9) {
        return FAILED;
    }

    /* ---- 4) 提交结果（与 PlanFromRest 类似的三种情况） ---- */
    if (back_ret_code == SUCCESS) {
        cmd_traj_info_.setTrajectory(exp_traj_info, back_traj_info);
        last_exp_traj_info_ = exp_traj_info;
        robot_on_backup_traj_ = false;
        gi_.new_goal = false;
        ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), cmd_traj_info_.getBackupTrajStartTT());
        latest_replan.setRetCode(SUPER_SUCCESS_WITH_BACKUP);
        return SUCCESS;
    }
    else if (back_ret_code == NO_NEED) {
        // 备份没有意义（exp 太短）
        robot_on_backup_traj_ = false;
        last_exp_traj_info_ = exp_traj_info;
        gi_.new_goal = false;
        ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1);
        latest_replan.setRetCode(SUPER_SUCCESS_NO_BACKUP);
        return SUCCESS;
    }
    else if (back_ret_code == FINISH) {
        // exp 全部已知自由 → 提交单轨迹
        cmd_traj_info_.setTrajectory(exp_traj_info);
        last_exp_traj_info_ = exp_traj_info;
        robot_on_backup_traj_ = false;
        gi_.new_goal = false;
        ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1);
        latest_replan.setRetCode(SUPER_SUCCESS_NO_BACKUP);
        return SUCCESS;
    }
    return FAILED;
}
```

---

## 命令解算 getOneCommandFromTraj（L322-370）

下游控制器/MPC 通过这个接口拿当前期望的位置-速度-加速度-jerk（PVAJ）+ yaw + yaw_rate。

```cpp
void SuperPlanner::getOneCommandFromTraj(StatePVAJ &pvaj,
                                         double &yaw, double &yaw_dot,
                                         bool &on_backup_traj,
                                         bool &traj_finish) {
    cmd_traj_info_.lock();
    const double &cur_t = ros_ptr_->getSimTime();
    const double &cmd_start_WT = cmd_traj_info_.getStartWallTime();   // 轨迹开始的墙钟时间
    const double &total_dur = cmd_traj_info_.getTotalDuration();      // 总时长

    traj_finish = (cur_t - cmd_start_WT) > total_dur;
    const double &eval_t = traj_finish ? total_dur : (cur_t - cmd_start_WT);

    // 关键：判断当前 eval_t 落在 exp 段还是 backup 段
    // 如果 eval_t > backup_traj_start_TT，则在 backup 上
    robot_on_backup_traj_ = cmd_traj_info_.isTTOnBackupTraj(eval_t);
    on_backup_traj = robot_on_backup_traj_;

    // 取出 4×3 的 PVAJ 矩阵（位置/速度/加速度/jerk）
    pvaj = cmd_traj_info_.posTraj().getState(eval_t);

    // yaw 单独维护（独立的 1D 多项式）
    yaw     = cmd_traj_info_.getYaw(eval_t)[0];
    yaw_dot = cmd_traj_info_.getYawRate(eval_t)[0];

    // 防 NaN：异常时保持上一个 yaw、把 yaw_dot 置零
    static double last_yaw = robot_state_.yaw;
    if (isnan(yaw))     { yaw = last_yaw;  yaw_dot = 0; }
    else                  last_yaw = yaw;
    if (isnan(yaw_dot)) yaw_dot = 0;

    cmd_traj_info_.unlock();
}
```

> 注意：SUPER 的位置指令包含 **jerk**（J 分量），与 EGO 只输出 PVA（不带 jerk）相比信息更全——这是因为 MINCO 是 7 阶多项式，可以稳定取到 4 阶导数（即 jerk）。下游 MPC 控制器（NLMPC）需要 jerk 用于前馈。

---

## 与后续模块的关系

```
SuperPlanner::PlanFromRest / ReplanOnce
    │
    ├── generateExpTraj (super_planner.cpp L379-826)            ← 见 4_generate_exp_traj.md
    │      │
    │      ├── PathSearch (L1106-1201)                          ← Astar 包装器
    │      │      └─ astar_ptr_->pointToPointPathSearch / escapePathSearch
    │      │
    │      ├── cg_ptr_->SearchPolytopeOnPath                    ← 见 6_corridor_ciri.md
    │      │
    │      ├── exp_traj_opt_->optimize                          ← 见 7_minco_trajopt.md
    │      │
    │      └── yaw_traj_opt_->optimize                          ← 1D yaw 多项式
    │
    └── generateBackupTrajectory (super_planner.cpp L828-1085)  ← 见 5_generate_backup_traj.md
           │
           ├── 沿 exp 轨迹采样 → 找首个 line-of-sight 失效点
           ├── cg_ptr_->GeneratePolytopeFromLine（单段 SFC）
           ├── back_traj_opt_->optimize（含 ts 自由度）
           └── yaw_traj_opt_->optimize
```
