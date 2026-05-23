# ego_replan_fsm.cpp —— EGOReplanFSM 初始化与回调
## 文件路径
`official/Fast-Drone-250/src/planner/plan_manage/src/ego_replan_fsm.cpp`

> 本文件包含 EGOReplanFSM 类的所有方法。分两篇文档：init() 和回调为一篇，execFSMCallback + getLocalTarget 为另一篇。

---

## init() 函数 —— EGO 的大脑启动程序

### 整体流程概览

`init()` 是 EGO 上电后执行的第一段逻辑，完成以下事情：

```
参数读取 → 模块初始化 → 订阅/发布/定时器建立 → 目标源选择 → 等待触发
```

---

### L7-33：参数读取

```cpp
// EGOReplanFSM::init() —— 参数读取阶段
void EGOReplanFSM::init(ros::NodeHandle &nh)
{
    // 状态机内部状态初始化
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;  // 一上电先进入 INIT 状态
    have_target_ = false;                  // 还没有目标点
    have_odom_ = false;                    // 还没有里程计数据
    have_recv_pre_agent_ = false;          // 多机模式：还没收到前机轨迹

    /* ========== FSM 核心参数 ========== */
    // flight_type: 目标点来源模式
    //   1 = MANUL_TARGET：手动通过 RViz 2D Nav Goal 发送（/move_base_simple/goal）
    //   2 = PRESET_TARGET：从 launch 参数读取预设航点序列
    nh.param("fsm/flight_type", target_type_, -1);

    // thresh_replan_time：轨迹执行多久后允许重规划（秒）
    // 在 EXEC_TRAJ 状态下，如果当前时间 > thresh_replan_time，触发 REPLAN_TRAJ
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);

    // thresh_no_replan_meter：距离目标多近时停止重规划（米）
    // 用于判断"是否已经快到终点了，不需要再修正"
    nh.param("fsm/thresh_no_replan_meter", no_replan_thresh_, -1.0);

    // planning_horizon：局部规划的"前视距离"（米）—— EGO 最核心参数之一
    // 表示从当前无人机位置往前看多远来设置局部目标
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);

    // planning_horizen_time：时间域前视窗口（秒）
    // planning_horizon 的时间版本，和空间版本配合使用
    nh.param("fsm/planning_horizen_time", planning_horizen_time_, -1.0);

    // emergency_time：发现障碍后多久触发急停（秒）
    nh.param("fsm/emergency_time", emergency_time_, 1.0);

    // realworld_experiment：实机/仿真模式标志
    //   false（仿真）：启动时自动 have_trigger_ = true，不需要外部触发
    //   true（实机）：必须收到 /traj_start_trigger 才 have_trigger_ = true
    nh.param("fsm/realworld_experiment", flag_realworld_experiment_, false);
    have_trigger_ = !flag_realworld_experiment_;  // 仿真=true，实机=false

    // fail_safe：急停后是否尝试重新生成轨迹
    nh.param("fsm/fail_safe", enable_fail_safe_, true);

    /* ========== 预设航点读取（flight_type=2 时使用）========== */
    // 从 launch 参数读取：fsm/waypoint0_x/y/z, waypoint1_x/y/z, ...
    nh.param("fsm/waypoint_num", waypoint_num_, -1);  // 有几个航点
    for (int i = 0; i < waypoint_num_; i++)
    {
        // 参数名格式：fsm/waypoint0_x, fsm/waypoint0_y, fsm/waypoint0_z
        nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
        nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
        nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }
}
```

### L35-40：核心模块初始化

```cpp
/* ========== 核心规划模块初始化 ========== */

// 可视化模块：用于在 RViz 中显示轨迹、目标点、地图等
// PlanningVisualization 会发布 markers 到 /planning/data_display
visualization_.reset(new PlanningVisualization(nh));

// 规划管理器：管理 GridMap（地图）和 BsplineOptimizer（轨迹优化）
planner_manager_.reset(new EGOPlannerManager);

// 初始化规划模块：读取 manager/* 和 grid_map/* 参数，建立地图，建立优化器
planner_manager_->initPlanModules(nh, visualization_);

// 将当前轨迹传递给优化器（初始化时为空）
planner_manager_->deliverTrajToOptimizer();

// 将无人机编号传递给优化器（多机编队时需要）
planner_manager_->setDroneIdtoOpt();
```

### L42-44：定时器建立（驱动 FSM 运转的核心）

```cpp
/* ========== 定时器：驱动整个 FSM 循环 ========== */
// 两个定时器是 EGO 能"持续运转"的关键

// exec_timer_：每 0.01 秒（100Hz）执行一次 execFSMCallback
//   → 驱动状态机切换、轨迹执行、重规划判定
// 注意：如果回调执行时间超过 0.01s，会导致定时器堆积，原版用 stop()/start() 避免阻塞
exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);

// safety_timer_：每 0.05 秒（20Hz）执行一次 checkCollisionCallback
//   → 检查当前轨迹是否与障碍物碰撞、深度图是否丢失
//   → 频率比 exec_timer_ 低，因为碰撞检查计算量较大
safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);
```

### L46-61：订阅器与发布器

```cpp
/* ========== 订阅器 ========== */

// odom_sub_：接收无人机里程计
//   话题名通过 launch remap，常见映射：
//     仿真：/odom_world 或 /pcl_render_node/odom
//     实机：/Odom_high_freq（FAST_LIO 高频里程计）
//   回调：更新 odom_pos_, odom_vel_, odom_orient_，设置 have_odom_ = true
odom_sub_ = nh.subscribe("odom_world", 1, &EGOReplanFSM::odometryCallback, this);

/* ---- 多机模式订阅（drone_id >= 1 时）---- */
// swarm_trajs_sub_：订阅前一架无人机的轨迹
//   话题名格式：/drone_{id-1}_planning/swarm_trajs
//   回调：swarmTrajsCallback，存储前机轨迹并在需要时触发重规划
if (planner_manager_->pp_.drone_id >= 1)
{
    string sub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id - 1)
                            + string("_planning/swarm_trajs");
    swarm_trajs_sub_ = nh.subscribe(sub_topic_name.c_str(), 10,
                                   &EGOReplanFSM::swarmTrajsCallback, this,
                                   ros::TransportHints().tcpNoDelay());  // tcpNoDelay 禁用 Nagle 算法，降低延迟
}

/* ========== 发布器 ========== */

// swarm_trajs_pub_：发布自己的轨迹给后一架无人机
//   话题名格式：/drone_{id}_planning/swarm_trajs
string pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id)
                        + string("_planning/swarm_trajs");
swarm_trajs_pub_ = nh.advertise<traj_utils::MultiBsplines>(pub_topic_name.c_str(), 10);

// broadcast_bspline_pub_：广播轨迹给所有无人机（用于广播式多机协调）
broadcast_bspline_pub_ = nh.advertise<traj_utils::Bspline>(
    "planning/broadcast_bspline_from_planner", 10);

// broadcast_bspline_sub_：接收其他无人机的广播轨迹
broadcast_bspline_sub_ = nh.subscribe("planning/broadcast_bspline_to_planner", 100,
                                     &EGOReplanFSM::BroadcastBsplineCallback, this,
                                     ros::TransportHints().tcpNoDelay());

// bspline_pub_：发布 B-spline 轨迹给 traj_server（核心输出）
//   traj_server 接收后转换为 PositionCommand 发给控制器
bspline_pub_ = nh.advertise<traj_utils::Bspline>("planning/bspline", 10);

// data_disp_pub_：发布可视化数据（调试用）
data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);
```

### L62-89：目标源选择与等待

```cpp
/* ========== 目标源选择（flight_type 决定目标点从哪来）========== */

if (target_type_ == TARGET_TYPE::MANUAL_TARGET)   // flight_type = 1
{
    // 方式 A：手动目标
    // 订阅 RViz 的 "2D Nav Goal"，用户每次在 RViz 点击目标点触发一次回调
    // 回调中会调用 planNextWaypoint() 开始规划
    waypoint_sub_ = nh.subscribe("/move_base_simple/goal", 1,
                                  &EGOReplanFSM::waypointCallback, this);
}
else if (target_type_ == TARGET_TYPE::PRESET_TARGET)  // flight_type = 2
{
    // 方式 B：预设航点序列（从 launch 参数读取）
    // 订阅触发信号 /traj_start_trigger，收一次就触发 have_trigger_ = true
    trigger_sub_ = nh.subscribe("/traj_start_trigger", 1,
                                  &EGOReplanFSM::triggerCallback, this);

    // 等待 1 秒让其他节点启动完成
    ROS_INFO("Wait for 1 second.");
    int count = 0;
    while (ros::ok() && count++ < 1000) { ros::spinOnce(); ros::Duration(0.001).sleep(); }

    // 等待遥控器触发（在实机模式下必须有 RC 触发信号才起飞）
    ROS_WARN("Waiting for trigger from [n3ctrl] from RC");
    while (ros::ok() && (!have_odom_ || !have_trigger_))
    {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
    }

    // 读取 launch 中的预设航点（waypoint0~4）
    readGivenWps();
}
```

---

## 关键回调函数

### odometryCallback —— 里程计数据更新（L225-243）

```cpp
// EGOReplanFSM::odometryCallback —— 里程计回调
// 这个回调是 EGO 所有状态判断的前提：没有 odom，状态机一步也走不了
void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
{
    // 读取当前无人机位置（全局地图坐标系）
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    // 读取当前无人机速度（ENU 坐标系）
    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    // 读取当前无人机姿态（四元数）
    // 用于保留当前朝向，EGO 本身不在优化中直接使用姿态
    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    // 标记：已经有有效里程计了——这是 execFSMCallback 中 INIT→WAIT_TARGET 的前提条件
    have_odom_ = true;
}
```

### waypointCallback —— 手动目标点回调（L211-223）

```cpp
// EGOReplanFSM::waypointCallback —— 接收 RViz 2D Nav Goal
// 每次用户在 RViz 点击目标点，这个函数就会被调用一次
void EGOReplanFSM::waypointCallback(const geometry_msgs::PoseStampedPtr &msg)
{
    // 安全过滤：z < -0.1 的目标被忽略（无效高度）
    if (msg->pose.position.z < -0.1)
        return;

    cout << "Triggered!" << endl;

    // 记录起始点：当前无人机位置（作为轨迹起点）
    init_pt_ = odom_pos_;

    // 【重要】构建终点：从 RViz 消息中读取 X 和 Y，Z 被硬编码为 1.0
    //   这是 EGO 中唯一一个 Z 轴被写死的地方！
    //   如果需要 3D 航点，只需要把 1.0 改成 msg->pose.position.z 即可
    Eigen::Vector3d end_wp(msg->pose.position.x,
                            msg->pose.position.y,
                            1.0);  // ← Z 轴硬编码，修改这里即可支持 3D 目标

    // 交给 planNextWaypoint() 开始规划
    planNextWaypoint(end_wp);
}
```

### planNextWaypoint —— 触发全局轨迹规划（L158-202）

```cpp
// EGOReplanFSM::planNextWaypoint —— 触发从当前点到目标点的全局规划
// 这是 EGO "收到目标"后执行的第一段真正的规划逻辑
void EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp)
{
    bool success = false;

    // 调用 PlannerManager 的 planGlobalTraj：
    //   输入：当前起点(位置+速度+加速度)、目标点(位置+速度+加速度)
    //   PlannerManager 内部会：
    //     1. 生成全局参考路径（A* 或多项式）
    //     2. 调用 reboundReplan() 做局部 B-spline 优化
    //     3. 发布 bspline 轨迹到 planning/bspline
    success = planner_manager_->planGlobalTraj(
        odom_pos_,       // 当前无人机位置
        odom_vel_,       // 当前无人机速度
        Eigen::Vector3d::Zero(),  // 当前加速度（设为0）
        next_wp,         // 目标点位置
        Eigen::Vector3d::Zero(), // 目标速度（设为0，到达时速度为0）
        Eigen::Vector3d::Zero()  // 目标加速度（设为0）
    );

    if (success)
    {
        end_pt_ = next_wp;  // 记录终点

        /* ---- 可视化：生成并显示全局轨迹 ---- */
        // 每隔 0.1s 取一个点，生成全局轨迹的离散可视化
        constexpr double step_size_t = 0.1;
        int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
        vector<Eigen::Vector3d> gloabl_traj(i_end);
        for (int i = 0; i < i_end; i++)
        {
            // evaluate() 根据时间参数 t 从全局轨迹中取位置
            gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
        }

        end_vel_.setZero();
        have_target_ = true;      // 标记：有有效目标了
        have_new_target_ = true;  // 标记：有新目标（用于 BsplineOptimizer 中 fitness 权重调整）

        /* ---- 状态机切换 ---- */
        if (exec_state_ == WAIT_TARGET)
        {
            // 第一次收到目标：INIT→WAIT_TARGET→GEN_NEW_TRAJ
            changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
        }
        else
        {
            // 已有轨迹在执行：等待进入 EXEC_TRAJ 状态，然后切换到 REPLAN_TRAJ
            // 循环等待是因为状态切换是异步的（下次 execFSMCallback 才生效）
            while (exec_state_ != EXEC_TRAJ)
            {
                ros::spinOnce();
                ros::Duration(0.001).sleep();
            }
            changeFSMExecState(REPLAN_TRAJ, "TRIG");
        }

        // 发布可视化（RViz 中看到绿色的全局轨迹）
        visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
        ROS_ERROR("Unable to generate global trajectory!");
        // 规划失败：留在当前状态，等待下一次重试（下次 execFSMCallback 会再次尝试）
    }
}
```

---

## 状态机状态定义（changeFSMExecState, L405-417）

```cpp
// EGOReplanFSM::changeFSMExecState —— 状态切换函数
// 每当状态发生变化时调用，打印状态转换日志
void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
{
    // continously_called_times_ 统计同一个状态被连续调用了多少次
    // 如果调用次数过多（比如一直失败），可以用来做预警
    if (new_state == exec_state_)
        continously_called_times_++;
    else
        continously_called_times_ = 1;  // 切换到新状态，重置计数

    // 状态名称数组（用于日志打印）
    static string state_str[8] = {
        "INIT",         // 0: 初始化，等待 odom
        "WAIT_TARGET",  // 1: 等待目标点或触发信号
        "GEN_NEW_TRAJ", // 2: 生成新轨迹（第一次规划）
        "REPLAN_TRAJ",  // 3: 重规划（基于已有轨迹修正）
        "EXEC_TRAJ",     // 4: 执行轨迹中（正常飞行）
        "EMERGENCY_STOP",// 5: 急停（发现碰撞或失去深度）
        "SEQUENTIAL_START" // 6: 多机模式顺序启动
    };

    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    // 打印状态转换日志：[调用者] from 旧状态 to 新状态
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}
```
