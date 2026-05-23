# traj_server.cpp —— 轨迹转指令的桥梁
## 文件路径
`official/Fast-Drone-250/src/planner/plan_manage/src/traj_server.cpp`

> `traj_server` 是 EGO 规划器与 PX4/控制器之间的桥梁。它接收 EGO 发布的 B-spline 轨迹，实时解算出 quadrotor_msgs::PositionCommand 发给下级控制器。

---

## 核心理解

```
EGO Planner (BsplineOptimizer)
    │ 发布: planning/bspline (traj_utils::Bspline)
    ▼
traj_server (本文件)
    │ 订阅: planning/bspline
    │ 每 0.01s 执行 cmdCallback
    │ 解算: 位置+速度+加速度+yaw
    ▼ 发布: /position_cmd (quadrotor_msgs::PositionCommand)
    │
    ▼
PX4 Controller / traj_server 内置控制器
    │ 接收: PositionCommand
    ▼
执行机构 (电机/PWM)
```

`traj_server` 的核心工作：在每个控制周期，根据当前时刻 `t` 从 B-spline 轨迹中**插值**出位置、速度、加速度，并计算偏航角（Yaw）。

---

## main 函数 —— 初始化（L233-265）

```cpp
int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_server");
    ros::NodeHandle nh("~");

    // 订阅 EGO 发布的 B-spline 轨迹
    ros::Subscriber bspline_sub = nh.subscribe("planning/bspline", 10, bsplineCallback);

    // 发布 quadrotor_msgs::PositionCommand 给下游控制器
    pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);

    // 定时器：每 0.01s（100Hz）执行一次 cmdCallback
    ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

    /* 控制参数（默认为0，表示下游控制器自己计算增益） */
    cmd.kx[0] = pos_gain[0];
    cmd.kx[1] = pos_gain[1];
    cmd.kx[2] = pos_gain[2];
    cmd.kv[0] = vel_gain[0];
    cmd.kv[1] = vel_gain[1];
    cmd.kv[2] = vel_gain[2];

    // 从 launch 参数读取前视时间（控制无人机朝哪个方向飞）
    nh.param("traj_server/time_forward", time_forward_, -1.0);
    last_yaw_ = 0.0;
    last_yaw_dot_ = 0.0;

    ros::Duration(1.0).sleep();  // 等待 1 秒确保其他节点就绪
    ROS_WARN("[Traj server]: ready.");

    ros::spin();  // 进入事件循环
    return 0;
}
```

**`time_forward_` 参数**：无人机"朝哪个方向看"。设为正值时，无人机朝向是轨迹上前方某一点的方向，而非当前运动方向。

---

## bsplineCallback —— 轨迹接收与解析（L27-69）

```cpp
// 订阅 EGO 发布的 B-spline 轨迹，收到后解析并存入 traj_ 数组
void bsplineCallback(traj_utils::BsplineConstPtr msg)
{
    // 1. 解析控制点位置（3 x N 矩阵，每列是一个控制点坐标）
    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
    {
        pos_pts(0, i) = msg->pos_pts[i].x;
        pos_pts(1, i) = msg->pos_pts[i].y;
        pos_pts(2, i) = msg->pos_pts[i].z;
    }

    // 2. 解析节点向量（knots）
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i)
    {
        knots(i) = msg->knots[i];
    }

    // 3. 用控制点和节点向量构造 B-spline 位置轨迹
    UniformBspline pos_traj(pos_pts, msg->order, 0.1);  // order=3 表示三次 B-spline
    pos_traj.setKnot(knots);                             // 设置节点向量

    // 4. 解析时间信息
    start_time_ = msg->start_time;   // 轨迹开始时刻（ROS Time）
    traj_id_ = msg->traj_id;         // 轨迹编号（用于轨迹追踪）

    // 5. 构建速度、加速度轨迹（通过求导）
    traj_.clear();
    traj_.push_back(pos_traj);              // traj_[0] = 位置
    traj_.push_back(traj_[0].getDerivative());  // traj_[1] = 速度（一阶导）
    traj_.push_back(traj_[1].getDerivative());  // traj_[2] = 加速度（二阶导）

    traj_duration_ = traj_[0].getTimeSum();  // 轨迹总时长

    receive_traj_ = true;  // 标记：已有有效轨迹，cmdCallback 可以开始发布了
}
```

---

## cmdCallback —— 指令发布（定时器驱动，L163-231）

### 整体流程

```cpp
// 每 0.01s（100Hz）执行一次：从轨迹中取当前位置/速度/加速度，发给控制器
void cmdCallback(const ros::TimerEvent &e)
{
    /* 如果还没收到轨迹，直接返回 */
    if (!receive_traj_)
        return;

    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - start_time_).toSec();  // 当前在轨迹中的时间参数

    Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()),
                    acc(Eigen::Vector3d::Zero()), pos_f;
    std::pair<double, double> yaw_yawdot(0, 0);

    static ros::Time time_last = ros::Time::now();

    /* ---- 分支1：轨迹执行中 ---- */
    if (t_cur < traj_duration_ && t_cur >= 0.0)
    {
        // 从 B-spline 上插值出 t_cur 时刻的位置、速度、加速度
        pos = traj_[0].evaluateDeBoorT(t_cur);      // 位置
        vel = traj_[1].evaluateDeBoorT(t_cur);        // 速度
        acc = traj_[2].evaluateDeBoorT(t_cur);        // 加速度

        // 计算偏航角（Yaw）和偏航角速度（Yaw_dot）
        yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);

        // 计算前向一点的位置（用于可视化，不用于控制）
        double tf = min(traj_duration_, t_cur + 2.0);
        pos_f = traj_[0].evaluateDeBoorT(tf);
    }
    /* ---- 分支2：轨迹执行完毕 → 悬停 ---- */
    else if (t_cur >= traj_duration_)
    {
        // 悬停在轨迹终点
        pos = traj_[0].evaluateDeBoorT(traj_duration_);
        vel.setZero();
        acc.setZero();
        yaw_yawdot.first = last_yaw_;    // 保持最后一个偏航角
        yaw_yawdot.second = 0;            // 角速度为 0
        pos_f = pos;
        return;  // 注意：这里 return 了，不再发布（悬停时可能不发指令）
    }
    /* ---- 分支3：非法时间参数 ---- */
    else
    {
        cout << "[Traj server]: invalid time." << endl;
    }

    time_last = time_now;

    /* ---- 填充 PositionCommand 消息 ---- */
    cmd.header.stamp = time_now;
    cmd.header.frame_id = "world";

    // 轨迹状态：READY 表示这是由轨迹生成的指令
    cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    cmd.trajectory_id = traj_id_;

    cmd.position.x = pos(0);
    cmd.position.y = pos(1);
    cmd.position.z = pos(2);

    cmd.velocity.x = vel(0);
    cmd.velocity.y = vel(1);
    cmd.velocity.z = vel(2);

    cmd.acceleration.x = acc(0);
    cmd.acceleration.y = acc(1);
    cmd.acceleration.z = acc(2);

    cmd.yaw = yaw_yawdot.first;
    cmd.yaw_dot = yaw_yawdot.second;

    last_yaw_ = cmd.yaw;

    // 发布给下游控制器
    pos_cmd_pub.publish(cmd);
}
```

---

## calculate_yaw —— 轨迹导向偏航角计算（L71-161）

### 核心思想

偏航角（Yaw）控制无人机朝向。原版 EGO 没有独立的 yaw 规划，而是让无人机始终"朝轨迹前方看"：

```
当前时刻 t_cur:
无人机 ────────▶ (朝向 local_target)
                   ↑
            朝轨迹前方 planning_horizon 处看

而不是：朝向当前速度方向
```

### 详细逻辑（L71-161）

```cpp
// 计算偏航角 = atan2(dir_y, dir_x)，dir = 轨迹上前方点 - 当前位置
// 返回 pair<yaw, yaw_dot>
std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos,
                                        ros::Time &time_now, ros::Time &time_last)
{
    constexpr double PI = 3.1415926;
    constexpr double YAW_DOT_MAX_PER_SEC = PI;  // 最大偏航角速度：PI rad/s = 180°/s
    std::pair<double, double> yaw_yawdot(0, 0);
    double yaw = 0;
    double yawdot = 0;

    // L80：计算期望朝向
    // 如果 t_cur + time_forward_ 还在轨迹范围内 → 朝轨迹前方 time_forward_ 处看
    // 否则 → 朝轨迹终点看
    Eigen::Vector3d dir = (t_cur + time_forward_ <= traj_duration_)
                              ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos
                              : traj_[0].evaluateDeBoorT(traj_duration_) - pos;

    // 如果距离太近（< 0.1m），保持当前朝向
    double yaw_temp = dir.norm() > 0.1
                          ? atan2(dir(1), dir(0))   // 朝向 = atan2(y差, x差)
                          : last_yaw_;              // 保持上一时刻朝向

    // L82：计算本周期最大允许偏航角变化量
    // max_yaw_change = YAW_DOT_MAX_PER_SEC × dt（秒）
    // 用于限幅：防止偏航角突变
    double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).toSec();

    /* ---- 核心：偏航角限幅 + 跳变处理 ---- */
    // 问题：atan2 返回范围是 [-π, π]
    // 如果当前 yaw=-170°，目标 yaw=170°，atan2 直接算会得到 -170°
    // 但实际上只需要转 20°（正向）而不是绕 340°（反向）
    // 所以需要处理 [-π, π] 的跳变
    // 解决方案：限制单步最大变化不超过 max_yaw_change，并在跳变时选最优方向

    // 分支1：yaw_temp - last_yaw_ > π（正向跳变，如 -170° → 170°）
    if (yaw_temp - last_yaw_ > PI)
    {
        if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
        {
            // 应该正向转：yaw = last_yaw_ + max_yaw_change
            yaw = last_yaw_ - max_yaw_change;
            if (yaw < -PI) yaw += 2 * PI;
            yawdot = -YAW_DOT_MAX_PER_SEC;
        }
        else
        {
            yaw = yaw_temp;
            if (yaw - last_yaw_ > PI)
                yawdot = -YAW_DOT_MAX_PER_SEC;
            else
                yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
        }
    }
    // 分支2：yaw_temp - last_yaw_ < -π（反向跳变，如 170° → -170°）
    else if (yaw_temp - last_yaw_ < -PI)
    {
        if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
        {
            // 应该反向转：yaw = last_yaw_ + max_yaw_change
            yaw = last_yaw_ + max_yaw_change;
            if (yaw > PI) yaw -= 2 * PI;
            yawdot = YAW_DOT_MAX_PER_SEC;
        }
        else
        {
            yaw = yaw_temp;
            if (yaw - last_yaw_ < -PI)
                yawdot = YAW_DOT_MAX_PER_SEC;
            else
                yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
        }
    }
    // 分支3：无跳变，直接限幅
    else
    {
        if (yaw_temp - last_yaw_ < -max_yaw_change)
        {
            yaw = last_yaw_ - max_yaw_change;
            if (yaw < -PI) yaw += 2 * PI;
            yawdot = -YAW_DOT_MAX_PER_SEC;
        }
        else if (yaw_temp - last_yaw_ > max_yaw_change)
        {
            yaw = last_yaw_ + max_yaw_change;
            if (yaw > PI) yaw -= 2 * PI;
            yawdot = YAW_DOT_MAX_PER_SEC;
        }
        else
        {
            yaw = yaw_temp;
            if (yaw - last_yaw_ > PI)
                yawdot = -YAW_DOT_MAX_PER_SEC;
            else if (yaw - last_yaw_ < -PI)
                yawdot = YAW_DOT_MAX_PER_SEC;
            else
                yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
        }
    }

    /* ---- 简单一阶低通滤波（LPF）---- */
    // 目的：让偏航角变化更平滑，防止抖动
    if (fabs(yaw - last_yaw_) <= max_yaw_change)
        yaw = 0.5 * last_yaw_ + 0.5 * yaw;  // 50% 旧值 + 50% 新值

    yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;

    last_yaw_ = yaw;
    last_yaw_dot_ = yawdot;

    yaw_yawdot.first = yaw;
    yaw_yawdot.second = yawdot;

    return yaw_yawdot;
}
```

### calculate_yaw 的关键参数

| 参数 | 值 | 说明 |
|---|---|---|
| `YAW_DOT_MAX_PER_SEC` | π rad/s = 180°/s | 最大偏航角速度（硬限制） |
| `time_forward_` | 从 launch 读取，默认 -1.0 | 朝轨迹前方多远看（负值时朝向当前速度方向） |

**`time_forward_` 的影响**：
- `time_forward_ > 0`（如 1.0）：朝轨迹上前方 1 秒处的点看
- `time_forward_ < 0`（如 -1.0）：朝当前速度方向看
- `time_forward_ = 0`：朝轨迹上当前点看（几乎不用）

**偏航角跳变处理示例**：
```
last_yaw_ = -170°   (弧度: -2.967)
yaw_temp  = +170°   (弧度: +2.967)
差值 = +340° > π

正确选择：转 -20° 而不是 +340°
（通过检查差值-2π = -20° 是否 < max_yaw_change）
```

---

## quadrotor_msgs::PositionCommand 消息格式

| 字段 | 类型 | 说明 |
|---|---|---|
| `header` | std_msgs/Header | 时间戳+坐标系 |
| `position` | geometry_msgs/Vector3 | 期望位置 (x,y,z) |
| `velocity` | geometry_msgs/Vector3 | 期望速度 (vx,vy,vz) |
| `acceleration` | geometry_msgs/Vector3 | 期望加速度 (ax,ay,az) |
| `jerk` | geometry_msgs/Vector3 | 期望加加速度（未使用，设为0） |
| `yaw` | double | 期望偏航角（弧度） |
| `yaw_dot` | double | 期望偏航角速度（弧度/秒） |
| `kx[3]` | double[3] | 位置环增益（下游控制器用） |
| `kv[3]` | double[3] | 速度环增益（下游控制器用） |
| `trajectory_id` | int | 轨迹编号 |
| `trajectory_flag` | int | 轨迹状态标志（READY=1） |

---

## 全局变量一览

| 变量 | 类型 | 作用域 | 说明 |
|---|---|---|---|
| `pos_cmd_pub` | ros::Publisher | 全局 | 发布 PositionCommand |
| `receive_traj_` | bool | 全局 | 是否已收到有效轨迹 |
| `traj_` | vector<UniformBspline> | 全局 | 位置/速度/加速度三条 B-spline |
| `traj_duration_` | double | 全局 | 轨迹总时长（秒） |
| `start_time_` | ros::Time | 全局 | 轨迹开始时刻 |
| `traj_id_` | int | 全局 | 轨迹编号 |
| `last_yaw_` | double | 全局 | 上一时刻偏航角 |
| `last_yaw_dot_` | double | 全局 | 上一时刻偏航角速度 |
| `time_forward_` | double | 全局 | 前视时间（偏航角计算用） |
| `cmd` | quadrotor_msgs::PositionCommand | 全局 | 待发布的指令消息 |
