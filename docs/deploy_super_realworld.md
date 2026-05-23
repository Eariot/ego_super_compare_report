# SUPER 实机部署测试说明

> 适用版本：`compare/SUPER/`（HKU-MARS/SUPER 主干）。
> 目标平台：搭载 PX4 飞控（Pixhawk 6X / CUAV V5+ 等）+ NVIDIA Jetson Orin / NX 机载计算机的高动态四旋翼。
> 运行栈：Ubuntu 20.04 + ROS Noetic + MAVROS 1.x + PX4 v1.14（推荐）。
> 传感器：Livox Mid-360 / Avia 激光雷达 + IMU（推荐配 FAST-LIO2）。

本文档分七部分：
1. 硬件与软件准备
2. 编译与依赖
3. 输入 / 输出话题与坐标系
4. 参数配置（基于 click_smooth_ros1.yaml 改）
5. PX4 桥接：双轨迹场景下如何"安全地"喂 setpoint
6. 实机测试流程
7. 常见问题与调参指南

---

## 1. 硬件与软件准备

### 1.1 推荐硬件清单

| 模块 | 型号示例 | 必要性 | 备注 |
|---|---|---|---|
| 飞控 | Pixhawk 6X / CUAV V5+ / Holybro Pix32 v6 | 必需 | PX4 v1.14 支持 NLMPC，可直接吃 jerk |
| 机载计算机 | Jetson Orin NX 16GB / Orin AGX | **强烈推荐** | SUPER 单帧 30-90 ms，Orin Nano 8GB 偏紧 |
| 激光雷达 | Livox Mid-360（推荐）/ Avia / Horizon | 必需 | SUPER 是为激光设计的，深度相机不推荐 |
| LiDAR-IMU SLAM | FAST-LIO2 / Point-LIO | 必需 | 输出 200Hz 高频 odom + 同步点云 |
| 串口 | TELEM2 (`/dev/ttyTHS0` 或 `/dev/ttyUSB0`) | 必需 | 飞控 ↔ 机载机 MAVLink |
| 遥控器 | 至少 2 个三档开关 | 必需 | 安全开关 + OFFBOARD 切换 |

> SUPER 论文实测平台：Livox Mid-360 + Pixhawk + Orin NX，机架轴距 280-350mm。深度相机方案不适合 SUPER（视野窄、点云密度不够，会让 ROG-Map 大量误识为未知，触发频繁 backup）。

### 1.2 软件依赖

```bash
sudo apt install ros-noetic-desktop-full \
                 ros-noetic-mavros ros-noetic-mavros-extras \
                 libeigen3-dev libpcl-dev libomp-dev \
                 libfmt-dev libdw-dev binutils-dev   # backward-cpp 依赖

# MAVROS 数据集
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh

# Livox SDK + ROS driver（如果用 Livox LiDAR）
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2 && mkdir build && cd build && cmake .. && make -j4 && sudo make install

git clone https://github.com/Livox-SDK/livox_ros_driver2.git ~/ros_ws/src/livox_ros_driver2
cd ~/ros_ws && catkin_make
```

### 1.3 关键自带消息包
- `mars_quadrotor_msgs`（在 `SUPER/mars_uav_sim/mars_quadrotor_msgs/`）：定义 `PolynomialTrajectory`、`PositionCommand`
- `super_planner` 自带 `cereal`、`fmt`、`backward-cpp` 头文件，**不需要单独装**
- `rog_map` 是独立 ROS package，可单独使用

---

## 2. 编译与依赖

### 2.1 拉源码

```bash
mkdir -p ~/super_ws/src
cd ~/super_ws/src
git clone https://github.com/Eariot/ego_super_compare_report.git compare

# SUPER 的所有 ROS 包都在 compare/SUPER/ 下，建立符号链接
ln -s compare/SUPER/super_planner ./
ln -s compare/SUPER/rog_map ./
ln -s compare/SUPER/mission_planner ./
ln -s compare/SUPER/mars_uav_sim ./

# 如果用 LIO，把 SLAM 也丢进 src
git clone https://github.com/hku-mars/FAST_LIO.git
```

### 2.2 编译

```bash
cd ~/super_ws
catkin_make -DCMAKE_BUILD_TYPE=Release -j6
source devel/setup.bash
echo "source ~/super_ws/devel/setup.bash" >> ~/.bashrc
```

> 提示：SUPER 第一次编译会比较慢（~10 分钟，因为 cereal、MINCO、CIRI 都是模板重灾区）。Orin AGX `-j8` 即可；Orin Nano 别超 `-j4`。

---

## 3. 输入 / 输出话题与坐标系

### 3.1 SUPER 节点（`fsm_node`）输入

| 话题（YAML 默认） | 类型 | 必要性 | 说明 |
|---|---|---|---|
| `/cloud_registered` | `sensor_msgs/PointCloud2` | **必需** | LIO 输出的去畸变 + 配准点云，世界系 |
| `/lidar_slam/odom` | `nav_msgs/Odometry` | **必需** | LIO 输出的高频 odom（≥100Hz） |
| `/goal` | `geometry_msgs/PoseStamped` | 可选（click 模式） | RViz 2D Nav Goal，位姿决定 yaw |
| `/start_trigger` | `std_msgs/Empty` 或自定义 | 可选 | 实机起飞触发（建议用） |

> 这些话题名都在 YAML 配置文件（`click_smooth_ros1.yaml`）里改，不需要 launch remap。

### 3.2 SUPER 节点输出

| 话题 | 类型 | 频率 | 用途 |
|---|---|---|---|
| `/planning/pos_cmd` | `mars_quadrotor_msgs/PositionCommand` | 100Hz | **核心输出**：含 P/V/A/J/yaw |
| `/planning_cmd/poly_traj` | `mars_quadrotor_msgs/PolynomialTrajectory` | replan 时 | 给 MPC 控制器吃整段多项式 |
| `/rog_map/occupied_inflate` | `sensor_msgs/PointCloud2` | 0.5-2Hz | 可视化 |
| `/super/exp_traj_viz` 等 | `visualization_msgs/Marker` | replan 时 | 11 种调试可视化 |

### 3.3 与 EGO 的关键差异

| 项目 | EGO | SUPER |
|---|---|---|
| 主输出 | `quadrotor_msgs::PositionCommand`（PVA + yaw） | `mars_quadrotor_msgs::PositionCommand`（**PVAJ** + yaw） |
| 是否含 jerk | **否** | **是**（SUPER 用 7 阶 MINCO，可稳定取 jerk） |
| 是否需要 trajectory_server | 是（B-spline → PVA） | 否（直接发 PVAJ） |
| MPC 集成 | 一般直接送 PX4 | 推荐先过一层 NMPC，再转 PX4（论文用 PnPMPC） |

### 3.4 坐标系约定

- ROG-Map / SUPER 全程 ENU（X 东 / Y 北 / Z 上）
- LIO 输出 ENU
- MAVROS 内部已做 ENU↔NED 转换，给 setpoint 用 ENU
- **`fix_map_origin` 是 ROG-Map 在不滑动模式下的固定原点**——开启滑动时（默认 true）该字段被忽略

---

## 4. 参数配置

### 4.1 基于 `click_smooth_ros1.yaml` 改实机版

复制一份：
```bash
cd ~/super_ws/src/compare/SUPER/super_planner/config
cp click_smooth_ros1.yaml realworld.yaml
```

主要修改如下（保留与原文件相同的层级，只改实机相关项）：

```yaml
fsm:
  click_goal_en: true
  click_goal_topic: "/goal"
  click_height: 1.2          # 实机起飞高度建议 1.0-1.5m
  click_yaw_en: true
  replan_rate: 10.0          # 实机 10Hz 即可（仿真可 15Hz）
  cmd_topic: "/planning/pos_cmd"
  mpc_cmd_topic: "/planning_cmd/poly_traj"
  timer_en: true

super_planner:
  backup_traj_en: true       # 实机务必开
  detailed_log_en: false
  visualization_en: true
  use_fov_cut: false         # Mid-360 是全向 LiDAR，FOV cut 关掉
  print_log: false
  visual_process: false
  frontend_in_known_free: true   # !!! 实机务必 true：前端只走已知自由空间 !!!
  goal_yaw_en: true
  goal_vel_en: false
  corridor_bound_dis: 1.0
  corridor_line_max_length: 1.2
  safe_corridor_line_max_length: 5.0
  iris_iter_num: 2
  obs_skip_num: 2
  replan_forward_dt: 0.15    # 实机略加宽（仿真 0.1）
  planning_horizon: 6.0       # 实机首飞 5-6m
  sensing_horizon: -1         # -1 = 不限，由 corridor_line_max_length 限制
  receding_dis: 4.0
  robot_r: 0.25               # !!! 一定要 ≥ 实际机体半径 + 0.05 安全余量 !!!
  yaw_dot_max: 2.5
  yaw_mode: 1                 # 1: 朝速度方向（推荐）；2: 朝目标
  mpc_horizon: 15

traj_opt:
  switch:
    save_log_en: false
    print_optimizer_log: false

  boundary:
    max_vel: 3.0              # 实机首飞 2.0-3.0
    max_acc: 4.0
    max_jerk: 100.0
    max_omg: 2.5
    max_acc_thr: 18.0         # 标准 250-280g 机架够用
    min_acc_thr: 4.0
    penna_margin: 0.05

  # exp_traj 和 backup_traj 的 penna_* 一般保持默认即可，
  # 只在出现"轨迹超出走廊""速度超限"时才微调对应权重 ×1.5

  flatness:
    mass: 1.6                  # !!! 实测整机重量 (kg)，含电池 !!!
    dh: 0.30                   # 水平阻尼系数（与机型相关，可暂时保留默认）
    dv: 0.30                   # 垂直阻尼系数
    cp: 0.001
    v_eps: 0.0001
    grav: 9.81

astar:
  map_voxel_num: [ 500, 500, 100 ]
  visual_process: false
  allow_diag: true
  heu_type: 2     # EUCLIDEAN
  debug_visualization_en: false

rog_map:
  resolution: 0.15            # 实机略放粗（仿真 0.1）
  inflation_resolution: 0.15
  inflation_step: 2
  unk_inflation_en: false
  unk_inflation_step: 1
  map_size: [ 60, 60, 5 ]      # 飞行场地的 1-1.5 倍
  fix_map_origin: [ 0, 0, 1.0 ]
  frontier_extraction_en: false
  virtual_ceil_height: 3.0    # 室内的天花板
  virtual_ground_height: -0.1

  load_pcd_en: false           # 实机不预加载点云

  map_sliding:
    enable: true               # !!! 实机务必 true，否则飞机出地图就丢！
    threshold: 1.0

  esdf:
    enable: false              # 实机首飞先关，省算力

  ros_callback:
    enable: true
    cloud_topic: "/cloud_registered"     # 改成你 LIO 实际发的 topic
    odom_topic: "/Odometry"               # FAST-LIO2 默认 /Odometry
    odom_timeout: 2.0

  visualization:
    enable: true
    range: [ 12, 12, 3 ]
    frame_id: "world"
    pub_unknown_map_en: false

  intensity_thresh: -10
  point_filt_num: 1

  raycasting:
    enable: true               # !!! 实机务必开 raycasting，让 miss 也累积 !!!
    batch_update_size: 1
    local_update_box: [ 50, 50, 4 ]
    ray_range: [0.5, 30]
    p_min: 0.12
    p_miss: 0.49
    p_free: 0.499
    p_occ: 0.85
    p_hit: 0.9
    p_max: 0.98
    unk_thresh: 1.0
```

### 4.2 实机最重要的 12 个参数

| 参数 | 路径 | 仿真 | 实机建议 | 调参意图 |
|---|---|---|---|---|
| `robot_r` | super_planner | 0.2 | **0.25-0.35** | = 机体外接圆半径 + 0.05 |
| `frontend_in_known_free` | super_planner | false | **true** | 前端不走 unknown，安全 |
| `replan_forward_dt` | super_planner | 0.1 | **0.15-0.2** | 给优化器留时间 |
| `planning_horizon` | super_planner | 7 | **5-6** | 首飞保守 |
| `backup_traj_en` | super_planner | true | **true** | 实机务必开 |
| `max_vel / max_acc` | traj_opt.boundary | 5/5 | **2-3 / 3-4** | 首飞保守 |
| `mass` | traj_opt.flatness | 1.64 | **实测值** | flatness 公式直接用 |
| `resolution` | rog_map | 0.1 | **0.15** | 实机算力放粗 |
| `map_size` | rog_map | 50 | **60** | 至少 = 飞行场地 ×1.5 |
| `map_sliding.enable` | rog_map | true | **true** | 实机务必开 |
| `raycasting.enable` | rog_map | false | **true** | 否则只标 occupied，不会清 free |
| `esdf.enable` | rog_map | false | **false** | 当前主流程不查 ESDF，关掉省算力 |

### 4.3 启动 launch

新建 `~/super_ws/src/compare/SUPER/super_planner/launch/realworld.launch`：

```xml
<launch>
  <node pkg="super_planner" name="fsm_node" type="fsm_node" output="screen">
    <param name="config_name" value="realworld.yaml" type="string"/>
  </node>

  <!-- 可选 RViz -->
  <node pkg="rviz" type="rviz" name="super_rviz"
        args="-d $(find super_planner)/rviz/realworld.rviz" output="log"/>
</launch>
```

---

## 5. PX4 桥接：双轨迹场景下的安全 setpoint

SUPER 输出 `mars_quadrotor_msgs/PositionCommand`（含 P/V/A/J/yaw/yaw_dot）。
PX4 通过 MAVROS 接收 `mavros_msgs/PositionTarget`（**没有 jerk 字段**）。

桥接策略有两种：

### 方案 A（简单）：直接送 PVA + yaw，丢掉 jerk

适合首飞、调参阶段。控制律退化成 EGO 同款，损失一些前馈精度。

### 方案 B（推荐）：经过 NMPC 中间层

SUPER 论文配套用的是自研 PnPMPC，不开源。开源替代品：
- [`mavros_controllers`](https://github.com/Jaeyoung-Lim/mavros_controllers) 的 `geometric_controller`：吃 PVA + yaw
- `gestelt`、`rotors_simulator/lee_position_controller`：吃 PVAJ + yaw

下面给方案 A 的完整桥接代码。

### 5.1 创建桥接包

```bash
cd ~/super_ws/src
catkin_create_pkg super_px4_bridge roscpp mavros mavros_msgs \
                  mars_quadrotor_msgs nav_msgs geometry_msgs std_msgs
```

### 5.2 桥接节点完整代码

`super_ws/src/super_px4_bridge/src/super_px4_bridge_node.cpp`：

```cpp
#include <ros/ros.h>
#include <mars_quadrotor_msgs/PositionCommand.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>

class SuperPx4Bridge {
public:
    SuperPx4Bridge(ros::NodeHandle& nh) : nh_(nh) {
        nh_.param<double>("takeoff_height", takeoff_h_, 1.2);
        nh_.param<double>("cmd_timeout", cmd_timeout_, 0.2);
        nh_.param<bool>("auto_arm", auto_arm_, false);

        cmd_sub_ = nh_.subscribe("/planning/pos_cmd", 10,
                                  &SuperPx4Bridge::cmdCallback, this);
        state_sub_ = nh_.subscribe("/mavros/state", 10,
                                    &SuperPx4Bridge::stateCallback, this);
        odom_sub_ = nh_.subscribe("/Odometry", 10,
                                   &SuperPx4Bridge::odomCallback, this);
        trigger_sub_ = nh_.subscribe("/start_trigger", 1,
                                      &SuperPx4Bridge::triggerCallback, this);

        setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(
                "/mavros/setpoint_raw/local", 10);

        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(
                "/mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(
                "/mavros/set_mode");

        timer_ = nh_.createTimer(ros::Duration(0.02),     // 50Hz
                                  &SuperPx4Bridge::timerCallback, this);

        ROS_INFO("[super_px4_bridge] Up. takeoff=%.2f, auto_arm=%d",
                 takeoff_h_, auto_arm_);
    }

private:
    void cmdCallback(const mars_quadrotor_msgs::PositionCommand::ConstPtr& msg) {
        last_cmd_ = *msg;
        last_cmd_t_ = ros::Time::now();
        have_cmd_ = true;
    }

    void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
        cur_state_ = *msg;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        odom_ = *msg;
        have_odom_ = true;
    }

    void triggerCallback(const std_msgs::Empty::ConstPtr&) {
        triggered_ = true;
        ROS_INFO("[super_px4_bridge] Triggered");
    }

    void timerCallback(const ros::TimerEvent&) {
        if (!cur_state_.connected) return;

        const ros::Time now = ros::Time::now();

        // ===== 自动 OFFBOARD + ARM（仅当 auto_arm=true）=====
        if (auto_arm_ && triggered_) {
            if (cur_state_.mode != "OFFBOARD" &&
                (now - last_req_t_).toSec() > 5.0) {
                mavros_msgs::SetMode cmd; cmd.request.custom_mode = "OFFBOARD";
                if (set_mode_client_.call(cmd) && cmd.response.mode_sent) {
                    ROS_INFO("OFFBOARD enabled");
                }
                last_req_t_ = now;
            } else if (!cur_state_.armed && (now - last_req_t_).toSec() > 5.0) {
                mavros_msgs::CommandBool cmd; cmd.request.value = true;
                if (arming_client_.call(cmd) && cmd.response.success) {
                    ROS_INFO("Armed");
                }
                last_req_t_ = now;
            }
        }

        // ===== 构造 setpoint =====
        mavros_msgs::PositionTarget sp;
        sp.header.stamp = now;
        sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
                                // MAVROS 已做 ENU→NED 内部转换，直接喂 ENU 数据

        const bool cmd_fresh = have_cmd_ &&
                                 (now - last_cmd_t_).toSec() < cmd_timeout_;

        if (cmd_fresh) {
            // 用 SUPER 的 PVA + yaw + yaw_rate（丢掉 jerk）
            sp.type_mask = 0;       // 全启用
            sp.position.x = last_cmd_.position.x;
            sp.position.y = last_cmd_.position.y;
            sp.position.z = last_cmd_.position.z;
            sp.velocity.x = last_cmd_.velocity.x;
            sp.velocity.y = last_cmd_.velocity.y;
            sp.velocity.z = last_cmd_.velocity.z;
            sp.acceleration_or_force.x = last_cmd_.acceleration.x;
            sp.acceleration_or_force.y = last_cmd_.acceleration.y;
            sp.acceleration_or_force.z = last_cmd_.acceleration.z;
            sp.yaw      = last_cmd_.yaw;
            sp.yaw_rate = last_cmd_.yaw_dot;
        } else if (have_odom_) {
            // 没收到命令 → 悬停（关键：保护飞机不掉下来）
            sp.type_mask =
                mavros_msgs::PositionTarget::IGNORE_VX |
                mavros_msgs::PositionTarget::IGNORE_VY |
                mavros_msgs::PositionTarget::IGNORE_VZ |
                mavros_msgs::PositionTarget::IGNORE_AFX |
                mavros_msgs::PositionTarget::IGNORE_AFY |
                mavros_msgs::PositionTarget::IGNORE_AFZ |
                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
            sp.position.x = odom_.pose.pose.position.x;
            sp.position.y = odom_.pose.pose.position.y;
            sp.position.z = std::max(odom_.pose.pose.position.z, takeoff_h_);
            sp.yaw = 0.0;
        } else {
            return;
        }

        setpoint_pub_.publish(sp);
    }

    ros::NodeHandle nh_;
    ros::Subscriber cmd_sub_, state_sub_, odom_sub_, trigger_sub_;
    ros::Publisher setpoint_pub_;
    ros::ServiceClient arming_client_, set_mode_client_;
    ros::Timer timer_;

    mars_quadrotor_msgs::PositionCommand last_cmd_;
    nav_msgs::Odometry odom_;
    mavros_msgs::State cur_state_;
    bool have_cmd_{false}, have_odom_{false}, triggered_{false}, auto_arm_{false};
    ros::Time last_cmd_t_, last_req_t_;
    double takeoff_h_, cmd_timeout_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "super_px4_bridge");
    ros::NodeHandle nh("~");
    SuperPx4Bridge bridge(nh);
    ros::spin();
    return 0;
}
```

### 5.3 实机时序细节（与 EGO 桥接的差异）

| 项目 | EGO 桥接 | SUPER 桥接 |
|---|---|---|
| 命令消息 | `quadrotor_msgs::PositionCommand` | `mars_quadrotor_msgs::PositionCommand` |
| 是否含 jerk | 否 | 是（这里我们丢掉，方案 A） |
| 发送频率建议 | 50Hz | 50Hz |
| 命令超时 cmd_timeout | 0.2s | 0.2s（同） |
| 命令中断时行为 | 悬停 | 悬停（依赖 SUPER 自身的 backup_traj 已经在飞向悬停位） |
| 是否需要主动急停 | 是（EGO 急停轨迹是全零） | **否**（SUPER 的 backup_traj 已经是减速段） |

> **SUPER 实机最大的安全优势**：当 ReplanOnce 失败、桥接超时时，飞机正在执行的 cmd_traj 末尾自动是 backup_traj，即使后续命令彻底中断 200ms 以上，飞机也会按已有多项式自然减速悬停在已知自由空间。EGO 没有这个机制——命令中断时桥接节点必须主动悬停。

### 5.4 PolynomialTrajectory 直发模式（方案 B 雏形）

如果你用的下游控制器（比如 `mavros_controllers/geometric_controller`）能直接吃多项式轨迹，可以监听 `/planning_cmd/poly_traj` 而不是 PVA 命令。SUPER 在 `cmd_traj_info_` 内部已经把 exp + backup 拼好了，整段多项式比离散 PVA 命令鲁棒性更高。

```cpp
// 简化版示意
void polyCallback(const mars_quadrotor_msgs::PolynomialTrajectory::ConstPtr& msg) {
    // 1) 解析多项式系数（每段 8×3 + 段时长）
    // 2) 缓存到下游 NMPC，作为参考轨迹
    // 3) NMPC 在每个采样点根据当前 odom 计算最优控制 → 转 PVA → 发 MAVROS
}
```

具体实现见 `super_planner/include/data_structure/cmd_traj.h` 中 `getState(t)` 的解析。

---

## 6. 实机测试流程

### 6.1 静态测试（不解锁）
1. 启动 `mavros`，确认 `/mavros/state` 中 `connected: true`
2. 启动 LIO（FAST-LIO2）：
   ```bash
   roslaunch fast_lio mapping_mid360.launch
   ```
   `rostopic hz /Odometry` 应 ≥ 100Hz；`rostopic hz /cloud_registered` 应 = 10Hz
3. 启动 SUPER：
   ```bash
   roslaunch super_planner realworld.launch
   ```
   确认终端输出 `[Fsm] Begin.` + `[Fsm] Wait for goal.`
4. 启动桥接：
   ```bash
   rosrun super_px4_bridge super_px4_bridge_node _auto_arm:=false _takeoff_height:=1.2
   ```
   `rostopic echo /mavros/setpoint_raw/local` 应有 50Hz 输出
5. 在 RViz 中应看到 `/rog_map/occupied_inflate` 占用栅格点云、机体周围有 SFC 多面体

### 6.2 第一次起飞（推荐流程）
1. 空旷场地架机，桨叶通电前
2. 通电飞控（USB），通电桨叶（电池）
3. 遥控器解锁 + 切 OFFBOARD
4. 桥接节点会用悬停模式爬升到 `takeoff_height`
5. 通过 RViz 点击目标点（2D Nav Goal）
6. SUPER 立即生成 exp + backup 轨迹 → 飞机开始飞向目标
7. 终端可见 `[Fsm] Replan succeed` 周期性出现

### 6.3 异常处理

| 现象 | 排查 |
|---|---|
| FSM 一直停在 INIT | LIO odom 没出 → `rostopic hz /Odometry` |
| FSM 一直停在 WAIT_GOAL | 没点击目标，或目标点击在 SFC 外 |
| 报 "Local start point is deeply occupied" | 当前位置被 ROG-Map 标占据 → 检查 `robot_r` 太大或 odom 漂移 |
| 报 "Replan over time" | 单帧规划超过 0.9 × replan_forward_dt → 调小 planning_horizon 或加大 replan_forward_dt |
| 飞机一直在悬停（不走） | 桥接没收到 cmd → `rostopic hz /planning/pos_cmd` |
| 频繁触发 backup（飞行很慢） | ROG-Map 大量未知 → `frontend_in_known_free: false` 实测会激进，但前提是 backup 充足 |
| 撞墙 | `robot_r` 不够 / `inflation_resolution` 太粗 |
| OFFBOARD 自动退出 | setpoint 中断 → 桥接节点 CPU 是否过载 |

---

## 7. 输入输出 / PX4 接口总览图

```
LIO（FAST-LIO2 / Point-LIO）
    │
    ├─ /Odometry (≥100Hz, ENU) ──────────────────┐
    └─ /cloud_registered (10Hz, ENU) ────────────┤
                                                  │
                                                  ▼
                                  ┌─────────────────────────────┐
                                  │     ROGMap (内置在 SUPER)    │
                                  │   prob_map + inf_map         │
                                  └──────────────┬──────────────┘
                                                  │
                                                  ▼
                                       ┌──────────────────┐
                                       │   SuperPlanner    │
                                       │  PlanFromRest /    │
                                       │  ReplanOnce        │
                                       └────────┬───────────┘
                                                │
                              ┌─────────────────┴─────────────────┐
                              ▼                                    ▼
                  /planning/pos_cmd (100Hz)        /planning_cmd/poly_traj (≈10Hz)
                  PositionCommand (P/V/A/J/yaw)    PolynomialTrajectory (整段)
                              │                                    │
                              ▼                                    ▼
                   ┌──────────────────────┐           ┌──────────────────────┐
                   │  super_px4_bridge   │           │ NMPC (mavros_         │
                   │  (方案 A，本文档)    │ 或者     │  controllers etc.)   │
                   └──────────┬───────────┘           └──────────┬───────────┘
                              │                                  │
                              └────────┬─────────────────────────┘
                                       ▼
                       /mavros/setpoint_raw/local (50Hz)
                       PositionTarget
                                       │
                                       ▼
                                ┌──────────┐
                                │  MAVROS  │
                                └────┬─────┘
                                     │ MAVLink (USB / TELEM2)
                                     ▼
                                ┌──────────┐
                                │   PX4    │
                                └────┬─────┘
                                     ▼
                                 电机/PWM
```

---

## 8. 推荐实测参数集（论文复现近似配置）

| 项目 | 值 |
|---|---|
| 机架轴距 | 280-350mm |
| 机体半径（含螺旋桨） | 0.20-0.25m |
| robot_r | 0.25-0.30 |
| max_vel / max_acc / max_jerk | 5.0 / 5.0 / 100（已做 backup 兜底，可放心拉高） |
| LiDAR | Livox Mid-360 / Avia 10Hz |
| LIO | FAST-LIO2 200Hz odom |
| 飞控 | Pixhawk 6X，PX4 v1.14 |
| 机载机 | Jetson Orin NX 16GB |
| 单帧 SUPER 总耗时 | 30-80 ms |
| 实机最高速度 | 5-8 m/s（论文实测可达 18 m/s） |

---

## 9. EGO 与 SUPER 部署的核心差异速查

| 维度 | EGO | SUPER |
|---|---|---|
| 推荐传感器 | 深度相机（D435 / D455） | 激光雷达（Mid-360 等） |
| SLAM | VINS-Fusion / Mono | FAST-LIO2 / Point-LIO |
| ROS 包入口 | `ego_planner_node` + `traj_server` | `fsm_node`（一个就够） |
| 配置 | launch + 参数服务器 | YAML 文件 |
| 控制频率 | 100Hz cmd | 100Hz cmd（实测同样） |
| jerk 输出 | ❌ | ✅（桥接方案 A 丢弃，方案 B 保留） |
| 命令中断响应 | 必须主动悬停 | 自动 backup_traj 减速 |
| 多机 | 支持 | 不支持 |
| 机载机门槛 | Jetson Nano / NX | **Orin NX 起步** |
| 配套 MPC | 不需要（直发 PX4） | 强烈建议 NMPC 中间层 |
| 实机首飞难度 | 中（VIO + 调参） | 中高（LIO + flatness 调参 + backup 调参） |
| 安全裕度 | 中（依赖急停） | 高（依赖 backup） |
