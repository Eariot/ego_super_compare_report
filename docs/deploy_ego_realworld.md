# EGO-Planner 实机部署测试说明

> 适用版本：`compare/ego-planner/`（同 ZJU-FAST-Lab/ego-planner master）。
> 目标平台：搭载 PX4 飞控（Pixhawk / CUAV / Holybro 等）+ NVIDIA Jetson / Intel NUC 机载计算机的四旋翼。
> 运行栈：Ubuntu 20.04 + ROS Noetic + MAVROS 1.x + PX4 v1.13/1.14。

本文档分四部分：
1. 硬件与软件准备
2. 编译与启动
3. 输入 / 输出话题与坐标系
4. 参数配置（按机型差异调）
5. PX4 桥接（核心：把 EGO 的 `PositionCommand` 转成 MAVROS 能消费的 `setpoint_raw/local`）

---

## 1. 硬件与软件准备

### 1.1 推荐硬件清单

| 模块 | 型号示例 | 必要性 | 备注 |
|---|---|---|---|
| 飞控 | Pixhawk 6X / CUAV V5+ / Holybro Pix32 v6 | 必需 | 固件 PX4 v1.13.x 起，能跑 OFFBOARD |
| 机载计算机 | Jetson Orin Nano / NX / NUC i5 | 必需 | EGO 单帧规划 5-15 ms，Orin Nano 已绰绰有余 |
| 深度相机 | Intel RealSense D435i / D455 | 二选一 | 推荐方案，对 EGO 适配最好 |
| 激光雷达 | Livox Mid-360 / Avia | 二选一 | 也可，但需要 `pcl_render_node` 把点云转成深度 |
| 视觉惯导 | VINS-Fusion / FAST-LIO 输出 odom | 必需 | EGO 需要 200Hz+ 高频 odom 才能稳定 |
| 串口 | Telem2 (`/dev/ttyTHS0` 或 `/dev/ttyUSB0`) | 必需 | 飞控 ↔ 机载计算机 MAVLink |

### 1.2 软件依赖

```bash
# Ubuntu 20.04 + ROS Noetic
sudo apt install ros-noetic-desktop-full \
                 ros-noetic-mavros ros-noetic-mavros-extras \
                 ros-noetic-realsense2-camera ros-noetic-realsense2-description \
                 ros-noetic-tf2-eigen ros-noetic-eigen-conversions \
                 libarmadillo-dev libnlopt-dev libnlopt-cxx-dev

# MAVROS 的 GeographicLib datasets（必须，否则 MAVROS 启动报错）
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

### 1.3 关键消息包（EGO 自带）
- `quadrotor_msgs`（在 `ego-planner/src/uav_simulator/Utils/quadrotor_msgs/`）：定义 `PositionCommand`（EGO 的核心输出）
- `traj_utils`（在 `ego-planner/src/planner/traj_utils/`）：定义 `Bspline` / `MultiBsplines`（FSM 内部传给 traj_server）

---

## 2. 编译与启动

### 2.1 拉源码
```bash
mkdir -p ~/ego_ws/src
cd ~/ego_ws/src
git clone https://github.com/Eariot/ego_super_compare_report.git compare
ln -s compare/ego-planner/src/* ./   # 把 ego-planner 的子包符号链接到 src
```

### 2.2 编译
```bash
cd ~/ego_ws
catkin_make -DCMAKE_BUILD_TYPE=Release -j4
source devel/setup.bash
echo "source ~/ego_ws/devel/setup.bash" >> ~/.bashrc
```

> 提示：Jetson 上 `-j4` 是为了避免 OOM。Orin 系列可加到 `-j6`。

### 2.3 启动顺序

实机启动 6 个节点，建议分 5 个 terminal（或一个总 launch）：

```bash
# Terminal 1: MAVROS（飞控 ↔ ROS 桥）
roslaunch mavros px4.launch fcu_url:=/dev/ttyTHS0:921600 gcs_url:=

# Terminal 2: 深度相机（RealSense 示例）
roslaunch realsense2_camera rs_camera.launch \
        align_depth:=true enable_pointcloud:=false \
        depth_width:=640 depth_height:=480 depth_fps:=30 \
        color_width:=640 color_height:=480 color_fps:=30

# Terminal 3: 状态估计（VINS-Fusion / FAST-LIO，二选一）
# 例如 VINS-Fusion：
roslaunch vins_fusion fast_drone_250.launch

# Terminal 4: EGO-Planner 本体
roslaunch ego_planner run_in_realworld.launch    # 见 §4

# Terminal 5: PX4 桥接（自己写的小节点，见 §5）
rosrun ego_px4_bridge ego_px4_bridge_node

# 可选 Terminal 6: RViz 监控
rosrun rviz rviz -d $(rospack find ego_planner)/launch/default.rviz
```

---

## 3. 输入 / 输出话题与坐标系

### 3.1 EGO 节点输入（订阅）

| 话题（默认名） | 类型 | 必要性 | 说明 |
|---|---|---|---|
| `/odom_world` | `nav_msgs/Odometry` | **必需** | VIO/LIO 输出，频率 ≥100Hz；`world` 系下机体位姿 |
| `/grid_map/depth` | `sensor_msgs/Image` | 与 cloud 二选一 | 深度图，16-bit Mono 或 32-bit Float（米） |
| `/grid_map/pose` | `geometry_msgs/PoseStamped` | 与 odom 二选一（深度图模式下用） | 相机位姿（一般由 VIO 输出） |
| `/grid_map/cloud` | `sensor_msgs/PointCloud2` | 与 depth 二选一 | LiDAR 点云模式 |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | flight_type=1 时 | RViz 2D Nav Goal |
| `/traj_start_trigger` | `geometry_msgs/PoseStamped` | flight_type=2 时（实机推荐） | 起飞触发，常由遥控器开关产生 |

> **flight_type 的选择**：实机务必用 `flight_type=2`（PRESET_TARGET，外部触发）；用 1（RViz 点击）有人工延迟，且 z 写死 1.0m。

### 3.2 EGO 节点输出（发布）

| 话题（默认名） | 类型 | 频率 | 用途 |
|---|---|---|---|
| `planning/bspline` | `traj_utils/Bspline` | replan 时（约 2.5Hz） | 给 traj_server 用 |
| `planning/data_display` | `traj_utils/DataDisp` | 100Hz | 可视化 |
| `/grid_map/occupancy_inflate` | `sensor_msgs/PointCloud2` | 1Hz | RViz 看膨胀地图 |

### 3.3 traj_server 的输入输出

`traj_server` 是 EGO 自带的"轨迹解算器"：

| 输入 | 输出 |
|---|---|
| `planning/bspline`（Bspline 控制点） | `/position_cmd` (`quadrotor_msgs/PositionCommand`) |
| `/odom_world` | 解算频率 100Hz |

`PositionCommand` 字段（关键）：
```
position    : Vector3 (x, y, z)
velocity    : Vector3
acceleration: Vector3
yaw         : double
yaw_dot     : double
```

> **核心注意**：`PositionCommand` **不是** PX4 能直接消费的消息。需要桥接（见 §5）。

### 3.4 坐标系

- EGO 内部全程使用 `world` 系（ENU 右手系：X 东 / Y 北 / Z 上）
- VINS-Fusion / FAST-LIO 默认输出 ENU
- PX4 内部 LOCAL_NED 是 NED（X 北 / Y 东 / Z 下），但 MAVROS 已经做了 ENU↔NED 转换，所以你给 MAVROS 的所有 setpoint 用 **ENU** 即可

---

## 4. 参数配置（基于 `simple_run.launch` + `advanced_param.xml` 改）

新建 `~/ego_ws/src/compare/ego-planner/src/planner/plan_manage/launch/run_in_realworld.launch`：

```xml
<launch>
  <!-- ============ 基础参数 ============ -->
  <arg name="map_size_x" value="40.0"/>
  <arg name="map_size_y" value="40.0"/>
  <arg name="map_size_z" value="3.0"/>

  <!-- !!! 这一项必须改成你 VIO/LIO 实际发布的 odom topic !!! -->
  <arg name="odom_topic" value="/Odom_high_freq"/>

  <include file="$(find ego_planner)/launch/advanced_param.xml">
    <arg name="map_size_x_" value="$(arg map_size_x)"/>
    <arg name="map_size_y_" value="$(arg map_size_y)"/>
    <arg name="map_size_z_" value="$(arg map_size_z)"/>
    <arg name="odometry_topic" value="$(arg odom_topic)"/>

    <!-- ============ RealSense D435i 实测内参（请用 rs-enumerate-devices -c 校核）============ -->
    <arg name="camera_pose_topic" value="/vins_fusion/camera_pose"/>
    <arg name="depth_topic" value="/camera/aligned_depth_to_color/image_raw"/>
    <arg name="cloud_topic" value="nouse_cloud"/>
    <arg name="cx" value="321.046"/>
    <arg name="cy" value="243.450"/>
    <arg name="fx" value="387.229"/>
    <arg name="fy" value="387.229"/>

    <!-- ============ 速度 / 加速度 / 前视距离（按机架尺寸调）============ -->
    <arg name="max_vel" value="1.5"/>      <!-- 实机首飞建议 1.0-1.5 -->
    <arg name="max_acc" value="2.0"/>      <!-- 1.5 倍 max_vel 比较稳 -->
    <arg name="planning_horizon" value="5.0"/>
                                            <!-- 务必 ≤ depth_filter_maxdist × 1.5 -->

    <!-- ============ 实机用 PRESET_TARGET 模式 ============ -->
    <arg name="flight_type" value="2"/>
    <arg name="point_num" value="2"/>
    <arg name="point0_x" value="3.0"/> <arg name="point0_y" value="0.0"/> <arg name="point0_z" value="1.2"/>
    <arg name="point1_x" value="0.0"/> <arg name="point1_y" value="0.0"/> <arg name="point1_z" value="1.2"/>
    <arg name="point2_x" value="0.0"/> <arg name="point2_y" value="0.0"/> <arg name="point2_z" value="1.2"/>
    <arg name="point3_x" value="0.0"/> <arg name="point3_y" value="0.0"/> <arg name="point3_z" value="1.2"/>
    <arg name="point4_x" value="0.0"/> <arg name="point4_y" value="0.0"/> <arg name="point4_z" value="1.2"/>
  </include>

  <!-- traj_server -->
  <node pkg="ego_planner" name="traj_server" type="traj_server" output="screen">
    <remap from="/odom_world" to="$(arg odom_topic)"/>
    <param name="traj_server/time_forward" value="1.0" type="double"/>
  </node>
</launch>
```

### 4.1 参数调整速查表（实机最重要的 10 个）

| 参数 | 含义 | 仿真值 | 实机建议 | 备注 |
|---|---|---|---|---|
| `manager/max_vel` | 最大速度 (m/s) | 2-3 | **1.0~1.5** 起步 | 跟 `optimization/max_vel`、`bspline/limit_vel` 三处保持一致 |
| `manager/max_acc` | 最大加速度 (m/s²) | 3 | **1.5×max_vel** | 太大易过冲 |
| `manager/max_jerk` | 最大 jerk | 4 | 4-8 | 一般不用动 |
| `fsm/planning_horizon` | 局部前视 (m) | 7.5 | **5.0** | 必须 ≤ 1.5×（深度图最大可视距离） |
| `fsm/emergency_time_` | 急停触发阈值 (s) | 1.0 | **0.8~1.2** | 时间越长越保守 |
| `grid_map/resolution` | 栅格分辨率 (m) | 0.1 | 0.1-0.15 | 实机算力限制下可调到 0.15 |
| `grid_map/obstacles_inflation` | 膨胀半径 (m) | 0.099 | **0.20~0.30** | **实机最关键！** = 机体半径 + 安全余量 |
| `grid_map/max_ray_length` | 最大射线 (m) | 4.5 | 4-5 | 跟 RealSense D435 实际可视距离一致 |
| `grid_map/depth_filter_maxdist` | 深度图最远 (m) | 5 | 4-5 | 同上 |
| `grid_map/p_hit / p_miss` | 占用概率 | 0.65/0.35 | 0.70/0.35 | 实机噪声大可适当提高 p_hit |

### 4.2 EGO 与你 VIO 的"约定俗成"清单

- **odom 频率必须 ≥ 100Hz**：EGO 的 `execFSMCallback` 是 100Hz，`have_odom_` 一掉就触发急停
- **depth + camera_pose 时间戳要对齐**：用 RealSense 的 `align_depth:=true` + VINS 的 `camera_pose` 输出（不是 odom）
- **TF 树必须正确**：`world → body → camera_link`，否则 GridMap 投影错位
- **z 轴向上正向**（ENU），不要用 NED

---

## 5. PX4 桥接：把 PositionCommand 转成 MAVROS setpoint

EGO 输出 `quadrotor_msgs/PositionCommand`（含 P/V/A/yaw/yaw_dot）。
PX4 通过 MAVROS 接收 `mavros_msgs/PositionTarget`（消息号 SET_POSITION_TARGET_LOCAL_NED）。
**两者字段不同，必须写一个桥接节点。**

### 5.1 创建桥接包

```bash
cd ~/ego_ws/src
catkin_create_pkg ego_px4_bridge roscpp mavros mavros_msgs quadrotor_msgs nav_msgs geometry_msgs
```

### 5.2 桥接节点完整代码

`ego_ws/src/ego_px4_bridge/src/ego_px4_bridge_node.cpp`：

```cpp
#include <ros/ros.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>

class EgoPx4Bridge {
public:
    EgoPx4Bridge(ros::NodeHandle& nh) : nh_(nh) {
        nh_.param<double>("takeoff_height", takeoff_height_, 1.2);
        nh_.param<double>("hover_x", hover_x_, 0.0);
        nh_.param<double>("hover_y", hover_y_, 0.0);

        // 输入：EGO 的位置命令
        cmd_sub_ = nh_.subscribe("/position_cmd", 10,
                                  &EgoPx4Bridge::cmdCallback, this);
        // 输入：MAVROS 状态（监控解锁/模式）
        state_sub_ = nh_.subscribe("/mavros/state", 10,
                                    &EgoPx4Bridge::stateCallback, this);
        // 输入：本机位姿（用于在没有 EGO 命令时悬停）
        odom_sub_ = nh_.subscribe("/Odom_high_freq", 10,
                                   &EgoPx4Bridge::odomCallback, this);

        // 输出：PX4 setpoint
        setpoint_pub_ = nh_.advertise<mavros_msgs::PositionTarget>(
                "/mavros/setpoint_raw/local", 10);

        // PX4 服务客户端
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

        // 50Hz 主循环
        timer_ = nh_.createTimer(ros::Duration(0.02),
                                  &EgoPx4Bridge::timerCallback, this);

        last_cmd_time_ = ros::Time(0);
    }

private:
    void cmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg) {
        last_cmd_ = *msg;
        last_cmd_time_ = ros::Time::now();
        have_cmd_ = true;
    }

    void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
        current_state_ = *msg;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        odom_ = *msg;
        have_odom_ = true;
    }

    void timerCallback(const ros::TimerEvent&) {
        // ===== 自动切 OFFBOARD + ARM（首次进入时）=====
        ros::Time now = ros::Time::now();
        if (!current_state_.connected) return;

        if (current_state_.mode != "OFFBOARD" &&
            (now - last_request_time_).toSec() > 5.0) {
            mavros_msgs::SetMode mode_cmd;
            mode_cmd.request.custom_mode = "OFFBOARD";
            if (set_mode_client_.call(mode_cmd) && mode_cmd.response.mode_sent) {
                ROS_INFO("OFFBOARD enabled");
            }
            last_request_time_ = now;
        } else if (!current_state_.armed &&
                    (now - last_request_time_).toSec() > 5.0) {
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = true;
            if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("Vehicle armed");
            }
            last_request_time_ = now;
        }

        // ===== 构造 setpoint =====
        mavros_msgs::PositionTarget sp;
        sp.header.stamp = now;
        sp.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
                                // 注意：MAVROS 已经做 ENU→NED 转换，
                                // 所以这里用 ENU 数据即可（FRAME_LOCAL_NED 是 MAVROS API 命名遗留）

        bool cmd_fresh = have_cmd_ && (now - last_cmd_time_).toSec() < 0.2;

        if (cmd_fresh) {
            // 用 EGO 的全状态命令（位置 + 速度 + 加速度 + yaw）
            sp.type_mask = 0;   // 全部启用
            sp.position.x  = last_cmd_.position.x;
            sp.position.y  = last_cmd_.position.y;
            sp.position.z  = last_cmd_.position.z;
            sp.velocity.x  = last_cmd_.velocity.x;
            sp.velocity.y  = last_cmd_.velocity.y;
            sp.velocity.z  = last_cmd_.velocity.z;
            sp.acceleration_or_force.x = last_cmd_.acceleration.x;
            sp.acceleration_or_force.y = last_cmd_.acceleration.y;
            sp.acceleration_or_force.z = last_cmd_.acceleration.z;
            sp.yaw      = last_cmd_.yaw;
            sp.yaw_rate = last_cmd_.yaw_dot;
        } else if (have_odom_) {
            // 没有命令 → 悬停在当前位置（高度锁定 takeoff_height）
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
            sp.position.z = std::max(odom_.pose.pose.position.z, takeoff_height_);
            sp.yaw = 0.0;
        } else {
            return;     // 啥都没有，先别发
        }

        setpoint_pub_.publish(sp);
    }

    ros::NodeHandle nh_;
    ros::Subscriber cmd_sub_, state_sub_, odom_sub_;
    ros::Publisher setpoint_pub_;
    ros::ServiceClient arming_client_, set_mode_client_;
    ros::Timer timer_;

    quadrotor_msgs::PositionCommand last_cmd_;
    nav_msgs::Odometry odom_;
    mavros_msgs::State current_state_;
    bool have_cmd_{false}, have_odom_{false};
    ros::Time last_cmd_time_, last_request_time_;
    double takeoff_height_, hover_x_, hover_y_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ego_px4_bridge");
    ros::NodeHandle nh("~");
    EgoPx4Bridge bridge(nh);
    ros::spin();
    return 0;
}
```

### 5.3 关键点解释

#### a. type_mask 的取值

`PositionTarget.type_mask` 是位掩码，控制 PX4 用哪些字段。

| 取值 | 启用字段 | 何时用 |
|---|---|---|
| `0` | P + V + A + yaw + yaw_rate（全启用） | EGO 正常工作时 |
| `IGNORE_VX|VY|VZ|AFX|AFY|AFZ|YAW_RATE` | 仅 P + yaw | 悬停 / 起飞 |
| 用 `0b110111000111`（跳过 P，只用 V+A+yaw_rate） | 速度模式 | 高速场景，但 EGO 不推荐 |

**实测结论**：用 `type_mask = 0`（全启用）效果最好——PX4 内部会用 P 做闭环，V/A 做前馈，yaw_rate 控转头平滑度。

#### b. 频率

- EGO 的 `traj_server` 发 `/position_cmd` 是 100Hz；
- 桥接定时器是 50Hz（PX4 OFFBOARD 要求 setpoint > 2Hz，常用 20-100Hz，50Hz 性价比高）；
- 一定不能让 setpoint 中断超过 0.5s，否则 PX4 会自动退出 OFFBOARD。

#### c. 安全保护

- `cmd_fresh` 判断 200ms 内是否有 EGO 命令；超时则切换到悬停模式（不会失控）；
- 桥接节点首次启动时会自动尝试 OFFBOARD + ARM —— 实机首飞建议**注释掉这部分**，用遥控器手动切 OFFBOARD 更安全。

---

## 6. 实机测试流程

### 6.1 静态测试（不解锁）
1. 连接遥控器，确认 GPS / VIO 锁定
2. `roslaunch mavros px4.launch ...` → 看 `/mavros/state` 中 `connected: true`
3. `roslaunch realsense2_camera rs_camera.launch ...` → `rostopic hz /camera/aligned_depth_to_color/image_raw` 应 ≥ 25Hz
4. 启动 VINS → `rostopic hz /Odom_high_freq` 应 ≥ 100Hz
5. 启动 EGO → 在 RViz 中应能看到 `/grid_map/occupancy_inflate` 点云
6. 启动桥接节点 → `rostopic echo /mavros/setpoint_raw/local` 应有持续输出

### 6.2 第一次起飞
1. 在空旷场地架机，桨叶向上但不通电（先用 USB 通电飞控）
2. 起飞前**确认 EGO 已收到 `/Odom_high_freq` 但还没收到 trigger**（FSM 应该停在 `WAIT_TARGET`）
3. 通电桨叶
4. 遥控器切 OFFBOARD + 解锁（或留给桥接节点自动）
5. 桥接节点会用悬停模式让飞机起到 `takeoff_height`
6. 通过 `rostopic pub /traj_start_trigger` 发起飞 trigger，EGO 开始走预设航点

### 6.3 异常处理

| 现象 | 原因 | 排查 |
|---|---|---|
| EGO 一直停在 `WAIT_TARGET` | 没收到 trigger，或 odom 频率不够 | `rostopic hz /Odom_high_freq`，发 trigger |
| 起飞瞬间冲到 max_vel | type_mask 配错 / EGO 输出异常 | 把 max_vel 调到 0.5 重试 |
| 飞机一直撞墙 | obstacles_inflation 太小 | 加到 0.25-0.30 |
| 老是急停 | depth 帧率低 / odom 抖 | 检查 `rostopic hz` |
| OFFBOARD 自动退出 | setpoint 中断 > 0.5s | 桥接节点是否在跑、CPU 是否过载 |

---

## 7. 输入输出 / PX4 接口总览图

```
        ┌─ /Odom_high_freq (100Hz, ENU) ──────┐
VIO ────┤                                     │
        └─ /vins_fusion/camera_pose (30Hz) ───┤
                                              ├─▶ EGO planner
RealSense ─ /camera/aligned_depth_to_color/image_raw (30Hz) ┘
                                              │
                                              ▼
                                    /position_cmd (100Hz)
                                    quadrotor_msgs/PositionCommand
                                              │
                                              ▼
                                  ┌────────────────────────┐
                                  │  ego_px4_bridge_node   │
                                  │  转换 + 安全保护         │
                                  └────────────────────────┘
                                              │
                                              ▼
                          /mavros/setpoint_raw/local (50Hz)
                          mavros_msgs/PositionTarget
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
                                       电机 / PWM
```

---

## 8. 推荐实测参数集（Fast-Drone-250 同款）

| 项目 | 值 |
|---|---|
| 机架轴距 | 250mm |
| 机体半径 (含螺旋桨) | 0.18m |
| obstacles_inflation | 0.20m |
| max_vel / max_acc | 1.5 / 2.5 |
| planning_horizon | 5.0m |
| 深度相机 | RealSense D435i 360p@30 |
| VIO | VINS-Fusion (mono+IMU) 200Hz |
| 飞控 | Pixhawk 4 mini，PX4 v1.13.3 |
| 机载机 | Jetson Xavier NX |
| 单帧 EGO 规划耗时 | 8-12 ms |
| 实测最高速度 | 2.0 m/s（保守）/ 4.0 m/s（激进调参） |
