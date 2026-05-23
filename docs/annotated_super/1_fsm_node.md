# fsm_node_ros1.cpp —— SUPER 程序入口

## 文件路径
`super_planner/Apps/fsm_node_ros1.cpp`

## 核心理解

跟 EGO `ego_planner_node.cpp` 极简风格一脉相承——主程序什么都不做，只把控制权交给 `FsmRos1`，然后 `AsyncSpinner` 起飞。所有规划逻辑都封装在 `fsm::Fsm`（基类）和 `FsmRos1`（ROS1 派生类）里。

```cpp
// L46-72：main 函数
int main(int argc, char **argv) {
    ros::init(argc, argv, "fsm_node");
    ros::NodeHandle nh("~");

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);  // 关掉 PCL 噪音
    cout << GREEN << " -- [Fsm-Test] Begin." << RESET << endl;

    // ============ 配置文件解析 ============
    // 默认值：<package_root>/config/click.yaml
#define CONFIG_FILE_DIR(name) (string(string(ROOT_DIR) + "config/"+name))
    std::string dft_cfg_path = CONFIG_FILE_DIR("click.yaml");
    std::string cfg_path, cfg_name;
    // 优先级：config_path > config_name > 默认 click.yaml
    if (nh.param("config_path", cfg_path, dft_cfg_path)) {
        cout << " -- [Fsm-Test] Load config from: " << cfg_path << endl;
    } else if(nh.param("config_name", cfg_name, dft_cfg_path)){
        cfg_path = CONFIG_FILE_DIR(cfg_name);
        cout << " -- [Fsm-Test] Load config by file name: " << cfg_name << endl;
    }

    // ============ 创建并初始化 FSM ============
    fsm_ptr = make_shared<FsmRos1>();
    fsm_ptr->init(nh, cfg_path);

    // ============ 异步 spinner ============
    // AsyncSpinner(0)：自动选择线程数（默认 = CPU 核数）
    // 这一选择对 SUPER 至关重要：
    //   - SUPER 的 callMainFsmOnce / callReplanOnce 会持有 replan_lock_ 互斥锁
    //   - ROG-Map 回调（点云 + odom）在另一线程更新地图
    //   - traj_server / yaw_server 回调在第三线程拷贝轨迹
    // 单线程 ros::spin() 无法满足实时性
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::Duration(1.0).sleep();           // 等待其他节点起来
    ros::waitForShutdown();
    return 0;
}
```

## 与 EGO 的相同点

| 项目 | EGO | SUPER |
|---|---|---|
| 节点风格 | 单一入口 | 单一入口 |
| 命令行 | `ros::init` | `ros::init` |
| 把所有逻辑交给一个类 | `EGOReplanFSM` | `FsmRos1` |

## 与 EGO 的不同点

| 项目 | EGO | SUPER |
|---|---|---|
| 参数加载 | `nh.param("fsm/...")`（参数服务器） | YAML 文件（cereal-yaml-loader） |
| 事件循环 | `ros::spin()`（单线程） | `ros::AsyncSpinner(0)`（多线程） |
| 信号回溯 | 无 | `BACKWARD_HAS_DW`（链 backward-cpp，崩溃时打印 backtrace） |
| ROS 兼容性 | 仅 ROS1 | 同时支持 ROS1（本文件）和 ROS2（`fsm_node_ros2.cpp`）—— 通过 `ros_interface/` 抽象层做适配 |

## ROOT_DIR 宏

`ROOT_DIR` 由 `super_planner/CMakeLists.txt` 定义：

```cmake
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
```

它是 `super_planner` 包的源代码根路径——这样配置文件就可以放在 `super_planner/config/` 里，不依赖 ROS 包查找机制。

## SUPER 配置文件结构（`config/click.yaml`）

YAML 文件被三个部分共享：

```yaml
super_planner:
  # 顶层规划参数（planning_horizon, robot_r, replan_forward_dt, ...）

exp_traj:
  # 探索轨迹优化器参数
  max_vel: 8.0
  max_acc: 6.0
  max_jerk: 30.0
  ...

backup_traj:
  # 备份轨迹优化器参数

rog_map:
  # 地图参数（resolution, half_map_size, batch_update_size, ...）

fsm:
  # FSM 参数（replan_rate, click_height, click_yaw_en, ...）
```

`super_planner::Config` / `traj_opt::Config` / `rog_map::ROGMapConfig` / `fsm::Config` 各自从 YAML 不同顶层 key 读取自己的字段。

## 与后续模块的关系

```
fsm_node_ros1.cpp
    ↓ 创建并初始化
FsmRos1::init  (include/ros_interface/ros1/fsm_ros1.hpp)
    ├── 加载 YAML 配置 → fsm::Config + super_planner::Config + rog_map::Config
    ├── 创建 ROGMapROS1（订阅点云 + odom，启动地图更新定时器）
    ├── 创建 SuperPlanner（构造 ExpTrajOpt + BackupTrajOpt + Astar + CorridorGenerator + CIRI）
    ├── 注册 RViz 点击目标订阅 → setGoalPosiAndYaw
    ├── 创建定时器 callMainFsmOnce（驱动状态机）
    └── 创建定时器 callReplanOnce（FOLLOW_TRAJ 状态下重规划）
```

**一句话总结**：`fsm_node_ros1.cpp` 是 SUPER 的"启动开关"，所有智能都在 `Fsm` + `SuperPlanner` 里。
