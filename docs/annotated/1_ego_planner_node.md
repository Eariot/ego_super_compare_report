# ego_planner_node.cpp —— EGO 程序入口
## 文件路径
`official/Fast-Drone-250/src/planner/plan_manage/src/ego_planner_node.cpp`

## 核心理解

EGO 的主程序极其简单，**主程序本身不参与任何规划逻辑**，它只做了两件事：

1. **初始化 EGOReplanFSM** — 把整个规划流程的控制权交给状态机
2. **ros::spin()** — 让 ROS 开始处理回调，从此所有逻辑都由 EGOReplanFSM 的定时器和回调驱动

这种设计叫"状态机驱动"：主程序只负责启动，启动后一切由状态机调度。

```cpp
int main(int argc, char **argv)
{
    // 1. 初始化 ROS 节点，节点名为 "ego_planner_node"
    ros::init(argc, argv, "ego_planner_node");
    ros::NodeHandle nh("~");   // "~" 表示私有命名空间，参数以 "~/" 开头

    // 2. 创建 EGO 有限状态机实例
    EGOReplanFSM rebo_replan;

    // 3. 初始化状态机（读取参数、建立订阅/发布/定时器）
    rebo_replan.init(nh);

    // 4. 进入事件循环——从此所有逻辑都由 EGOReplanFSM 的回调驱动
    //    主程序在这里永远等待，不返回
    ros::spin();

    return 0;  // 不会执行到这里
}
```

## 重要注释

| 行 | 内容 | 说明 |
|---|---|---|
| 1-6 | include 头文件 | `ego_replan_fsm.h` 定义了 EGOReplanFSM 类 |
| 11 | `ros::init` | 任何 ROS 程序的第一步 |
| 12 | `NodeHandle nh("~")` | 创建私有句柄，`~` 表示这个节点的私有命名空间 |
| 14 | 创建 EGOReplanFSM 对象 | **所有规划逻辑都封装在这个类里** |
| 16 | `rebo_replan.init(nh)` | 核心初始化函数，做参数读取、订阅/发布器创建、定时器启动 |
| 19 | `ros::spin()` | 进入回调处理循环，**阻塞在此** |

## 延伸理解

如果以后要调试 EGO，这个文件几乎不需要改。需要改的地方都在 `ego_replan_fsm.cpp` 的 `init()` 函数里——那里建立了所有订阅和发布关系。

主程序的另一个用法（注释掉了）是异步Spinner：
```cpp
ros::AsyncSpinner async_spinner(4);  // 4 个线程的异步Spinner
async_spinner.start();
ros::waitForShutdown();              // 等待 Ctrl+C
```
这种方式比 `ros::spin()` 更适合多线程场景，但原版默认用 `ros::spin()`。

## 与后续模块的关系

```
ego_planner_node.cpp
    ↓ 创建并初始化
EGOReplanFSM.init()
    ├── 创建 PlannerManager → 管理 GridMap 和 BsplineOptimizer
    ├── 创建 PlanningVisualization → 调试可视化
    ├── 创建定时器 → execFSMCallback (100Hz) + checkCollisionCallback (20Hz)
    └── 建立订阅/发布 → odom, waypoint, bspline 等
```

**一句话总结**：主程序 = 启动器 + 事件循环，所有智能都在 EGOReplanFSM 里。
