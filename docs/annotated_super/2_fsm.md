# fsm.cpp / fsm.h —— SUPER 的 6 状态有限状态机

## 文件路径
- 头文件：`super_planner/include/fsm/fsm.h`
- 实现：`super_planner/src/super_core/fsm.cpp`（230 行）
- 派生类（ROS1 适配器）：`super_planner/include/ros_interface/ros1/fsm_ros1.hpp`

## 核心理解

SUPER 把 FSM 的"骨架"（基类 `fsm::Fsm`）和"输入输出"（派生类 `FsmRos1` / `FsmRos2`）严格分离：

- `Fsm` 只关心"逻辑状态"：何时调用 PlanFromRest、何时切到 FOLLOW_TRAJ、何时进入 EMER_STOP；
- `FsmRos1` 实现 `publishPolyTraj()` / `publishCurPoseToPath()` / `resetVisualizedPath()` —— 把基类标记的事件转换成具体的 ROS topic 发布。

这样的好处：**同一份 FSM 代码同时支持 ROS1 和 ROS2**。

---

## 状态定义（fsm.h L75-90）

```cpp
enum MACHINE_STATE {
    INIT = 0,           // 上电默认状态
    WAIT_GOAL,          // 等待目标点（含等 odom）
    YAWING,             // 旋转调整航向（占位，当前实现未启用）
    GENERATE_TRAJ,      // 第一次规划（plan-from-rest）
    FOLLOW_TRAJ,        // 跟随轨迹中（重规划在此触发）
    EMER_STOP           // 应急停止
};

vector<string> MACHINE_STATE_STR{
    "INIT", "WAIT_GOAL", "YAWING", "GENERATE_TRAJ", "FOLLOW_TRAJ", "EMER_STOP"
};
```

对应的 EGO 是 7 个状态（INIT / WAIT_TARGET / GEN_NEW_TRAJ / REPLAN_TRAJ / EXEC_TRAJ / EMERGENCY_STOP / SEQUENTIAL_START）。最大区别：

- SUPER 把"生成新轨迹"和"重规划"压缩成 GENERATE_TRAJ + FOLLOW_TRAJ 两态——重规划不切状态，而是在 FOLLOW_TRAJ 持续状态里由独立的 `callReplanOnce` 触发；
- SUPER 没有多机模式（无 SEQUENTIAL_START），不广播自身轨迹给其他无人机；
- SUPER 多了 YAWING 占位状态（注释中保留，逻辑未启用，预留给"先转头再起飞"场景）。

---

## 主回调 callMainFsmOnce（fsm.cpp L88-174）

这是 SUPER 状态机的"心脏"，由 ROS 定时器每隔 `1/replan_rate` 秒触发一次（典型 10 Hz）。

```cpp
void Fsm::callMainFsmOnce() {
    if (stop) return;

    static double fsm_start_time = ros_ptr_->getSimTime();
    double cur_t = (ros_ptr_->getSimTime() - fsm_start_time);
    static double last_print_t = 0.0;
    planner_ptr_->getRobotState(robot_state_);  // 从 ROG-Map 拿最新机体状态

    // 每 1 秒打印一次状态（避免日志爆炸）
    if (cur_t - last_print_t > 1.0) {
        last_print_t = cur_t;
        // ... 检查 odom 时戳，超 0.1s 没更新就报警 ...
        cout << " -- [Fsm " << cur_t << "] Current state: "
             << MACHINE_STATE_STR[machine_state_] << endl;
    }

    switch (machine_state_) {
        // ============ INIT ============
        case INIT: {
            if (!started_) return;        // 还没收到任何目标 → 不动
            // 这里依赖 setGoalPosiAndYaw 设置 started_=true
            ChangeState("MainFsmCallback", WAIT_GOAL);
            break;
        }

        // ============ WAIT_GOAL ============
        case WAIT_GOAL: {
            if (!gi_.new_goal) return;    // 没新目标 → 等
            ChangeState("MainFsmCallback", GENERATE_TRAJ);
            resetVisualizedPath();         // 派生类清空 RViz 中累积的轨迹路径
            break;
        }

        // ============ GENERATE_TRAJ ============
        // 这是从静止状态规划的入口，对应 PlanFromRest
        case GENERATE_TRAJ: {
            // 1. 已经离目标够近 → 直接结束
            if (closeToGoal(0.1)) {
                ChangeState("MainFsmCallback", WAIT_GOAL);
                gi_.new_goal = false;
                finish_plan = true;
                return;
            }

            // 2. 调 SuperPlanner 主入口
            int retcode = planner_ptr_->PlanFromRest(gi_.goal_p, gi_.goal_yaw, gi_.new_goal);

            // 3. 如果 SuperPlanner 内部判定 goal 不可达（在障碍物中）
            if (!planner_ptr_->goalValid()) {
                ChangeState("MainFsmCallback", WAIT_GOAL);
                return;
            }

            // 4. 成功 → 进入 FOLLOW_TRAJ
            if (retcode == SUCCESS || retcode == FINISH) {
                gi_.new_goal = false;
                plan_from_rest_ = true;       // 让 callReplanOnce 跳过这一帧（避免重复规划）
                finish_plan = (retcode == FINISH);
                publishPolyTraj();            // 派生类把 cmd_traj_info_ 发到 /poly_traj
                ChangeState("MainFsmCallback", FOLLOW_TRAJ);
            } else {
                cout << " -- [Fsm] PlanFromRest failed, try replan." << endl;
                // 留在 GENERATE_TRAJ，下一帧再尝试
            }
            replan_logs_.push_back(planner_ptr_->getLatestReplanLog());
            break;
        }

        // ============ FOLLOW_TRAJ ============
        // 注意：状态本身不做规划，规划由 callReplanOnce 处理
        case FOLLOW_TRAJ: {
            publishCurPoseToPath();   // 把当前 odom 累积到 RViz Path 上
            break;
        }

        // ============ EMER_STOP ============
        // SUPER 的应急停止逻辑很简单：直接退到 WAIT_GOAL，等用户重新点击目标
        case EMER_STOP: {
            ChangeState("MainFsmCallback", WAIT_GOAL);
            break;
        }
        default: break;
    }
}
```

> 关键观察：SUPER 的"应急停止"不像 EGO 那样发布 emergency 轨迹——它依赖 **备份轨迹** 自动接管，让飞机在已规划好的 backup traj 上减速到悬停。EMER_STOP 状态本身只是"清理 + 让用户重新点目标"。

---

## 重规划回调 callReplanOnce（fsm.cpp L45-86）

```cpp
void Fsm::callReplanOnce() {
    if (stop) return;

    // 早退条件 1：必须在 FOLLOW_TRAJ
    if (machine_state_ != FOLLOW_TRAJ) return;

    // 早退条件 2：已经到达目标
    if (finish_plan) return;

    // 早退条件 3：刚做完 plan-from-rest，跳过这一帧
    // 这个标志在 GENERATE_TRAJ 的成功分支被置为 true
    // 目的：避免 PlanFromRest 刚结束就立即触发 ReplanOnce 造成轨迹抖动
    if (plan_from_rest_) {
        plan_from_rest_ = false;
        return;
    }

    // 修正目标：把目标点 shift 到最近的"非占据"格（防止用户点到障碍物里）
    planner_ptr_->getMap()->getNearestInfCellNot(GridType::OCCUPIED,
                                                  gi_.goal_p, gi_.goal_p, 3.0);

    TimeConsuming replan_once_time("replan_once_time", false);
    RET_CODE ret_code = planner_ptr_->ReplanOnce(gi_.goal_p, gi_.goal_yaw, gi_.new_goal);

    /* 根据返回码切换状态 */
    if (ret_code == EMER) {
        ChangeState("ReplanTimerCallback", EMER_STOP);
    } else if (ret_code == NEW_TRAJ) {
        // 上一段轨迹快执行完了 → 退回 GENERATE_TRAJ 重新 plan-from-rest
        ChangeState("ReplanTimerCallback", GENERATE_TRAJ);
    } else if (ret_code == SUCCESS || ret_code == FINISH) {
        gi_.new_goal = false;
        publishPolyTraj();   // 把更新后的 cmd_traj_info_ 发出去
    }
    // FAILED 时不切状态：留在 FOLLOW_TRAJ，下一帧继续
    // ⚠ 这是 SUPER 容错的关键：单次失败不会立刻急停，因为 backup_traj 还在保护飞行

    planner_ptr_->getModuleTimeConsuming(log_module_time);
    log_module_time[log_module_time.size() - 2] = replan_once_time.stop();
    replan_logs_.push_back(planner_ptr_->getLatestReplanLog());
    WriteTimeToLog();   // 把各模块耗时写到 log_csv（用于后期画图）
}
```

### ReplanOnce 返回码语义

| ret_code | 含义 | FSM 反应 |
|---|---|---|
| SUCCESS | 新 exp + backup 都生成成功 | 不切状态，更新轨迹 |
| FINISH | 整段已是 known free，不需要 backup | 不切状态，更新轨迹 |
| NEW_TRAJ | 当前轨迹时间已用完 | → GENERATE_TRAJ |
| EMER | 探索轨迹生成失败且当前已在 backup traj 上 | → EMER_STOP |
| FAILED | 单次规划失败（最常见） | 不切状态，下帧重试 |

---

## 目标接收 setGoalPosiAndYaw（fsm.cpp L184-223）

```cpp
void Fsm::setGoalPosiAndYaw(const Vec3f &p, const Quatf &q) {
    auto click_point = p;

    // 1. 强制目标高度（如果 click_height > -5，把 z 锁到该值）
    //    防止 RViz 2D Nav Goal 给出 z=0 导致目标在地面下
    if (cfg_.click_height > -5) {
        click_point.z() = cfg_.click_height;
    }

    // 2. 把目标点平移到最近"非占据"格（搜索半径 3 m）
    //    与 EGO 的 z=1.0 硬编码相比更智能——SUPER 永远不会被"卡在障碍物里"
    if (planner_ptr_->getMap()->getNearestInfCellNot(GridType::OCCUPIED,
                                                     click_point, gi_.goal_p, 3.0)) {
        cout << " -- [Fsm] Get goal at " << gi_.goal_p.transpose() << endl;
    } else {
        // 3 m 内没有自由空间 → 拒绝
        fmt::print(fg(fmt::color::indian_red), "Goal is deeply occupied, skip this goal.\n");
        return;
    }

    // 3. 太近的目标直接忽略
    if ((robot_state_.p - gi_.goal_p).norm() < 0.1) return;

    // 4. 处理 yaw：可选启用
    if (cfg_.click_yaw_en) {
        if (isnan(q.w()) || ...) {
            gi_.goal_yaw = NAN;          // 没收到合法四元数 → 自由 yaw
        } else {
            gi_.goal_yaw = geometry_utils::get_yaw_from_quaternion(q);
        }
    } else {
        gi_.goal_yaw = NAN;              // 配置关闭 yaw → 永远 NAN（自由 yaw）
    }

    started_ = true;          // 让 INIT → WAIT_GOAL 转换可以发生
    gi_.new_goal = true;       // 让 WAIT_GOAL → GENERATE_TRAJ 转换可以发生
}
```

---

## ChangeState（fsm.cpp L225-229）

```cpp
void Fsm::ChangeState(const string &call_func, const MACHINE_STATE &new_state) {
    fmt::print(fg(fmt::color::green), " -- [Fsm]: [{}] change state from [{}] to [{}].\n",
               call_func, MACHINE_STATE_STR[int(machine_state_)], MACHINE_STATE_STR[int(new_state)]);
    machine_state_ = new_state;
}
```

仅做日志 + 赋值。注意 SUPER 没有像 EGO 那样跟踪"连续调用次数"——因为 SUPER 的失败容错不需要这个计数器，它依靠 backup_traj。

---

## 状态机完整流程图

```
                                       click 目标 / setGoalPosiAndYaw
                                              ↓
                                       started_=true, gi_.new_goal=true
                                              ↓
              odom 已收到                   有新目标                 PlanFromRest 成功
   ┌────────┐ ─────────▶ ┌──────────┐ ─────────▶ ┌──────────────┐ ─────────▶ ┌──────────┐
   │  INIT  │            │WAIT_GOAL │            │GENERATE_TRAJ │            │FOLLOW_TRAJ│
   └────────┘            └──────────┘            └──────────────┘            └──────────┘
                              ↑                          │                         │
                              │                          │ 失败                    │
                              │                          ↓                         │
                              │                  留在 GENERATE_TRAJ                │
                              │                                                   │
                              │  ChangeState                                       │
                              │   ↑   ↑                                            │ callReplanOnce
                              │   │   │ ret_code==FINISH/SUCCESS（更新轨迹）       │
                              │   │   └──────────────────────────────────────────┘
                              │   │
                              │   │   ret_code==NEW_TRAJ → GENERATE_TRAJ
                              │   │
                              │   │   ret_code==EMER
                              │   ↓
                              │ ┌──────────┐
                              └─│EMER_STOP │ ChangeState 后立刻退回 WAIT_GOAL
                                └──────────┘
```

---

## 与 EGO 状态机的设计差异

| 项目 | EGO（7 状态） | SUPER（6 状态） |
|---|---|---|
| 重规划是否切状态 | 是（EXEC_TRAJ ↔ REPLAN_TRAJ 反复切换） | 否（FOLLOW_TRAJ 持续状态，重规划由独立定时器） |
| 失败响应 | 立即重试 + 急停 | 重试，靠 backup_traj 兜底，少急停 |
| 多机协调 | SEQUENTIAL_START（drone_id≥1 等前机轨迹） | 不支持多机 |
| 目标处理 | RViz 2D Nav Goal 的 z 写死 1.0 | 用 `click_height` 配置 + `getNearestInfCellNot` 自动避开占据格 |
| 急停轨迹 | callEmergencyStop 实时生成"全零"B-spline | 直接走 backup_traj，不需要单独的急停轨迹 |
| 安全检查 | 独立的 20Hz `checkCollisionCallback` | 由 ReplanOnce 内部对当前 cmd_traj 做碰撞检查 |
