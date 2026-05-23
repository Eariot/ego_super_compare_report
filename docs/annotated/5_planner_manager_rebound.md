# planner_manager.cpp —— reboundReplan 核心三步
## 文件路径
`official/Fast-Drone-250/src/planner/plan_manage/src/planner_manager.cpp`

> 本文件聚焦 `reboundReplan()` 函数——EGO 局部 B-spline 优化的核心引擎。它接收起点/终点，输出优化后的 B-spline 轨迹，内部分三步：INIT（初始路径生成）→ OPTIMIZE（B-spline 优化）→ REFINE（时间重分配）。

---

## 整体数据流

```
callReboundReplan()
    │
    ├── getLocalTarget() → local_target_pt_（从全局轨迹取 planning_horizon 处点）
    │
    └── reboundReplan(start_pt, start_vel, start_acc,
                     local_target_pt, local_target_vel,
                     flag_polyInit, flag_randomPolyTraj)
             │
             ├── STEP 1: INIT —— 生成初始路径
             │      ├── flag_polyInit=true: 多项式轨迹生成
             │      └── flag_polyInit=false: 从当前轨迹延伸
             │
             ├── STEP 2: OPTIMIZE —— B-spline 优化
             │      └── BsplineOptimizeTrajRebound() → L-BFGS 求解
             │
             └── STEP 3: REFINE —— 时间重分配（单机模式）
                      └── refineTrajAlgo() → 如果速度/加速度超限，重新分配时间
```

---

## reboundReplan 函数详解（L49-340）

### 入口检查（L53-67）

```cpp
// EGOPlannerManager::reboundReplan —— 局部重规划核心函数
bool EGOPlannerManager::reboundReplan(
    Eigen::Vector3d start_pt,     // 当前无人机位置
    Eigen::Vector3d start_vel,   // 当前无人机速度
    Eigen::Vector3d start_acc,   // 当前无人机加速度
    Eigen::Vector3d local_target_pt,    // 局部目标点（来自 getLocalTarget）
    Eigen::Vector3d local_target_vel,   // 局部目标速度
    bool flag_polyInit,           // 是否用多项式生成初始路径
    bool flag_randomPolyTraj)     // 是否用随机扰动初始路径
{
    static int count = 0;
    printf("\033[47;30m\n[drone %d replan %d]==============================================\033[0m\n",
           pp_.drone_id, count++);

    // 安全检查：如果起点和终点太近（< 0.2m），放弃规划
    if ((start_pt - local_target_pt).norm() < 0.2)
    {
        cout << "Close to goal" << endl;
        continous_failures_count_++;
        return false;
    }

    // 将局部目标点传给优化器（用于计算 terminal cost）
    bspline_optimizer_->setLocalTargetPt(local_target_pt);

    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt, t_refine;  // 计时：各阶段耗时
```

### STEP 1: INIT —— 生成初始路径（L71-216）

#### 分支A：多项式初始路径（flag_polyInit=true）

```cpp
/*** STEP 1: INIT ***/
// 时间步长 ts = 控制点间距 / 最大速度 × 系数
// 距离远时：ts = ctrl_pt_dist / max_vel × 1.5（更紧凑）
// 距离近时：ts = ctrl_pt_dist / max_vel × 5.0（更宽松，避免超限）
double ts = (start_pt - local_target_pt).norm() > 0.1
                ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.5
                : pp_.ctrl_pt_dist / pp_.max_vel_ * 5;

vector<Eigen::Vector3d> point_set, start_end_derivatives;
static bool flag_first_call = true, flag_force_polynomial = false;
bool flag_regenerate = false;

do
{
    point_set.clear();
    start_end_derivatives.clear();
    flag_regenerate = false;

    if (flag_first_call || flag_polyInit || flag_force_polynomial)
    {
        // === 第一次调用或强制用多项式 ===
        flag_first_call = false;
        flag_force_polynomial = false;

        // 计算从起点到终点的预计飞行时间
        double dist = (start_pt - local_target_pt).norm();
        // 如果 v²/a > dist（速度平方/加速度 > 距离）→ 匀速段不存在
        // 否则 → 有匀速段的梯形速度曲线时间
        double time = (pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist)
                          ? sqrt(dist / pp_.max_acc_)
                          : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_
                            + 2 * pp_.max_vel_ / pp_.max_acc_;

        if (!flag_randomPolyTraj)
        {
            // 确定性路径：单段最小snap多项式
            // 输入：起点(位置+速度+加速度)、终点(位置+速度+加速度)
            // 输出：一条经过这两点的多项式轨迹
            gl_traj = PolynomialTraj::one_segment_traj_gen(
                start_pt, start_vel, start_acc,
                local_target_pt, local_target_vel,
                Eigen::Vector3d::Zero(),  // 终点加速度=0
                time);
        }
        else
        {
            // 随机路径：在中点加随机扰动，增加探索性
            // horizen_dir = 水平面垂直于运动方向的向量
            // vertical_dir = 竖直方向
            // 扰动幅度与连续失败次数有关（失败越多，扰动越大）
            Eigen::Vector3d horizen_dir =
                ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
            Eigen::Vector3d vertical_dir =
                ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
            Eigen::Vector3d random_inserted_pt = ...;  // 带随机扰动的中点
            Eigen::MatrixXd pos(3, 3);
            pos.col(0) = start_pt;
            pos.col(1) = random_inserted_pt;
            pos.col(2) = local_target_pt;
            Eigen::VectorXd t(2);
            t(0) = t(1) = time / 2;
            gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel,
                                                  start_acc, Eigen::Vector3d::Zero(), t);
        }

        // === 沿多项式轨迹采样控制点 ===
        double t;
        bool flag_too_far;
        ts *= 1.5;  // 先放大，再逐步缩小（见下面 do-while）
        do
        {
            ts /= 1.5;  // 逐步二分，找到恰好满足要求的 ts
            point_set.clear();
            flag_too_far = false;
            Eigen::Vector3d last_pt = gl_traj.evaluate(0);
            for (t = 0; t < time; t += ts)
            {
                Eigen::Vector3d pt = gl_traj.evaluate(t);
                // 检查相邻采样点距离：不能超过 ctrl_pt_dist × 1.5
                if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
                {
                    flag_too_far = true;
                    break;
                }
                last_pt = pt;
                point_set.push_back(pt);
            }
        } while (flag_too_far || point_set.size() < 7);  // 至少 7 个点

        // 记录边界条件：起点速度/加速度、终点速度/加速度
        t -= ts;
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
    }
```

#### 分支B：从当前轨迹延伸（flag_polyInit=false）

```cpp
    else
    {
        // === 基于当前轨迹延伸 ===
        double t;
        double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();

        // 建立弧长参数化：pseudo_arc_length[i] = 从 t_cur 到第 i 个采样点的累积距离
        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
            segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
            if (t > t_cur)
            {
                pseudo_arc_length.push_back(
                    (segment_point.back() - segment_point[segment_point.size() - 2]).norm()
                    + pseudo_arc_length.back());
            }
        }

        // 从当前轨迹末尾延伸到 local_target_pt，生成一段多项式过渡
        double poly_time =
            (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm()
            / pp_.max_vel_ * 2;
        if (poly_time > ts)
        {
            PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(
                local_data_.position_traj_.evaluateDeBoorT(t),
                local_data_.velocity_traj_.evaluateDeBoorT(t),
                local_data_.acceleration_traj_.evaluateDeBoorT(t),
                local_target_pt, local_target_vel,
                Eigen::Vector3d::Zero(), poly_time);
            // ... 将多项式点加入 segment_point ...
        }

        // 沿弧长均匀采样，生成控制点序列
        double sample_length = 0;
        double cps_dist = pp_.ctrl_pt_dist * 1.5;
        do
        {
            cps_dist /= 1.5;
            point_set.clear();
            sample_length = 0;
            // 根据累积弧长，在 segment_point 之间线性插值
            while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
            {
                // 线性插值：sample_length 落在 pseudo_arc_length[id] 和 [id+1] 之间
                point_set.push_back(interpolated_point);
                sample_length += cps_dist;
            }
            point_set.push_back(local_target_pt);  // 最后加入终点
        } while (point_set.size() < 7);

        // 边界条件
        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        // 如果初始路径异常长（超出 planning_horizon 3倍），强制用多项式重生成
        if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3)
        {
            flag_force_polynomial = true;
            flag_regenerate = true;
        }
    }
} while (flag_regenerate);  // 循环直到生成正常的初始路径
```

#### 初始路径 → 控制点（L218-224）

```cpp
    // 将采样点序列参数化为三次均匀 B-spline 的控制点
    Eigen::MatrixXd ctrl_pts, ctrl_pts_temp;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    // 初始化优化器中的控制点（建立碰撞方向向量等）
    vector<std::pair<int, int>> segments;
    segments = bspline_optimizer_->initControlPoints(ctrl_pts, true);

    t_init = ros::Time::now() - t_start;  // 记录 STEP1 耗时
    t_start = ros::Time::now();
```

---

### STEP 2: OPTIMIZE —— B-spline 优化（L227-286）

```cpp
/*** STEP 2: OPTIMIZE ***/
bool flag_step_1_success = false;
vector<vector<Eigen::Vector3d>> vis_trajs;

if (pp_.use_distinctive_trajs)
{
    // === 多轨迹候选模式 ===
    // 首先生成多条不同的初始路径（different distinctive trajectories）
    std::vector<ControlPoints> trajs = bspline_optimizer_->distinctiveTrajs(segments);
    cout << "\033[1;33m" << "multi-trajs=" << trajs.size() << "\033[1;0m" << endl;

    // 逐条优化，取代价最低的那条
    double final_cost, min_cost = 999999.0;
    for (int i = trajs.size() - 1; i >= 0; i--)
    {
        if (bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts_temp, final_cost, trajs[i], ts))
        {
            flag_step_1_success = true;
            if (final_cost < min_cost)
            {
                min_cost = final_cost;
                ctrl_pts = ctrl_pts_temp;
            }
            vis_trajs.push_back(point_set);  // 可视化每条候选轨迹
        }
    }

    t_opt = ros::Time::now() - t_start;
    visualization_->displayMultiInitPathList(vis_trajs, 0.2);
}
else
{
    // === 单轨迹优化模式（默认）===
    // 调用 BsplineOptimizeTrajRebound：L-BFGS 求解
    // 代价函数 = λ1×f_smooth + λ2×f_dist + λ3×f_feas + λ2×f_swarm + λ2×f_terminal
    flag_step_1_success = bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
    t_opt = ros::Time::now() - t_start;
    visualization_->displayInitPathList(point_set, 0.2, 0);  // 可视化初始路径
}

cout << "plan_success=" << flag_step_1_success << endl;

// 优化失败：连续失败计数 +1
if (!flag_step_1_success)
{
    visualization_->displayOptimalList(ctrl_pts, 0);
    continous_failures_count_++;
    return false;
}
```

---

### STEP 3: REFINE —— 时间重分配（L289-325）

```cpp
    t_start = ros::Time::now();

    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);  // 用优化后的控制点重建 B-spline
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    // 注意：只在单机模式（drone_id <= 0）下启用
    // 多机模式下 disable（保持轨迹同步）
    if (pp_.drone_id <= 0)
    {
        double ratio;
        bool flag_step_2_success = true;

        // 检查速度/加速度是否超限
        // ratio > 1.0 表示需要更长时间才能满足限制
        if (!pos.checkFeasibility(ratio, false))
        {
            cout << "Need to reallocate time." << endl;

            Eigen::MatrixXd optimal_control_points;
            // refineTrajAlgo：重新分配 B-spline 的时间参数
            // ratio > 1 → 增加时间；ratio < 1 → 可缩短时间
            flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives,
                                                  ratio, ts, optimal_control_points);
            if (flag_step_2_success)
                pos = UniformBspline(optimal_control_points, 3, ts);
        }

        if (!flag_step_2_success)
        {
            // 时间重分配后仍撞障碍物
            printf("\033[34mThis refined trajectory hits obstacles. "
                   "It doesn't matter if appears occasionally. "
                   "But if continuously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
            continous_failures_count_++;
            return false;
        }
    }
    else
    {
        static bool print_once = true;
        if (print_once)
        {
            print_once = false;
            ROS_ERROR("IN SWARM MODE, REFINE DISABLED!");
        }
    }

    t_refine = ros::Time::now() - t_start;
```

---

### 收尾（L328-340）

```cpp
    // 保存规划结果到 local_data_
    updateTrajInfo(pos, ros::Time::now());

    // 统计信息：总耗时、各阶段耗时、平均耗时
    static double sum_time = 0;
    static int count_success = 0;
    sum_time += (t_init + t_opt + t_refine).toSec();
    count_success++;
    cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m"
         << ",optimize:" << (t_init + t_opt).toSec()
         << ",refine:" << t_refine.toSec()
         << ",avg_time=" << sum_time / count_success << endl;

    // 成功！重置连续失败计数
    continous_failures_count_ = 0;
    return true;
}
```

---

## refineTrajAlgo —— 时间重分配（L521-540）

```cpp
bool EGOPlannerManager::refineTrajAlgo(UniformBspline &traj,
                                        vector<Eigen::Vector3d> &start_end_derivative,
                                        double ratio, double &ts,
                                        Eigen::MatrixXd &optimal_control_points)
{
    // ratio > 1.0：速度或加速度超限，需要更多时间
    // 1. 按 ratio 延长轨迹时间
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, dt, t_inc);

    // 2. 重建 B-spline
    traj = UniformBspline(ctrl_pts, 3, ts);

    // 3. 重新生成参考点（用于 fitness cost）
    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
        bspline_optimizer_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    // 4. 再次优化（这次用 fitness cost 而非 distance cost）
    bool success = bspline_optimizer_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);
    return success;
}
```

---

## initPlanModules —— 模块初始化（L15-43）

```cpp
void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
{
    // 读取规划参数
    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);
    nh.param("manager/use_distinctive_trajs", pp_.use_distinctive_trajs, false);
    nh.param("manager/drone_id", pp_.drone_id, -1);

    local_data_.traj_id_ = 0;

    // 创建 GridMap（地图管理）
    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    // 创建 BsplineOptimizer（B-spline 优化器）
    bspline_optimizer_.reset(new BsplineOptimizer);
    bspline_optimizer_->setParam(nh);                // 读取优化参数
    bspline_optimizer_->setEnvironment(grid_map_, obj_predictor_);  // 注入地图
    bspline_optimizer_->a_star_.reset(new AStar);     // 创建 A* 搜索器
    bspline_optimizer_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

    visualization_ = vis;
}
```

---

## 各阶段耗时参考

| 阶段 | 典型耗时 | 占比 | 优化方向 |
|---|---|---|---|
| STEP 1 (INIT) | 0.1-0.5 ms | ~5-10% | 采样点数量、ts 二分次数 |
| STEP 2 (OPTIMIZE) | 2-10 ms | ~80-90% | L-BFGS 迭代次数（max_iterations=200）、控制点数量 |
| STEP 3 (REFINE) | 1-3 ms | ~10% | refineTrajAlgo 是否被触发 |

**`continous_failures_count_` 的作用**：
- 连续失败越多，`flag_randomPolyTraj` 的扰动幅度越大
- 用于打破局部最优，提高规划成功率
