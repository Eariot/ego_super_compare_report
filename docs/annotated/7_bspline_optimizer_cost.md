# bspline_optimizer.cpp —— EGO 代价函数体系
## 文件路径
`official/Fast-Drone-250/src/planner/bspline_opt/src/bspline_optimizer.cpp`

> BsplineOptimizer 是 EGO 的核心数值优化器。它接收初始化后的 B-spline 控制点，通过 L-BFGS 算法最小化代价函数，输出无碰撞且满足动力学约束的优化轨迹。本篇聚焦代价函数的设计。

---

## 代价函数总体架构

EGO 使用**加权求和**的方式将多个目标合并为一个总代价：

```
总代价 = λ1 × f_smoothness      (轨迹平滑)
       + λ2 × f_distance        (障碍物避障)
       + λ3 × f_feasibility     (动力学可行性)
       + λ2 × f_swarm           (多机避碰)
       + λ2 × f_terminal        (终点收敛)
```

### combineCostRebound（L1672-1705）—— rebound 阶段总代价

```cpp
// BsplineOptimizer::combineCostRebound —— rebind/replan 阶段的总代价函数
// 每次 L-BFGS 迭代中被调用，计算代价和梯度
void BsplineOptimizer::combineCostRebound(const double *x, double *grad, double &f_combine, const int n)
{
    // x：L-BFGS 优化变量 = 控制点坐标（展平为 1D 数组）
    // grad：输出 = 代价函数对 x 的梯度
    // n：变量数量 = 3 × (控制点总数 - order_)

    // 1. 将展平数组写回 Eigen 矩阵格式
    memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

    /* 2. 分别计算各子代价和梯度 */
    double f_smoothness, f_distance, f_feasibility, f_swarm, f_terminal;

    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_distance   = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_swarm       = Eigen::MatrixXd::Zero(3, cps_.size);
    Eigen::MatrixXd g_terminal    = Eigen::MatrixXd::Zero(3, cps_.size);

    // 各子代价计算（每个函数同时返回代价值和梯度）
    calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
    calcDistanceCostRebound(cps_.points, f_distance, g_distance, iter_num_, f_smoothness);
    calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);
    calcSwarmCost(cps_.points, f_swarm, g_swarm);
    calcTerminalCost(cps_.points, f_terminal, g_terminal);

    /* 3. 加权求和 */
    f_combine = lambda1_ * f_smoothness
              + new_lambda2_ * f_distance
              + lambda3_ * f_feasibility
              + new_lambda2_ * f_swarm       // swarm 用 new_lambda2_（可能动态调整）
              + lambda2_ * f_terminal;

    /* 4. 梯度加权求和 */
    Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness
                             + new_lambda2_ * g_distance
                             + lambda3_ * g_feasibility
                             + new_lambda2_ * g_swarm
                             + lambda2_ * g_terminal;

    /* 5. 写回 L-BFGS 格式（只优化中间控制点，首尾固定）*/
    memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
}
```

### combineCostRefine（L1707-1731）—— refine 阶段总代价

```cpp
// refine 阶段：只优化中间部分，用 fitness cost 替代 distance cost
// 目的：让轨迹更贴合原始路径，而非激进避障
void BsplineOptimizer::combineCostRefine(const double *x, double *grad, double &f_combine, const int n)
{
    memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

    double f_smoothness, f_fitness, f_feasibility;
    Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_fitness    = Eigen::MatrixXd::Zero(3, cps_.points.cols());
    Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.points.cols());

    calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
    calcFitnessCost(cps_.points, f_fitness, g_fitness);
    calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);

    // refine 阶段代价 = λ1×smooth + λ4×fitness + λ3×feasibility
    // 注意：没有 distance, swarm, terminal！
    f_combine = lambda1_ * f_smoothness + lambda4_ * f_fitness + lambda3_ * f_feasibility;

    Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + lambda4_ * g_fitness + lambda3_ * g_feasibility;
    memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
}
```

---

## f_smoothness —— 平滑代价（calcSmoothnessCost, L935-974）

### 核心思想

B-spline 的三阶导数（jerk，加加速度）与轨迹平滑性直接相关：
- **jerk 最小化** → 轨迹运动平缓，无突兀抖动
- 数学形式：`q[i+3] - 3q[i+2] + 3q[i+1] - q[i]`（三次差分）

```cpp
// BsplineOptimizer::calcSmoothnessCost —— 轨迹平滑代价
// 使用 jerk（二阶加速度变化率）作为平滑性度量
void BsplineOptimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                           Eigen::MatrixXd &gradient,
                                           bool falg_use_jerk /* = true*/)
{
    cost = 0.0;

    if (falg_use_jerk)  // 默认使用 jerk
    {
        Eigen::Vector3d jerk, temp_j;

        // 对每组相邻的 4 个控制点，计算 jerk
        // 对于三次均匀 B-spline：速度 ∝ (q[i+1]-q[i])/ts
        //                    加速度 ∝ (q[i+2]-2q[i+1]+q[i])/ts²
        //                    加加速度(jerk) ∝ (q[i+3]-3q[i+2]+3q[i+1]-q[i])/ts³
        for (int i = 0; i < q.cols() - 3; i++)
        {
            // B-spline 性质：第 i 个控制点对 [i, i+3] 时间区间的jerk 有贡献
            jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);

            // 代价 = Σ ||jerk||²（所有 jerk 平方和最小化）
            cost += jerk.squaredNorm();

            // 梯度：d(cost)/d(q[i]) = 2 * jerk * d(jerk)/d(q[i])
            // 通过链式法则展开：
            // q[i]   出现在 jerk = q[i+3]-3q[i+2]+3q[i+1]-q[i] 中，系数 = -1
            temp_j = 2.0 * jerk;
            gradient.col(i + 0) += -temp_j;   // d/dq[i]   = -2*jerk
            gradient.col(i + 1) +=  3.0 * temp_j;  // d/dq[i+1] = +6*jerk
            gradient.col(i + 2) += -3.0 * temp_j;  // d/dq[i+2] = -6*jerk
            gradient.col(i + 3) +=  temp_j;   // d/dq[i+3] = +2*jerk
        }
    }
    else  // 备选：使用加速度
    {
        Eigen::Vector3d acc, temp_acc;
        for (int i = 0; i < q.cols() - 2; i++)
        {
            acc = q.col(i + 2) - 2 * q.col(i + 1) + q.col(i);  // 二阶差分
            cost += acc.squaredNorm();
            temp_acc = 2.0 * acc;
            gradient.col(i + 0) += temp_acc;     // d/dq[i]   = +2*acc
            gradient.col(i + 1) += -2.0 * temp_acc;  // d/dq[i+1] = -4*acc
            gradient.col(i + 2) += temp_acc;     // d/dq[i+2] = +2*acc
        }
    }
}
```

---

## f_distance —— 避障代价（calcDistanceCostRebound, L864-903）

### 核心思想

障碍物附近的控制点应被推离障碍物。使用**距离惩罚函数**：

```
      cost
        │
        │    ┌─ cubic (d < demarcation)
   c   │   /
        │  /
   ─────┼─┘─────────────────────────────── clearance (d = dist0)
        │
        └────────────────────────────────── d (到最近障碍物的距离)
           0          demarcation
```

```cpp
// BsplineOptimizer::calcDistanceCostRebound —— 碰撞惩罚代价
void BsplineOptimizer::calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost,
                                                Eigen::MatrixXd &gradient,
                                                int iter_num, double smoothness_cost)
{
    cost = 0.0;
    int end_idx = q.cols() - order_;  // 只优化中间控制点

    // demarcation：分段函数的拐点
    // dist < demarcation：使用三次惩罚（更陡峭）
    // dist > demarcation：使用二次惩罚（更平缓）
    double demarcation = cps_.clearance;  // 典型值 0.2m
    double a = 3 * demarcation, b = -3 * pow(demarcation, 2), c = pow(demarcation, 3);

    force_stop_type_ = DONT_STOP;

    // 迭代后期（iter_num > 3 且轨迹已经足够平滑）
    // → 触发碰撞反弹（collision rebound）：检查障碍物，更新 base_point/direction
    if (iter_num > 3 && smoothness_cost / (cps_.size - 2 * order_) < 0.1)
    {
        check_collision_and_rebound();  // 更新碰撞约束方向
    }

    /*** 计算每个控制点到所有障碍物的距离代价 ***/
    for (auto i = order_; i < end_idx; ++i)
    {
        for (size_t j = 0; j < cps_.direction[i].size(); ++j)
        {
            // dist：控制点沿"避障方向"到障碍物的有符号距离
            // base_point[i][j]：第 i 个控制点对应的第 j 个障碍物约束的参考点
            // direction[i][j]：避障方向（由 check_collision_and_rebound 计算）
            double dist = (cps_.points.col(i) - cps_.base_point[i][j]).dot(cps_.direction[i][j]);

            double dist_err = cps_.clearance - dist;  // err > 0 → 距障碍物太近
            Eigen::Vector3d dist_grad = cps_.direction[i][j];

            if (dist_err < 0)
            {
                // 距离足够远（dist > clearance）：无惩罚
            }
            else if (dist_err < demarcation)
            {
                // 较近：使用三次惩罚函数（更平滑的过渡）
                cost += pow(dist_err, 3);
                gradient.col(i) += -3.0 * dist_err * dist_err * dist_grad;
            }
            else
            {
                // 很远但仍 < demarcation：使用二次 + 一次多项式
                // 惩罚值 = a·d² + b·d + c
                cost += a * dist_err * dist_err + b * dist_err + c;
                gradient.col(i) += -(2.0 * a * dist_err + b) * dist_grad;
            }
        }
    }
}
```

---

## f_fitness —— 路径跟随代价（calcFitnessCost, L905-933）

### 核心思想

Refine 阶段使用 fitness cost 而非 distance cost：
- 轨迹应尽量贴合原始参考路径（初始多项式轨迹）
- 使用椭圆距离度量（沿路径方向和垂直方向权重不同）

```cpp
// BsplineOptimizer::calcFitnessCost —— 路径跟随代价（用于 refine 阶段）
// f = |x·v|²/a² + |x×v|²/b²
//   x = 控制点(De Boor中心) - 参考点
//   v = 路径方向单位向量
//   a = 沿路径方向容许偏差（25m，较大）
//   b = 垂直路径方向容许偏差（1m，较小）→ 重点约束垂直偏差
void BsplineOptimizer::calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
{
    cost = 0.0;
    int end_idx = q.cols() - order_;

    // a²=25, b²=1：垂直路径方向约束更严格
    double a2 = 25, b2 = 1;

    for (auto i = order_ - 1; i < end_idx + 1; ++i)
    {
        // De Boor 插值中心点（控制点不是轨迹上的点，用 De Boor 求中心）
        Eigen::Vector3d x = (q.col(i - 1) + 4 * q.col(i) + q.col(i + 1)) / 6.0 - ref_pts_[i - 1];

        // 路径方向单位向量（参考路径方向）
        Eigen::Vector3d v = (ref_pts_[i] - ref_pts_[i - 2]).normalized();

        // 沿路径方向偏差 + 垂直路径方向偏差
        double xdotv = x.dot(v);
        Eigen::Vector3d xcrossv = x.cross(v);

        // 椭圆距离公式
        double f = pow(xdotv, 2) / a2 + pow(xcrossv.norm(), 2) / b2;
        cost += f;

        // 计算梯度
        Eigen::Matrix3d m;
        m << 0, -v(2), v(1),
             v(2), 0, -v(0),
            -v(1), v(0), 0;
        Eigen::Vector3d df_dx = 2 * xdotv / a2 * v + 2 / b2 * m * xcrossv;

        // B-spline 梯度链式法则
        gradient.col(i - 1) += df_dx / 6;
        gradient.col(i)     += 4 * df_dx / 6;
        gradient.col(i + 1) += df_dx / 6;
    }
}
```

---

## f_feasibility —— 动力学可行性（calcFeasibilityCost, L994-1196）

### 核心思想

轨迹的速度和加速度必须在限制范围内。使用**软约束**：超出限制时施加惩罚。

```cpp
// BsplineOptimizer::calcFeasibilityCost —— 速度/加速度约束代价
// 速度限制：|vi| ≤ max_vel_
// 加速度限制：|ai| ≤ max_acc_
void BsplineOptimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                             Eigen::MatrixXd &gradient)
{
    cost = 0.0;
    double ts = bspline_interval_;    // B-spline 时间步长
    double ts_inv2 = 1 / ts / ts;    // 预计算：1/ts²（用于加速度换算）

    /* ---- 速度可行性检查 ---- */
    for (int i = 0; i < q.cols() - 1; i++)
    {
        // 速度 = 位移差 / 时间步长
        Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;

        for (int j = 0; j < 3; j++)
        {
            if (vi(j) > max_vel_)
            {
                // 速度超上限 → 惩罚
                double diff = vi(j) - max_vel_;
                cost += pow(diff, 2) * ts_inv2;
                gradient(j, i + 0) += -2 * diff / ts * ts_inv2;
                gradient(j, i + 1) +=  2 * diff / ts * ts_inv2;
            }
            else if (vi(j) < -max_vel_)
            {
                // 速度超下限（反向超速）
                double diff = vi(j) + max_vel_;
                cost += pow(diff, 2) * ts_inv2;
                gradient(j, i + 0) += -2 * diff / ts * ts_inv2;
                gradient(j, i + 1) +=  2 * diff / ts * ts_inv2;
            }
        }
    }

    /* ---- 加速度可行性检查 ---- */
    for (int i = 0; i < q.cols() - 2; i++)
    {
        // 加速度 = 二阶差分 × 1/ts²
        Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

        for (int j = 0; j < 3; j++)
        {
            if (ai(j) > max_acc_)
            {
                double diff = ai(j) - max_acc_;
                cost += pow(diff, 2);
                gradient(j, i + 0) +=  2 * diff * ts_inv2;
                gradient(j, i + 1) += -4 * diff * ts_inv2;
                gradient(j, i + 2) +=  2 * diff * ts_inv2;
            }
            else if (ai(j) < -max_acc_)
            {
                double diff = ai(j) + max_acc_;
                cost += pow(diff, 2);
                gradient(j, i + 0) +=  2 * diff * ts_inv2;
                gradient(j, i + 1) += -4 * diff * ts_inv2;
                gradient(j, i + 2) +=  2 * diff * ts_inv2;
            }
        }
    }
}
```

---

## f_terminal —— 终点收敛代价（calcTerminalCost, L976-992）

### 核心思想

局部规划的目标是让轨迹终点尽量接近 `local_target_pt_`：

```cpp
// BsplineOptimizer::calcTerminalCost —— 终点约束代价
// 代价 = || DeBoor终点 - local_target_pt_ ||²
void BsplineOptimizer::calcTerminalCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
{
    cost = 0.0;

    // B-spline 终点 = (q[n-3] + 4*q[n-2] + q[n-1]) / 6
    Eigen::Vector3d q_3 = q.col(q.cols() - 3);
    Eigen::Vector3d q_2 = q.col(q.cols() - 2);
    Eigen::Vector3d q_1 = q.col(q.cols() - 1);

    // De Boor 插值得到 B-spline 在末端的值
    dq = 1 / 6.0 * (q_3 + 4 * q_2 + q_1) - local_target_pt_;
    cost += dq.squaredNorm();

    // 梯度（通过 De Boor 的权重传播）
    gradient.col(q.cols() - 3) += 2 * dq * (1 / 6.0);
    gradient.col(q.cols() - 2) += 2 * dq * (4 / 6.0);
    gradient.col(q.cols() - 1) += 2 * dq * (1 / 6.0);
}
```

---

## 代价函数参数速查表

| 参数 | 典型值 | 作用对象 | 含义 |
|---|---|---|---|
| `lambda1_` (λ_smooth) | 1.0 | f_smoothness | 平滑权重：越大轨迹越平滑 |
| `lambda2_` (λ_collision) | 1.0 | f_distance, f_swarm, f_terminal | 避障权重：越大越激进避障 |
| `lambda3_` (λ_feasibility) | 1.0 | f_feasibility | 可行性权重：越大越严格遵守速度/加速度限制 |
| `lambda4_` (λ_fitness) | 1.0 | f_fitness | 跟随权重（refine 阶段）：越大越贴近原路径 |
| `dist0_` | 0.2m | f_distance | 安全距离基准：小于此距离开始惩罚 |
| `swarm_clearance_` | 0.5m | f_swarm | 多机安全距离 |
| `max_vel_` | 1.0 m/s | f_feasibility | 最大速度限制 |
| `max_acc_` | 2.0 m/s² | f_feasibility | 最大加速度限制 |

---

## 优化器核心：L-BFGS 算法（rebound_optimize, L1433-1600）

```cpp
// BsplineOptimizer::rebound_optimize —— L-BFGS 求解总代价最小化
bool BsplineOptimizer::rebound_optimize(double &final_cost)
{
    iter_num_ = 0;
    int start_id = order_;     // 前 order_ 个控制点固定
    int end_id = cps_.size;    // 后 order_ 个控制点自由（Free end）
    variable_num_ = 3 * (end_id - start_id);  // 优化变量数量

    // 初始化 L-BFGS 参数
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
    lbfgs_params.mem_size = 16;      // 内存大小（历史梯度数）
    lbfgs_params.max_iterations = 200;  // 最大迭代次数
    lbfgs_params.g_epsilon = 0.01;     // 梯度收敛阈值

    ros::Time t0 = ros::Time::now();

    do
    {
        min_cost_ = std::numeric_limits<double>::max();
        iter_num_ = 0;

        // 初始化优化变量：展平为 double[] 数组
        double q[variable_num_];
        memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));

        // 调用 L-BFGS 优化器
        // costFunctionRebound：代价函数回调（就是 combineCostRebound）
        // earlyExit：早停检查回调
        int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost,
                                            BsplineOptimizer::costFunctionRebound,
                                            BsplineOptimizer::earlyExit,
                                            this, &lbfgs_params);

        // 优化收敛或达到最大迭代 → 检查碰撞
        if (result == lbfgs::LBFGS_CONVERGENCE ||
            result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
            result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
            result == lbfgs::LBFGS_STOP)
        {
            // 阶段1：多机太近 → 重启优化
            if (min_ellip_dist_ > swarm_clearance_)
            {
                restart_nums++;
                initControlPoints(cps_.points, false);
                new_lambda2_ *= 2;  // 增大避障权重
                continue;
            }

            // 阶段2：检查轨迹碰撞（采样检查前 2/3 轨迹）
            UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
            for (double t = tm; t < tmp * 2 / 3; t += t_step)
            {
                if (grid_map_->getInflateOccupancy(traj.evaluateDeBoorT(t)))
                {
                    // 发现碰撞：重启优化
                    restart_nums++;
                    initControlPoints(cps_.points, false);
                    new_lambda2_ *= 2;
                    break;
                }
            }

            if (!flag_occ)  // 无碰撞 → 成功
                success = true;
        }
        else if (result == lbfgs::LBFGSERR_CANCELED)
        {
            // L-BFGS 被 earlyExit 中断（碰撞反弹触发）
            // 继续迭代，最多 20 次 rebound
            rebound_times++;
        }

    } while ((碰撞或 swarm 过近 且 重启次数<3) 或
             (被中断且 rebound次数<=20));

    return success;
}
```

---

## check_collision_and_rebound —— 碰撞反弹机制（L1198-1393）

### 核心思想

L-BFGS 迭代后期，如果轨迹仍不够平滑，就触发碰撞检查：
1. 扫描控制点，发现哪个控制点撞障碍物
2. 用 A* 搜索绕障碍物的路径
3. 将 A* 路径的交点作为新的 `base_point` 和 `direction`
4. 更新后重新迭代优化

```cpp
bool BsplineOptimizer::check_collision_and_rebound(void)
{
    // 1. 扫描控制点，找出与障碍物碰撞的区间
    // 对每个控制点：用 getInflateOccupancy() 检查
    // 如果控制点 i 在障碍物中，向前找到第一个安全点 in_id，向后找到第一个安全点 out_id
    // → 得到一段 [in_id, out_id] 的碰撞区间

    if (flag_new_obs_valid)  // 发现新的碰撞
    {
        // 2. 对每个碰撞区间，用 A* 搜索绕障路径
        vector<vector<Eigen::Vector3d>> a_star_pathes;
        for (size_t i = 0; i < segment_ids.size(); ++i)
        {
            Eigen::Vector3d in(cps_.points.col(segment_ids[i].first));
            Eigen::Vector3d out(cps_.points.col(segment_ids[i].second));
            if (a_star_->AstarSearch(0.1, in, out))  // 0.1 = 网格分辨率
            {
                a_star_pathes.push_back(a_star_->getPath());
            }
        }

        // 3. 将 A* 路径与原始 B-spline 控制线求交
        // 得到一系列交点（intersection_point）
        // → 作为新的碰撞约束

        // 4. 设置新的 base_point 和 direction
        // direction = 从控制点到交点的方向（推离障碍物）
        // base_point = 交点

        force_stop_type_ = STOP_FOR_REBOUND;  // 触发 L-BFGS 重新迭代
        return true;
    }

    return false;
}
```

---

## 完整优化流程总结

```
reboundReplan()
    │
    ├─ STEP 1: 生成初始控制点（多项式 or 当前轨迹延伸）
    │
    ├─ STEP 2: 调用 BsplineOptimizeTrajRebound()
    │      │
    │      ├─ rebound_optimize()
    │      │     │
    │      │     ├─ L-BFGS 迭代 (max 200 次)
    │      │     │     ├─ costFunctionRebound = combineCostRebound()
    │      │     │     │     ├─ calcSmoothnessCost (λ1)
    │      │     │     │     ├─ calcDistanceCostRebound (λ2)
    │      │     │     │     ├─ calcFeasibilityCost (λ3)
    │      │     │     │     ├─ calcSwarmCost (λ2)
    │      │     │     │     └─ calcTerminalCost (λ2)
    │      │     │     │
    │      │     │     └─ earlyExit 检查
    │      │     │            ├─ 碰撞 → restart（最多3次）
    │      │     │            ├─ swarm 太近 → restart
    │      │     │            └─ 收敛 → 碰撞验证
    │      │     │
    │      │     └─ collision check（三阶段验证）
    │      │
    │      └─ 返回优化成功/失败
    │
    └─ STEP 3: refineTrajAlgo()（单机模式）
           ├─ 时间重分配（如果 velocity/acceleration 超限）
           └─ BsplineOptimizeTrajRefine()
                  └─ combineCostRefine()
                         ├─ calcSmoothnessCost (λ1)
                         ├─ calcFitnessCost (λ4)
                         └─ calcFeasibilityCost (λ3)
```
