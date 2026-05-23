# MINCO 轨迹优化 —— ExpTrajOpt + BackupTrajOpt 代价函数

## 文件路径
- `super_planner/src/traj_opt/exp_traj_optimizer_s4.cpp`（1034 行）
- `super_planner/src/traj_opt/backup_traj_optimizer_s4.cpp`（781 行）
- `super_planner/include/traj_opt/exp_traj_optimizer_s4.h`、`backup_traj_optimizer_s4.h`、`minco.h`
- `super_planner/include/utils/optimization/minco.h`、`utils/geometry/quadrotor_flatness.hpp`、`utils/optimization/lbfgs.h`

## 核心理解

SUPER 的轨迹优化建立在三个数学工具上：

1. **MINCO（Minimum Control Polynomial）** —— Wang et al. RA-L 2022 提出的多项式参数化方法。给定每段时间 `T` 和段间路点 `q`，能在闭式时间内得到 7 阶多项式系数，并且方便求解 cost 对 `T`/`q` 的偏导。
2. **微分平坦（quadrotor flatness）** —— 把机体姿态、推力等动力学量从位置轨迹的高阶导数（速度/加速度/jerk）映射出来。这样"姿态约束"和"推力约束"也能写成轨迹位置的函数。
3. **smoothL1 软约束** —— 把"v² ≤ vmax²"等不等式约束转换成可微的 `smoothL1(v² - vmax²)` 罚函数，在 L-BFGS 中作为 cost 的一部分梯度下降。

最终优化变量：
- `tau` —— 段时间的"反正切空间"（保证 T > 0）；
- `xi` —— 段间路点（针对 SFC 用 vPolytope 顶点参数化，保证点在多面体内）；
- BackupTrajOpt 还多了 `ts` —— 接管时刻。

求解器：L-BFGS（`utils/optimization/lbfgs.h`），共享 EGO 的同款库（`Lewis-Overton` line search 版本）。

---

## ExpTrajOpt 类结构

### 关键 API（exp_traj_optimizer_s4.h L319-345）

```cpp
class ExpTrajOpt {
public:
    bool optimize(const StatePVAJ &headPVAJ,        // 起点状态（4×3：P/V/A/J）
                  const StatePVAJ &tailPVAJ,         // 终点状态
                  const vec_Vec3f &guide_path,        // 引导路径（用作 attractor）
                  const vector<double> &guide_stamp,  // 引导路径时间戳
                  PolytopeVec &sfc,                   // 凸多面体走廊（位置硬约束）
                  Trajectory &out_traj);              // 输出 MINCO 轨迹
    void getInitValue(VecDf &ts, vec_Vec3f &ps) const { ts = init_t_; ps = init_ps_; }
};
```

### OptimizationVariables 结构

由 `defaultInitialization`（L129）和 `setupProblemAndCheck`（L131）填充，主要字段：

```cpp
struct OptimizationVariables {
    int temporalDim;            // 段数 = 时间变量数
    int spatialDim;              // 路点变量数（每个路点 3 个分量；用 vPoly 时是 N-1 维）
    double rho;                  // 时间惩罚权重 ρ_T
    PolyhedraH hPolytopes;       // SFC（H-表示）
    PolyhedraV vPolytopes;       // SFC 顶点（V-表示，用于参数化）
    PolyhedraH hOverlapPolytopes;
    VecDi vPolyIdx, hPolyIdx;    // 每段映射到哪个 SFC

    Mat3Df waypoint_attractor;       // 每个 SFC 重叠区的"吸引点"
    VecDf waypoint_attractor_dead_d; // 吸引点的"死区"半径

    double smooth_eps;
    int integral_res;            // 积分分辨率（每段采样多少点做约束积分）
    VecDf magnitudeBounds;       // [vmax, amax, jmax, omgmax, accthrmin, accthrmax]
    VecDf penaltyWeights;        // [pos, vel, acc, jer, attract, omg, thr]
    bool block_energy_cost;
    int pos_constraint_type;     // 0: vPoly 参数化, 1: 自由路点
    flatness::FlatnessMap quadrotor_flatness;

    minco::MINCO_S4NU minco;     // 7 阶 MINCO（位置 8 个系数）
    Mat3Df points;                // 段间路点（3 × N-1）
    VecDf times;                  // 段时长
    VecDf penalty_log;
    int iter_num;
};
```

---

## costFunctional —— L-BFGS 主回调（exp_traj_optimizer_s4.cpp L251-344）

```cpp
double ExpTrajOpt::costFunctional(void *ptr, const VecDf &x, VecDf &g) {
    OptimizationVariables &obj = *static_cast<OptimizationVariables *>(ptr);

    // 1) 把扁平化变量 x 拆成 tau (时间) 和 xi (空间)
    Eigen::Map<const VecDf> tau(x.data(), obj.temporalDim);
    Eigen::Map<const VecDf> xi(x.data() + obj.temporalDim, obj.spatialDim);
    Eigen::Map<VecDf> gradTau(g.data(), obj.temporalDim);
    Eigen::Map<VecDf> gradXi(g.data() + obj.temporalDim, obj.spatialDim);

    // 2) 还原物理量
    Mat3Df points;
    VecDf times;
    gcopter::forwardMapTauToT(tau, times);          // tau → T（用 e^x + 1 之类保证正）
    if (obj.pos_constraint_type == 1) {
        // 自由路点
        VecDf xi_e = xi;
        points = Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic>>(xi_e.data(), 3, xi_e.size() / 3);
    } else {
        // vPoly 参数化：xi → 用 SFC 顶点的凸组合参数化路点
        gcopter::forwardP(xi, obj.vPolyIdx, obj.vPolytopes, points);
    }

    // 3) 能量代价（min control = 最小化 jerk² 积分）
    double cost{0};
    obj.minco.setParameters(points, times);
    MatD3f partialGradByCoeffs(8 * times.size(), 3);
    VecDf partialGradByTimes(times.size());
    if (!obj.block_energy_cost) {
        obj.minco.getEnergy(cost);                                  // 闭式能量
        obj.minco.getEnergyPartialGradByCoeffs(partialGradByCoeffs);
        obj.minco.getEnergyPartialGradByTimes(partialGradByTimes);
    }
    obj.penalty_log(0) = cost;

    // 4) 约束代价（位置/速度/加速度/jerk/姿态/推力 + attractor）
    constraintsFunctional(times, obj.minco.getCoeffs(),
                           obj.hPolyIdx, obj.hPolytopes,
                           obj.waypoint_attractor, obj.waypoint_attractor_dead_d,
                           obj.smooth_eps, obj.integral_res,
                           obj.magnitudeBounds, obj.penaltyWeights,
                           obj.quadrotor_flatness,
                           cost, partialGradByTimes, partialGradByCoeffs,
                           obj.penalty_log);

    // 5) 把"对系数和段时长的梯度"反传到"路点和段时长"
    Mat3Df gradByPoints;
    VecDf gradByTimes;
    obj.minco.propogateGrad(partialGradByCoeffs, partialGradByTimes,
                             gradByPoints, gradByTimes);
    cost += obj.rho * times.sum();      // 时间总长惩罚（ρ_T·ΣT）
    gradByTimes.array() += obj.rho;

    // 6) 反传到 tau / xi 优化变量
    gcopter::propagateGradientTToTau(tau, gradByTimes, gradTau);
    if (obj.pos_constraint_type == 1) {
        MatDf gp = gradByPoints;
        gradXi = Eigen::Map<VecDf>(gp.data(), gp.size());
    } else {
        gcopter::backwardGradP(xi, obj.vPolyIdx, obj.vPolytopes, gradByPoints, gradXi);
        gcopter::normRetrictionLayer(xi, obj.vPolyIdx, obj.vPolytopes, cost, gradXi);
    }
    return cost;
}
```

---

## constraintsFunctional —— 7 类约束代价（exp_traj_optimizer_s4.cpp L45-244）

整体结构：双重循环遍历所有段、所有积分采样点；在每个采样点求一次 7 类 violation 与梯度。

```cpp
for (int i = 0; i < piece_num; i++) {
    const Mat83f &c = coeffs.block<8, 3>(i * 8, 0);    // 第 i 段 8×3 系数矩阵
    const auto &step = T(i) * integralFrac;             // 该段积分步长

    for (int j = 0; j <= integralResolution; j++) {     // 段内 integralRes 采样点
        // 计算 8 个 beta 基（位置/速度/加速度/jerk/snap）
        double s1 = j * step, s2 = s1*s1, ..., s7 = s4*s3;
        Vec8f beta0,...,beta4;
        beta0 << 1,s1,s2,s3,s4,s5,s6,s7;
        beta1 << 0,1,2*s1,3*s2,4*s3,5*s4,6*s5,7*s6;
        beta2 << 0,0,2,6*s1,12*s2,20*s3,30*s4,42*s5;
        beta3 << 0,0,0,6,24*s1,60*s2,120*s3,210*s4;
        beta4 << 0,0,0,0,24,120*s1,360*s2,840*s3;

        const Vec3f pos = c.transpose() * beta0;
        const Vec3f vel = c.transpose() * beta1;
        const Vec3f acc = c.transpose() * beta2;
        const Vec3f jer = c.transpose() * beta3;
        const Vec3f sna = c.transpose() * beta4;

        Vec3f gradPos, gradVel, gradAcc, gradJer; gradPos.setZero(); ...
        double tmp_cost = 0.0;

        /* (1) 位置约束：每个采样点必须落在 hPolys[L] 这个 SFC 内 */
        const auto &L = hIdx(i);
        const auto &K = hPolys[L].rows();
        for (int k = 0; k < K && weightPos > 0; k++) {
            const Vec3f outerNormal = hPolys[L].block<1, 3>(k, 0);
            // violaPos > 0 表示越出该面（外法线方向）
            const double violaPos = outerNormal.dot(pos) + hPolys[L](k, 3);
            double violaPosPena, violaPosPenaD;
            if (gcopter::smoothedL1(violaPos, smoothFactor, violaPosPena, violaPosPenaD)) {
                gradPos += weightPos * violaPosPenaD * outerNormal;
                tmp_cost += weightPos * violaPosPena;
            }
        }

        /* (2) 路点吸引：第 i 段开头/末尾必须在第 i 个 attractor 附近 */
        if (weightAtt > 0.0) {
            const bool is_waypoint = (j == 0) && (i != 0);
            const bool is_end       = ((j == integralResolution) && (i != piece_num - 1));
            const auto idx = is_end ? i : i - 1;
            if (is_waypoint || is_end) {
                Vec3f p_a = pos - waypoint_attractor.col(idx);
                const auto &violaAtt = p_a.squaredNorm() - dead_d² ;
                double violaAttPena, violaAttPenaD;
                if (gcopter::smoothedL1(violaAtt, smoothFactor, violaAttPena, violaAttPenaD)) {
                    gradPos += weightAtt * violaAttPenaD * 2.0 * p_a;
                    tmp_cost += weightAtt * violaAttPena;
                }
            }
        }

        /* (3) 速度约束：|v|² ≤ vmax² */
        const double violaVel = vel.squaredNorm() - vmaxSqr;
        if (weightVel > 0 && gcopter::smoothedL1(violaVel, smoothFactor, violaVelPena, violaVelPenaD)) {
            gradVel += weightVel * violaVelPenaD * 2.0 * vel;
            tmp_cost += weightVel * violaVelPena;
        }

        /* (4) 加速度约束：|a|² ≤ amax² —— 同结构 */
        /* (5) jerk 约束：|j|² ≤ jmax² —— 同结构 */

        /* (6+7) 姿态/推力约束（quadrotor flatness 反推）*/
        Vec3f totalGradPos, totalGradVel, totalGradAcc, totalGradJer;
        if (weightOmg > 0 && weightAccThr > 0) {
            double thr; Vec4f quat; Vec3f omg;
            // flatness::forward：(v, a, j, ψ, ψ̇) → (推力 thr, 四元数 quat, 角速度 omg)
            flatMap.forward(vel, acc, jer, 0.0, 0.0, thr, quat, omg);

            // (6) 角速度约束：|ω|² ≤ ωmax²
            const double violaOmg = omg.squaredNorm() - omgmaxSqr;
            if (weightOmg > 0 && gcopter::smoothedL1(violaOmg, smoothFactor, ...)) {
                gradOmg += weightOmg * violaOmgPenaD * 2.0 * omg;
                tmp_cost += weightOmg * violaOmgPena;
            }

            // (7) 推力约束：(thr - thrustMean)² ≤ thrustRadi²
            const double violaThrust = (thr - thrustMean) * (thr - thrustMean) - thrustSqrRadi;
            if (weightAccThr > 0 && gcopter::smoothedL1(violaThrust, smoothFactor, ...)) {
                gradThr += weightAccThr * violaThrustPenaD * 2.0 * (thr - thrustMean);
                tmp_cost += weightAccThr * violaThrustPena;
            }

            // 反传 ω 和 thr 的梯度到 (vel, acc, jer)
            flatMap.backward(gradPos, gradVel, gradAcc, gradJer, gradThr,
                             Vec4f(0,0,0,0), gradOmg,
                             totalGradPos, totalGradVel, totalGradAcc, totalGradJer,
                             totalGradPsi, totalGradPsiD);
        } else {
            totalGradPos = gradPos;
            totalGradVel = gradVel;
            totalGradAcc = gradAcc;
            totalGradJer = gradJer;
        }

        /* 把"对 (pos, vel, acc, jer) 的梯度"积分到段系数上 */
        const auto node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;
        const double alpha = j * integralFrac;
        gradC.block<8, 3>(i * 8, 0) +=
                (beta0 * totalGradPos.transpose() +
                 beta1 * totalGradVel.transpose() +
                 beta2 * totalGradAcc.transpose() +
                 beta3 * totalGradJer.transpose()) * node * step;
        gradT(i) += (totalGradPos.dot(vel) + totalGradVel.dot(acc) +
                     totalGradAcc.dot(jer) + totalGradJer.dot(sna)) * alpha * node * step
                    + node * integralFrac * tmp_cost;
        cost += node * step * tmp_cost;
    }
}
```

### 7 个代价权重（penalty_log 顺序）

| 索引 | 名字 | 含义 | 默认值（YAML） |
|---|---|---|---|
| 0 | energy | 最小 control（jerk²/snap²/...）能量 | （隐式由 ρ_T、积分代替） |
| POS=1 | pos | 落在 SFC 内 | 1e6 |
| VEL=2 | vel | |v| ≤ vmax | 1e3 |
| ACC=3 | acc | |a| ≤ amax | 1e3 |
| JER=4 | jer | |j| ≤ jmax | 1e3 |
| ATT=5 | attract | 路点吸引（与 EGO 的 fitness cost 类似） | 1e2 |
| OMG=6 | omg | |ω| ≤ ωmax | 1e3 |
| THR=7 | thr | 推力箱约束 | 1e3 |

> 注：实际 YAML 文件里 `penna_pos / penna_vel / ...` 还会在不同剧情下调整。SUPER 论文报告的"高速 8m/s"配置下 penna_pos = 1e6，远高于其他权重，强制保证轨迹必在 SFC 内。

### smoothedL1 函数（gcopter 命名空间）

```cpp
// 把不等式约束 violation 转成可微罚函数：
// f(x) = 0,                                if x ≤ 0
//      = (x³)/(3·μ),                        if 0 < x ≤ μ
//      = x - 2/3·μ,                         if x > μ
// f'(x) 按段对应 0 / x²/μ / 1
inline bool smoothedL1(double v, double mu, double &pena, double &grad);
```

---

## BackupTrajOpt 与 ExpTrajOpt 的差异

| 项目 | ExpTrajOpt | BackupTrajOpt |
|---|---|---|
| 轨迹段数 | 多段（与 SFC 数对应） | 通常 2-3 段（短刹车轨迹） |
| SFC 数量 | 多个，路点参数化用相邻 SFC 重叠区 | **1 个**（直接传 PolyhedronH） |
| 优化变量 | tau + xi | tau + xi + **ts**（接管时刻） |
| 终点约束 | 自由（落在最后一个 SFC 内） | 必须接管 exp 轨迹（强约束） |
| Attractor | 路点 attractor（每个 SFC 重叠区的中心） | exp 轨迹的位置（沿时间采样追踪） |
| 是否启用 ω/thr 约束 | 是（高速场景） | 是（刹车阶段更需要） |

具体差异看 `backup_traj_optimizer_s4.cpp` 的 `constraintsFunctional`（L32+）：

- 没有 `weightAtt`（路点 attractor）这一项；
- 多了 `BackupAttractToExp`（追踪 exp 位置，让 backup 与 exp 平滑接管）；
- 优化变量多一维 `ts`，对应"接管时刻"的梯度由"追踪误差对 ts 求导"得到。

---

## MINCO_S4NU 数据结构（`utils/optimization/minco.h`）

```cpp
class MINCO_S4NU {
    // s = 4：snap-minimum + 自由首尾末状态
    // 每段轨迹：8 个系数 × 3 维 = 24 个未知数
    // 给定段时长 T、首末状态（4×3）、段间路点 q，闭式求出系数矩阵 c
public:
    void setParameters(const Eigen::Matrix3Xd &q, const Eigen::VectorXd &T);
    void getEnergy(double &energy);                        // ∫|jerk|²dt 闭式
    void getTrajectory(Trajectory &traj);
    void getEnergyPartialGradByCoeffs(...);
    void getEnergyPartialGradByTimes(...);
    void propogateGrad(const MatD3f &gradByCoeffs, const VecDf &gradByTimes,
                        Mat3Df &gradByPoints, VecDf &gradByTimes_out);
};
```

> Banded system 解法：MINCO 的 P/V/A/J 在段间连续 → 系数矩阵 c 由首末状态 + 路点 q + 段时长 T 唯一确定。这构成一个 8N×8N 的线性系统，且系数矩阵是带状的，可在 O(N) 时间求解（`utils/optimization/banded_system.cpp`）。

---

## 与 EGO B-spline 优化的对比

| 项目 | EGO B-spline | SUPER MINCO |
|---|---|---|
| 多项式阶数 | 三次（cubic）均匀 B-spline | 7 阶 minimum-snap（s=4） |
| 时间是否优化 | 否（仅在 refine 阶段做时间重分配） | 是（每段时长是优化变量） |
| 路点是否优化 | 控制点间接对应 | 直接优化段间路点 |
| 障碍物约束 | 软约束（distance cost + collision rebound） | 硬约束（落在 SFC 内）+ smoothL1 软化 |
| 高阶动力学 | 仅速度/加速度 | 速度/加速度/jerk/姿态/推力 |
| 优化器 | L-BFGS（同款 Lewis-Overton） | L-BFGS（同款） |
| 控制点数 | 30-100 个 | 段路点 5-30 个 |
| 单次优化耗时 | 5-15 ms | 10-40 ms |
| 输出 | B-spline 控制点 + knot vector | 多段多项式系数（SE3 兼容） |

EGO 的 B-spline 优势：表示紧凑、计算快；劣势：阶数低（cubic），速度/加速度满足边界状态要求时缺自由度。
SUPER 的 MINCO 优势：阶数高，能精确施加 jerk/姿态/推力约束，时间也优化；劣势：cost 计算比 B-spline 重，且需要前端给 SFC。
