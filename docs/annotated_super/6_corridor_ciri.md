# corridor_generator.cpp + ciri.cpp —— 凸多面体安全飞行走廊

## 文件路径
- `super_planner/src/super_core/corridor_generator.cpp`（376 行）
- `super_planner/src/super_core/ciri.cpp`（419 行）
- 头文件：`super_planner/include/super_core/corridor_generator.h`、`ciri.h`

## 核心理解

SFC（**S**afe **F**light **C**orridor）是一组凸多面体的并集，每个多面体由若干半空间 `n_k · x + d_k ≤ 0` 围成。MINCO 优化时把"轨迹位置不撞障"这个非凸约束转换成"轨迹位置必须落在指定凸多面体内"——后者是凸约束，可以用 smoothL1 软化后做梯度优化。

```
机体起点 ●────●────●────●────● 目标
        SFC₁  SFC₂  SFC₃  SFC₄  SFC₅
       (重叠 ≥ min_overlap_threshold)
```

每个 SFC 由 **CIRI** 算法（Convex polytope generation by Iterative Region Inflation，对应 LIN/IRIS 系列方法）从一根种子线段（seed line）膨胀出来。

---

## CorridorGenerator 类 —— 多段走廊串联

### 构造函数（corridor_generator.cpp L30-47）

```cpp
CorridorGenerator::CorridorGenerator(
        const ros_interface::RosInterface::Ptr &ros_ptr,
        const rog_map::ROGMapROS::Ptr &map_ptr,
        const double bound_dis,                  // 走廊周围搜索盒子的边界（m）
        const double seed_line_max_dis,           // 单段种子线段最大长度
        const double min_overlap_threshold,        // 相邻两段必须有的最小 overlap depth
        const double virtual_groud_height,        // 虚拟地面（强制 z >= 此值）
        const double virtual_ceil_height,          // 虚拟天花板
        const double robot_r,                      // 机体半径（避障膨胀用）
        const int box_search_skip_num,             // 邻域搜索的稀疏步长
        const int iris_iter_num                    // CIRI 内部迭代次数
) {
    ciri_ = std::make_shared<CIRI>(ros_ptr_);
    ciri_->setupParams(robot_r, iris_iter_num);
    bound_dis_ = bound_dis;
    seed_line_max_length_ = seed_line_max_dis;
    min_overlap_threshold_ = min_overlap_threshold;
    robot_r_ = robot_r;
    box_search_skip_num_ = box_search_skip_num;
    iris_iter_num_ = iris_iter_num;
    virtual_ceil_height_  = virtual_ceil_height - robot_r;
    virtual_groud_height_ = virtual_groud_height + robot_r;
}
```

### SearchPolytopeOnPath —— 把折线切成多段 SFC（L54-195）

这是 `generateExpTraj` 的关键依赖。算法：

1. 从 path[0] 开始，沿折线尽量找最远的 `path[j]`，使得 `[path[first_id], path[j]]` 仍然是 line-free（用 `isLineFree` 检查）；
2. 把 `[first_id, second_id]` 当作种子线段，调 CIRI 膨胀出 `temp_poly`；
3. 检查与上一段 SFC 的 overlap：足够大就接续；不够就用 `path[first_id]` 这一个点作为种子，生成"修复多面体" `temp_poly_fix_p`；
4. 处理"小多面体可省略"的优化（如果 `temp_poly` 与上上段已 overlap，则把上一段 pop_back）；
5. 重复直到走完整条 path。

```cpp
bool CorridorGenerator::SearchPolytopeOnPath(const vec_Vec3f &path, PolytopeVec &sfcs,
                                              Vec3f &shifted_start_pt,
                                              bool cut_first_poly) {
    sfcs.clear();
    if (path.empty()) return false;

    int first_id = 0, second_id;
    /* ---- shift 起点：跳过开头的所有占据点 ---- */
    while (first_id < path.size() && map_ptr_->isOccupiedInflate(path[first_id]))
        first_id++;

    /* ---- 如果首点被 shift 过，先生成一个"逃生多面体"作为 SFC[0] ---- */
    if (first_id != 0) {
        shifted_start_pt = path[first_id];
        double dis = (path[first_id] - path[0]).norm() * 1.2;
        Polytope temp_poly;
        GenerateEmptyPolytope(path[0], dis, temp_poly);   // 简单方盒
        sfcs.emplace_back(temp_poly);
    }

    /* ---- 主循环 ---- */
    int max_loop = 1000, cnt_loop = 0;
    while (cnt_loop++ < max_loop) {
        /* (1) 找最远 line-free 点 */
        second_id = first_id;
        for (int j = first_id + 1; j < path.size(); j++) {
            if (!map_ptr_->isLineFree(path[first_id], path[j],
                                        seed_line_max_length_,
                                        line_seed_neighbor_list)) {
                second_id = j - 1;
                if (second_id - 1 > first_id) second_id -= 1;   // 安全余量
                break;
            }
            second_id = j;
        }
        if (second_id == first_id && second_id + 1 < path.size())
            second_id += 1;

        /* (2) 用种子线段生成多面体 */
        Polytope temp_poly;
        if (!GeneratePolytopeFromLine(Line{path[first_id], path[second_id]}, temp_poly)) {
            return false;
        }

        /* (3) 检查与上一段的 overlap */
        if (!sfcs.empty()) {
            Polytope overlap = sfcs.back().CrossWith(temp_poly);
            Vec3f interior_pt;
            double interior_depth = geometry_utils::findInteriorDist(overlap.GetPlanes(),
                                                                       interior_pt);
            temp_poly.overlap_depth_with_last_one = interior_depth;
            temp_poly.interior_pt_with_last_one    = interior_pt;

            if (interior_depth < min_overlap_threshold_) {
                /* (4) overlap 不够 → 在 first_id 点附近生成修复多面体 */
                Polytope temp_poly_fix_p;
                if (!GeneratePolytopeFromPoint(path[first_id], temp_poly_fix_p))
                    return false;

                Polytope new_overlap = sfcs.back().CrossWith(temp_poly_fix_p);
                interior_depth = geometry_utils::findInteriorDist(new_overlap.GetPlanes(),
                                                                    interior_pt);
                if (interior_depth <= 0.01) return false;        // 修复也无解 → 失败

                temp_poly_fix_p.overlap_depth_with_last_one = interior_depth;
                temp_poly_fix_p.interior_pt_with_last_one    = interior_pt;
                sfcs.push_back(temp_poly_fix_p);

                // 重新检查 [fix_p, temp_poly] 的 overlap
                new_overlap = sfcs.back().CrossWith(temp_poly);
                interior_depth = geometry_utils::findInteriorDist(new_overlap.GetPlanes(),
                                                                    interior_pt);
                if (interior_depth <= 0.01) return false;
            }
            else {
                /* (5) overlap 足够：检查能否省掉上一段 */
                int temp_id = sfcs.size() - 2;
                if (temp_id > 0) {
                    Polytope overlap2 = sfcs[temp_id].CrossWith(temp_poly);
                    double interior_depth2 = geometry_utils::findInteriorDist(
                            overlap2.GetPlanes(), interior_pt);
                    if (interior_depth2 > sfcs[temp_id + 1].overlap_depth_with_last_one * 0.25) {
                        // sfcs[temp_id] 与 temp_poly 直接 overlap 也够大 → 省掉 sfcs[temp_id+1]
                        temp_poly.overlap_depth_with_last_one = interior_depth2;
                        temp_poly.interior_pt_with_last_one    = interior_pt;
                        sfcs.pop_back();
                    }
                }
            }
        }

        sfcs.push_back(temp_poly);
        if (second_id == path.size() - 1) break;
        first_id = second_id;
    }

    if (cnt_loop >= max_loop) return false;
    return !sfcs.empty();
}
```

> 这一段的两个剪枝技巧（"修复多面体"和"省略中间多面体"）是 SUPER 走廊比 LIN/IRIS 标准实现紧凑的关键——平均能减少 30%-50% 的多面体数量，对优化器的负担有显著影响。

### GeneratePolytopeFromLine / GeneratePolytopeFromPoint

两者都封装 CIRI：

```cpp
bool CorridorGenerator::GeneratePolytopeFromLine(const Line &line, Polytope &out_poly) {
    /* (1) 取线段周围 bound_dis_ 立方体内的所有占据点 → 障碍物点云 pc */
    Vec3f box_min, box_max;
    getSeedBBox(line.first, line.second, box_min, box_max);
    box_min -= Vec3f::Constant(bound_dis_);
    box_max += Vec3f::Constant(bound_dis_);

    Eigen::Matrix3Xd pc;     // 障碍物点云矩阵
    map_ptr_->boxSearchOccupied(box_min, box_max, box_search_skip_num_, pc);

    /* (2) bd：边界约束（盒子 6 个面 + 虚拟天花板/地面）*/
    Eigen::MatrixX4d bd = generateBoundary(box_min, box_max);

    /* (3) 调 CIRI 主算法 */
    auto ret_code = ciri_->comvexDecomposition(bd, pc, line.first, line.second);
    if (ret_code != SUCCESS) return false;

    out_poly.SetPlanes(ciri_->getPolytope());
    return true;
}
```

`GeneratePolytopeFromPoint` 是 `GeneratePolytopeFromLine` 的退化情况，把 `a == b` 当作"零长度线段"。

---

## CIRI 算法 —— 凸多面体核心

### 算法概述

CIRI 的核心：**用一个椭球 E（Ellipsoid）作为种子，在每次迭代中**：

1. 找椭球外最近的障碍物或边界 → 生成一个切平面（tangent plane）；
2. 用该切平面把椭球往内"切"一刀；
3. 在切完的多面体内重新拟合最大椭球 E_new；
4. 重复，直到没有未处理的障碍物为止；
5. 最后一轮的切平面集合就是输出多面体。

### 入口函数 `comvexDecomposition`（ciri.cpp L32-220）

```cpp
RET_CODE CIRI::comvexDecomposition(const Eigen::MatrixX4d& bd,    // 边界约束（每行 [n.x n.y n.z d]）
                                    const Eigen::Matrix3Xd& pc,   // 障碍物点云
                                    const Eigen::Vector3d& a,     // 种子线起点
                                    const Eigen::Vector3d& b) {   // 种子线终点
    /* (1) 验证 a, b 在 bd 内部 */
    const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);
    const Eigen::Vector4d bh(b(0), b(1), b(2), 1.0);
    if ((bd * ah).maxCoeff() > epsilon_ ||
        (bd * bh).maxCoeff() > epsilon_) {
        return INIT_ERROR;
    }

    const int M = bd.rows();   // 边界约束数
    const int N = pc.cols();   // 障碍物点数

    /* (2) 初始椭球：从 a 到 b 的胶囊形 */
    Ellipsoid E(Mat3f::Identity(), (a + b) / 2);
    if ((a - b).norm() > 0.1) {
        findEllipsoid(pc, a, b, E);   // 从单位球开始拉伸到 [a, b]
    }

    /* (3) 主迭代循环：iter_num 次 */
    vector<Eigen::Vector4d> planes;
    Vec3f infeasible_pt_w;

    for (int loop = 0; loop < iter_num_; ++loop) {
        // 把所有障碍物点和边界约束变换到椭球本地坐标
        const Eigen::Vector3d fwd_a = E.toEllipsoidFrame(a);
        const Eigen::Vector3d fwd_b = E.toEllipsoidFrame(b);
        const Eigen::MatrixX4d bd_e = E.toEllipsoidFrame(bd);
        const Eigen::Matrix3Xd pc_e = E.toEllipsoidFrame(pc);

        // 计算每个障碍物到椭球中心的距离（在本地坐标下 = 范数）
        Eigen::VectorXd distRs = pc_e.colwise().norm();
        // 计算每个边界平面到椭球中心的距离
        Eigen::VectorXd distDs = bd_e.rightCols<1>().cwiseAbs()
                                 .cwiseQuotient(bd_e.leftCols<3>().rowwise().norm());

        Eigen::Matrix<uint8_t, -1, 1> bdFlags = Eigen::Matrix<uint8_t, -1, 1>::Constant(M, 1);
        Eigen::Matrix<uint8_t, -1, 1> pcFlags = Eigen::Matrix<uint8_t, -1, 1>::Constant(N, 1);
        bool completed = false;
        int bdMinId, pcMinId;
        double minSqrD = distDs.minCoeff(&bdMinId);
        double minSqrR = distRs.minCoeff(&pcMinId);

        planes.clear();
        planes.reserve(30);

        /* (4) 内层循环：贪心选取最近约束生成切平面 */
        for (int i = 0; !completed && i < (M + N); ++i) {
            Eigen::Vector4d temp_plane_w;

            if (minSqrD < minSqrR) {
                /* Case A：最近的是边界 → 直接把对应边界平面变换到世界系作为切平面 */
                Vec4f p_e = bd_e.row(bdMinId);
                temp_plane_w = E.toWorldFrame(p_e);
                bdFlags(bdMinId) = 0;
            }
            else {
                /* Case B：最近的是障碍物 → 计算"球到椭球的切平面" */
                const auto &pt_w = pc.col(pcMinId);
                const auto dis = distancePointToSegment(pt_w, a, b);
                if (dis < robot_r_ - 1e-2) {
                    // 不可行：障碍物离种子线太近，无法保留 robot_r 安全距离
                    return FAILED;
                }

                if (robot_r_ < epsilon_) {
                    /* 零半径快速分支：直接做正交投影 */
                    const Vec3f &pt_e = pc_e.col(pcMinId);
                    Eigen::Vector4d temp_tangent;
                    temp_tangent(3) = -distRs(pcMinId);
                    temp_tangent.head(3) = pt_e.transpose() / distRs(pcMinId);

                    /* 修正：如果 fwd_a / fwd_b 落在切平面外侧，重新计算切平面 */
                    if (temp_tangent.head(3).dot(fwd_a) + temp_tangent(3) > epsilon_) {
                        const Eigen::Vector3d delta = pc_e.col(pcMinId) - fwd_a;
                        temp_tangent.head(3) = fwd_a - (delta.dot(fwd_a) / delta.squaredNorm()) * delta;
                        // ... 重新归一化 ...
                    }
                    // 同样处理 fwd_b
                    temp_plane_w = E.toWorldFrame(temp_tangent);
                }
                else {
                    /* 非零半径：构造"机体球"和椭球的切平面 */
                    const Vec3f &pt_e = pc_e.col(pcMinId);
                    const Vec3f &pt_w = pc.col(pcMinId);
                    const Mat3f C_inv = E.C().inverse();
                    Ellipsoid E_pe(C_inv * sphere_template_.C(), pt_e);
                    Vec3f close_pt_e;
                    E_pe.pointDistaceToEllipsoid(Vec3f(0, 0, 0), close_pt_e);
                    Vec3f c_pt_w = E.toWorldFrame(close_pt_e);

                    temp_plane_w.head(3) = (pt_w - c_pt_w).normalized();
                    temp_plane_w(3)      = -temp_plane_w.head(3).dot(c_pt_w);

                    /* 同样修正：保证 a, b 落在内侧 */
                    if (temp_plane_w.head(3).dot(a) + temp_plane_w(3) > -epsilon_) {
                        findTangentPlaneOfSphere(pt_w, robot_r_, a, E.d(), temp_plane_w);
                    } else if (temp_plane_w.head(3).dot(b) + temp_plane_w(3) > -epsilon_) {
                        findTangentPlaneOfSphere(pt_w, robot_r_, b, E.d(), temp_plane_w);
                    }
                }
                pcFlags(pcMinId) = 0;
            }

            /* (5) 标记被该切平面"挡住"的所有约束（不再考虑） */
            completed = true;
            minSqrD = INFINITY;
            for (int j = 0; j < M; ++j) {
                if (bdFlags(j)) {
                    completed = false;
                    if (minSqrD > distDs(j)) { bdMinId = j; minSqrD = distDs(j); }
                }
            }
            minSqrR = INFINITY;
            for (int j = 0; j < N; ++j) {
                if (pcFlags(j)) {
                    if ((temp_plane_w.head(3).dot(pc.col(j)) + temp_plane_w(3)) > robot_r_ - epsilon_) {
                        pcFlags(j) = 0;          // 该障碍物点已被切除
                    } else {
                        completed = false;
                        if (minSqrR > distRs(j)) { pcMinId = j; minSqrR = distRs(j); }
                    }
                }
            }
            planes.push_back(temp_plane_w);
        }

        /* (6) 把 planes 转成 hPoly 矩阵 */
        Eigen::MatrixX4d hPoly;
        hPoly.resize(planes.size(), 4);
        for (size_t i = 0; i < planes.size(); ++i)
            hPoly.row(i) = planes[i];

        /* (7) 最后一轮直接返回；否则重新拟合内接最大椭球，进入下一轮 */
        if (loop == iter_num_ - 1) {
            polytope_ = hPoly;
            return SUCCESS;
        } else {
            findMaximumInscribedEllipsoid(hPoly, E);   // 内接最大椭球（涉及 SDP 求解）
        }
    }

    return SUCCESS;
}
```

### 关键子算法

| 子算法 | 文件位置 | 作用 |
|---|---|---|
| `findEllipsoid` | ciri.cpp 中段 | 用胶囊形（capsule）拟合 a→b 的椭球 |
| `findMaximumInscribedEllipsoid` | ciri.cpp 末段 | 在多面体内拟合最大内接椭球（用 SDP/MVIE） |
| `findTangentPlaneOfSphere` | utils/geometry | 球-球切平面 |
| `Ellipsoid::pointDistaceToEllipsoid` | utils/ellipsoid.cpp | 点到椭球面距离 |

> CIRI 实际上是 IRIS 算法（Iterative Regional Inflation by Semidefinite programming, Liu et al. 2017）的一种实用变体，把"最大椭球"和"切平面"反复迭代。SUPER 默认 `iris_iter_num = 1` —— 单次迭代即可，因为 SUPER 的 SFC 不要求最大体积，够用就好。

---

## 与 EGO 避障方式的对比

| 项目 | EGO（ESDF-Free + Collision Rebound） | SUPER（CIRI 凸多面体） |
|---|---|---|
| 障碍物表示 | 占用栅格 + 在线 A* | 占用栅格 + 离线 SFC |
| 避障约束 | 软约束（cost）：点到障碍物有距离 + 反弹方向 | 硬约束（凸约束）：点必须在多面体内 |
| 优化器视角下 | 非凸（cost 不平滑） | 凸（半空间约束 smoothL1 化） |
| 处理动态障碍物 | 容易（每次迭代实时检查） | 较慢（需要重新生成 SFC） |
| 计算成本 | 取决于 A* 频率（约 1-3 ms） | SFC 一次性生成（5-15 ms），但只在前端做 |
| 优化阶段速度 | 慢（cost 估值要查询 GridMap） | 快（cost 估值是矩阵-向量乘） |
| 走廊数量 | 0（无显式走廊） | 5-30 个多面体/段 |
