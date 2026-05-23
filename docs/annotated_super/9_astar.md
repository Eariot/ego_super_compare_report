# astar.cpp —— SUPER 前端路径搜索

## 文件路径
- `super_planner/src/super_core/astar.cpp`
- `super_planner/include/path_search/astar.h`、`config.hpp`

## 核心理解

SUPER 的 A* 不是普通的栅格 A*，它有几个特化设计：

1. **双视图搜索**：可在 `inf_map`（膨胀图，分辨率粗、查询快）或 `prob_map`（精细图）上跑，由调用者选择 `flag = ON_INF_MAP | ON_PROB_MAP`；
2. **未知格处理**：可选 `UNKNOWN_AS_OCCUPIED`（前端只走已知自由）或 `UNKNOWN_AS_FREE`（允许未知，必须 backup_traj 兜底）；
3. **Escape Search**：起点可能落在膨胀的占据格中，需要先逃逸到自由格；
4. **球形邻域 + 精细膨胀**：通过 `setFineInfNeighbors(neighbor_step)` 配置以 robot_r 为半径的球形邻域，搜索时所有邻居都得"球形线段无碰"；
5. **预排序的扩展邻居**：构造时一次性生成 `[-100, +100]³` 的全部偏移，按距离排序，搜索时按需取（避免每次重新算邻居）。

> 与 EGO 的 `dyn_a_star.cpp` 对比：EGO 的 A* 仅在 collision rebound 时局部使用，单次搜索范围很小（绕障）；SUPER 的 A* 是规划主路径的前端，搜索范围 = 整个 planning_horizon。

---

## 类成员（astar.h）

```cpp
class Astar {
private:
    PathSearchConfig cfg_;                    // 静态配置（地图大小、tie_breaker、heu_type 等）
    rog_map::ROGMapROS::Ptr map_ptr_;
    ros_interface::RosInterface::Ptr ros_ptr_;

    // 节点缓冲：预分配整个搜索范围内每个栅格一个 GridNode
    // rounds_ 计数：每次搜索 ++，节点的 rounds 不等于 rounds_ → 视为未访问
    vector<GridNodePtr> grid_node_buffer_;
    int rounds_{0};

    vector<rog_map::Vec3i> sorted_pts;        // 预排序的 [-100, +100]³ 偏移列表
    vec_Vec3i neighbor_list;                   // 球形邻域（用于 isLineFree 检查）

    /* 每次搜索的运行时数据 */
    struct {
        rog_map::Vec3f start_pt, goal_pt;
        rog_map::Vec3f local_map_center_d, local_map_min_d, local_map_max_d;
        rog_map::Vec3i local_map_center_id_g;
        double resolution, searching_horizon;
        bool use_inf_map, use_prob_map;
        bool unknown_as_occ, unknown_as_free;
        bool use_inf_neighbor;
        double mission_rcv_WT;
    } md_;
};
```

### 节点结构

```cpp
struct GridNode {
    rog_map::Vec3i id_g;       // 全局栅格索引
    double g_score, f_score;
    GridNode* father_ptr;
    int rounds;                // 与 rounds_ 比较判断是否访问过
    enum NodeState { OPEN_LIST, CLOSED_LIST } state;
    multimap<double, GridNodePtr>::iterator open_set_iter;
};
```

---

## 构造函数（astar.cpp L33-61）

```cpp
Astar::Astar(const std::string &cfg_path,
              const ros_interface::RosInterface::Ptr &ros_ptr,
              rog_map::ROGMapROS::Ptr rm)
    : ros_ptr_(ros_ptr), map_ptr_(rm) {
    cfg_ = PathSearchConfig(cfg_path);

    // 1) 预分配整个搜索区域的 GridNode
    int map_buffer_size = cfg_.map_voxel_num(0)
                         * cfg_.map_voxel_num(1)
                         * cfg_.map_voxel_num(2);
    grid_node_buffer_.resize(map_buffer_size);
    for (auto &i: grid_node_buffer_) {
        i = new GridNode;
        i->rounds = 0;       // 初始 0，不等于 rounds_=1 时被视为未访问
    }

    // 2) 预生成 [-100, +100]³ 的所有偏移，按欧氏距离排序
    int test_num = 100;
    for (int i = -test_num; i <= test_num; i++) {
        for (int j = -test_num; j <= test_num; j++) {
            for (int k = -test_num; k <= test_num; k++) {
                sorted_pts.push_back(rog_map::Vec3i(i, j, k));
            }
        }
    }
    sort(sorted_pts.begin(), sorted_pts.end(),
         [](const rog_map::Vec3i &pt1, const rog_map::Vec3i &pt2) {
             return pt1.squaredNorm() < pt2.squaredNorm();
         });
}
```

> `sorted_pts` 用在 `escapePathSearch` 中——按距离从近到远扫描，找到第一个自由格作为逃逸目的地。

---

## 三个启发式（astar.cpp L109-148）

```cpp
double Astar::getHeu(GridNodePtr node1, GridNodePtr node2, int type) const {
    switch (type) {
        case DIAG: {       // 对角线启发（标准的 3D 八连通最优启发）
            // h(n) = √3·diag + √2·side + 1·straight
            double dx = std::abs(node1->id_g(0) - node2->id_g(0));
            double dy = std::abs(node1->id_g(1) - node2->id_g(1));
            double dz = std::abs(node1->id_g(2) - node2->id_g(2));
            int diag = std::min({dx, dy, dz});
            dx -= diag; dy -= diag; dz -= diag;
            // ...
            return tie_breaker_ * h;
        }
        case MANH:        // 曼哈顿启发：dx + dy + dz
            return tie_breaker_ * (dx + dy + dz);
        case EUCL:        // 欧氏启发：(node2 - node1).norm()
            return tie_breaker_ * (node2->id_g - node1->id_g).norm();
    }
}
```

`tie_breaker_` 通常取 1.001，避免大量等代价节点反复出栈。

---

## 主入口 pointToPointPathSearch（L233-380+）

```cpp
RET_CODE Astar::pointToPointPathSearch(const Vec3f &start_pt,
                                        const Vec3f &end_pt,
                                        const int &flag,                    // 模式标志
                                        const double &searching_horizon,
                                        vec_Vec3f &out_path,
                                        const double &time_out = ...) {
    /* (1) 解析 flag 并初始化 md_ */
    RET_CODE setup_ret = setup(start_pt, end_pt, flag, searching_horizon);
    if (setup_ret != SUCCESS) return setup_ret;

    out_path.clear();
    ++rounds_;     // 新一轮搜索 → 自动让所有 GridNode 视为未访问

    /* (2) 起点 / 终点投影到地图内 */
    Vec3f hit_pt;
    Vec3f local_start_pt = start_pt;
    Vec3f local_end_pt   = end_pt;
    bool start_pt_out_local_map = false;

    if (!insideLocalMap(start_pt)) {
        // 起点在地图外 → 沿"start → 地图中心"方向找与地图盒的交点 → +2 格余量 → 投到自由格
        if (rog_map::lineIntersectBox(start_pt, md_.local_map_center_d,
                                       md_.local_map_min_d, md_.local_map_max_d, hit_pt)) {
            local_start_pt = start_pt + (hit_pt - start_pt).normalized()
                             * ((hit_pt - start_pt).norm() + md_.resolution * 2);
            start_pt_out_local_map = true;
            if (!map_ptr_->getNearestInfCellNot(OCCUPIED, local_start_pt, local_start_pt, 3.0))
                return INIT_ERROR;
        }
    }
    if (!insideLocalMap(end_pt)) {
        // 终点在地图外 → 沿"end → start (或地图中心)"方向找交点
        // ...同样处理
    }

    /* (3) 标准 A* 主循环（用 multimap 当 priority queue） */
    multimap<double, GridNodePtr> open_set;
    GridNodePtr start_node = grid_node_buffer_[hash_id_of(start_idx)];
    start_node->id_g = start_idx;
    start_node->g_score = 0;
    start_node->f_score = getHeu(start_node, end_node);
    start_node->father_ptr = nullptr;
    start_node->rounds = rounds_;
    start_node->state = OPEN_LIST;
    open_set.insert({start_node->f_score, start_node});

    while (!open_set.empty()) {
        // 时间预算检查
        if (ros_ptr_->getSimTime() - md_.mission_rcv_WT > time_out) {
            return TIME_OUT;
        }

        // 取出 f 最小节点
        auto it = open_set.begin();
        GridNodePtr current = it->second;
        open_set.erase(it);
        current->state = CLOSED_LIST;

        // 到达搜索范围 → REACH_HORIZON
        if (md_.searching_horizon > 0 &&
            (current->id_g - start_idx).cwiseAbs().array().any() > horizon_voxel) {
            // 沿 closed list 回溯
            retrievePath(current, node_path);
            ConvertNodePathToPointPath(node_path, out_path);
            return REACH_HORIZON;
        }
        // 到达终点 → REACH_GOAL
        if (current->id_g == end_idx) {
            retrievePath(current, node_path);
            ConvertNodePathToPointPath(node_path, out_path);
            return REACH_GOAL;
        }

        /* (4) 扩展邻居：26 邻居 */
        for (int dx = -1; dx <= 1; dx++)
        for (int dy = -1; dy <= 1; dy++)
        for (int dz = -1; dz <= 1; dz++) {
            if (dx == 0 && dy == 0 && dz == 0) continue;
            Vec3i neighbor_id = current->id_g + Vec3i(dx, dy, dz);

            // 4a) 边界检查
            if (!insideLocalMap(neighbor_id)) continue;

            // 4b) 占据检查（根据 use_inf_map / use_prob_map 选不同 grid）
            //     如果 unknown_as_occ → unknown 视为障碍
            GridType type = md_.use_inf_map ?
                              map_ptr_->getInfGridType(neighbor_id) :
                              map_ptr_->getGridType(neighbor_id);
            if (type == OCCUPIED || type == OUT_OF_MAP) continue;
            if (md_.unknown_as_occ && type == UNKNOWN) continue;

            // 4c) 球形邻域线段检查（仅 use_inf_neighbor 时）
            //     确保以 robot_r 为半径的球扫过这条边时不撞障
            if (md_.use_inf_neighbor) {
                bool collide = false;
                for (auto &nei : neighbor_list) {
                    if (map_ptr_->isOccupiedInflate(neighbor_id + nei)) {
                        collide = true; break;
                    }
                }
                if (collide) continue;
            }

            // 4d) 计算 g、f
            int hash = hash_id_of(neighbor_id);
            GridNodePtr neighbor = grid_node_buffer_[hash];
            double tentative_g = current->g_score + Vec3f(dx,dy,dz).norm();

            if (neighbor->rounds != rounds_) {
                // 第一次访问
                neighbor->id_g = neighbor_id;
                neighbor->rounds = rounds_;
                neighbor->state = OPEN_LIST;
                neighbor->g_score = tentative_g;
                neighbor->f_score = tentative_g + getHeu(neighbor, end_node);
                neighbor->father_ptr = current;
                neighbor->open_set_iter = open_set.insert({neighbor->f_score, neighbor});
            } else if (neighbor->state == OPEN_LIST && tentative_g < neighbor->g_score) {
                // 已在 open list，但找到更短路径 → 更新
                neighbor->g_score = tentative_g;
                neighbor->f_score = tentative_g + getHeu(neighbor, end_node);
                neighbor->father_ptr = current;
                open_set.erase(neighbor->open_set_iter);
                neighbor->open_set_iter = open_set.insert({neighbor->f_score, neighbor});
            }
            // CLOSED_LIST 中的节点不更新（因为启发式 admissible，不会再有更优）
        }
    }

    return NO_PATH;
}
```

---

## escapePathSearch —— 起点逃逸搜索

`super_planner.cpp::PathSearch` 调用 A* 之前，先调一次 `escapePathSearch`：

```cpp
RET_CODE Astar::escapePathSearch(const Vec3f &start_pt, int flag, vec_Vec3f &out_path);
```

意图：起点可能在膨胀的占据格上（如刚起飞，机体半径覆盖几个 inflated 格）。如果直接跑 A* 会立刻"无邻居可扩"。`escapePathSearch` 用 `sorted_pts`（按距离从近到远扫）找到最近的非占据格，从起点拉一条直线作为逃逸路径，主搜索从逃逸终点开始。

返回码：
- `NO_NEED`：起点本身就是非占据 → 不需要逃逸；
- `REACH_HORIZON` / `REACH_GOAL`：找到逃逸点；
- 其他：失败。

---

## flag 编码（path_search/config.hpp）

```cpp
#define ON_INF_MAP            (1 << 0)   // 在膨胀地图上搜
#define ON_PROB_MAP           (1 << 1)   // 在原图上搜
#define UNKNOWN_AS_OCCUPIED   (1 << 2)   // unknown 当障碍
#define UNKNOWN_AS_FREE       (1 << 3)   // unknown 当自由
#define USE_INF_NEIGHBOR      (1 << 4)   // 启用球形邻域检查
#define DONT_USE_INF_NEIGHBOR (1 << 5)   // 强制禁用（覆盖 USE_INF_NEIGHBOR）
```

`super_planner.cpp::PathSearch`（L1107-1201）使用流程：

```cpp
// 1) 起点逃逸（在 prob_map 上搜，unknown 取决于配置）
int flag_es = ON_PROB_MAP | (cfg_.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE);
RET_CODE ret_es = astar_ptr_->escapePathSearch(start_pt, flag_es, out_path);

// 2) 主搜索：先在 inf_map 上搜（更快）
int flag = ON_INF_MAP | (cfg_.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE)
            | DONT_USE_INF_NEIGHBOR;
RET_CODE ret_code = astar_ptr_->pointToPointPathSearch(temp_start_point, goal, flag,
                                                        temp_plannning_horizon, path);

// 3) inf_map 失败 → 回退 prob_map（更精确，但更慢）
if (ret_code == NO_PATH) {
    flag = ON_PROB_MAP | (cfg_.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE)
            | USE_INF_NEIGHBOR;     // prob_map 上需要球形邻域检查保证安全
    ret_code = astar_ptr_->pointToPointPathSearch(temp_start_point, goal, flag,
                                                    temp_plannning_horizon, path);
}
```

> `frontend_in_known_free` 是非常重要的开关：true 时前端只走已知自由空间（保守，依赖 backup_traj）；false 时允许走 unknown 空间（激进，需要更细粒度的安全检查）。SUPER 论文中两种模式下都做了基线对比，"未知-as-free + backup" 比"未知-as-occupied"在 8m/s 高速场景下成功率高 20%+。

---

## 与 EGO `dyn_a_star.cpp` 的对比

| 项目 | EGO dyn_a_star | SUPER astar |
|---|---|---|
| 调用频率 | 仅在 `check_collision_and_rebound` 中按需 | 每次 `generateExpTraj` 一次 |
| 搜索范围 | 局部小段（2-3 m，绕一个障碍物） | 整个 planning_horizon（10-20 m） |
| 地图选择 | 固定（grid_map） | 动态：inf_map / prob_map / 失败回退 |
| 邻域 | 26 邻居（无球形检查） | 26 邻居 + 球形邻域线检查（保证机体半径安全） |
| Unknown 处理 | 视为可通行 | 由 flag 决定，可两种模式 |
| 起点逃逸 | 无 | escapePathSearch 显式逃逸 |
| 时间预算 | 无超时 | `time_out` 参数控制 |
| 启发式 | 欧氏距离 | DIAG / MANH / EUCL 三选一 |
