# ROG-Map —— 占用栅格 + 膨胀 + ESDF + 滑动窗

## 文件路径
- 头文件：`rog_map/include/rog_map/rog_map.h`、`prob_map.h`、`inf_map.h`、`esdf_map.h`、`free_cnt_map.h`
- 实现：`rog_map/src/rog_map/rog_map.cpp`、`prob_map.cpp`（848 行）、`inf_map.cpp`、`esdf_map.cpp`
- 滑动窗：`rog_map/include/rog_map/rog_map_core/sliding_map.h`
- 射线投射：`rog_map/include/rog_map/rog_map_core/raycaster.h/cpp`
- ROS 适配层：`rog_map/include/rog_map_ros/rog_map_ros1.hpp`、`rog_map_ros2.hpp`

## 核心理解

ROG-Map（**R**eactive **O**ccupancy **G**rid Map）是 SUPER 项目独立可复用的子模块。它把多种"地图视图"统一到一个 `ROGMap` 类下：

```
ROGMap (主类)
   │
   └─ ProbMap                    ← 概率占用栅格（log-odds）
         ├─ occupancy_buffer_     ← 主存储（一维 hash 地址）
         ├─ inf_map_  : InfMap     ← 膨胀地图（避障查询）
         ├─ esdf_map_ : ESDFMap    ← 欧几里得有符号距离场（可选）
         └─ fcnt_map_ : FreeCntMap ← 自由空间计数（前沿提取用）
```

所有子地图都继承 `SlidingMap`：当机体位置离地图原点超过 `map_sliding_thresh` 时，地图整体平移，新边界设为 unknown，旧边界丢弃。这避免了"全局栅格无限增长"问题。

---

## 与 EGO `grid_map` 的关键差异

| 项目 | EGO grid_map | ROG-Map |
|---|---|---|
| 概率更新 | logit 累加（hit/miss） | logit 累加（hit/miss） |
| 膨胀 | 单层（`occupancy_buffer_inflate_`） | 独立 `inf_map_`（继承 SlidingMap，可单独平移） |
| ESDF | **无**（EGO 是 ESDF-Free） | 可选（`esdf_en` 配置） |
| 前沿提取 | 无 | 可选（`frontier_extraction_en`） |
| 局部滑动 | 一次性对全图清边界 | 整体平移（O(平移量)，不是 O(N)） |
| 多机融合 | 不支持（点云独立处理） | 不支持（独立机体） |
| 输入 | 深度图 / 点云 | 点云（`PointCloud`） + Pose |
| 查询 API | `getInflateOccupancy(pos)` | `isOccupied / isOccupiedInflate / isUnknown / isLineFree / getNearestCellNot` |

---

## ROGMap 主类（rog_map.cpp）

### init —— 初始化（L28-78）

```cpp
void ROGMap::init() {
    initProbMap();    // 委托给 ProbMap

    map_info_log_file_.open(DEBUG_FILE_DIR("rm_info_log.csv"), std::ios::out | std::ios::trunc);
    time_log_file_.open(DEBUG_FILE_DIR("rm_performance_log.csv"), std::ios::out | std::ios::trunc);

    robot_state_.p = cfg_.fix_map_origin;

    // 是否启用滑动窗
    if (cfg_.map_sliding_en) {
        mapSliding(Vec3f(0, 0, 0));
        inf_map_->mapSliding(Vec3f(0, 0, 0));
    } else {
        // 不滑动：用 fix_map_origin 锁住地图原点
        local_map_bound_min_d_ = -cfg_.half_map_size_d + cfg_.fix_map_origin;
        local_map_bound_max_d_ =  cfg_.half_map_size_d + cfg_.fix_map_origin;
        mapSliding(cfg_.fix_map_origin);
        inf_map_->mapSliding(cfg_.fix_map_origin);
    }

    // 离线 PCD 加载（用于仿真不带感知，直接给地图）
    if (cfg_.load_pcd_en) {
        PointCloud::Ptr pcd_map(new PointCloud);
        if (pcl::io::loadPCDFile(cfg_.pcd_name, *pcd_map) == -1) exit(-1);
        updateOccPointCloud(*pcd_map);
        if (cfg_.esdf_en) esdf_map_->updateESDF3D(robot_state_.p);
        map_empty_ = false;
    }
}
```

### isLineFree —— 折线碰撞查询（L132-205）

CorridorGenerator 生成 SFC 时反复调用：

```cpp
bool ROGMap::isLineFree(const Vec3f& start_pt, const Vec3f& end_pt,
                         const double& max_dis, const vec_Vec3i& neighbor_list) const {
    raycaster::RayCaster raycaster;
    raycaster.setResolution(cfg_.resolution);
    Vec3f ray_pt;
    raycaster.setInput(start_pt, end_pt);

    while (raycaster.step(ray_pt)) {
        if (max_dis > 0 && (ray_pt - start_pt).norm() > max_dis) return false;

        // 关键：除了检查 ray_pt 本身，还检查 neighbor_list 偏移后的格子
        // neighbor_list 通常是球形邻域（半径 robot_r）
        if (neighbor_list.empty()) {
            if (isOccupied(ray_pt)) return false;
        } else {
            Vec3i ray_pt_id_g;
            posToGlobalIndex(ray_pt, ray_pt_id_g);
            for (const auto& nei : neighbor_list) {
                if (isOccupied(ray_pt_id_g + nei)) return false;
            }
        }
    }
    return true;
}
```

> 邻域检查相当于"线段膨胀"：把 raycast 路径上的每个点都按 `neighbor_list` 检查一遍，等价于沿路径扫描一个球。这是 SUPER 走廊生成时保证机体半径安全的关键。

### updateMap —— 主入口（L240-262）

```cpp
void ROGMap::updateMap(const PointCloud& cloud, const Pose& pose) {
    if (cfg_.ros_callback_en) return;     // 已经从 ROS callback 注入了

    if (cloud.empty()) {
        // 防呆：连续 100 次空点云 → 提示用户
        ...
        return;
    }
    updateRobotState(pose);                // 更新机体状态 + local box
    updateProbMap(cloud, pose);             // 调 ProbMap
    writeTimeConsumingToLog(time_log_file_);
}
```

---

## ProbMap —— 概率占用栅格

### log-odds 与 EGO 的等价关系

```
配置参数（YAML）：
  p_hit  = 0.7  → l_hit  = log(0.7/0.3)  ≈ +0.847
  p_miss = 0.4  → l_miss = log(0.4/0.6)  ≈ -0.405
  p_min  = 0.12 → l_min  = log(0.12/0.88) ≈ -1.991
  p_max  = 0.97 → l_max  = log(0.97/0.03) ≈ +3.497
  p_occ  = 0.80 → l_occ  = log(0.80/0.20) ≈ +1.386
  p_free = 0.30 → l_free = log(0.30/0.70) ≈ -0.847

每次观测更新：
  occupancy_buffer_[idx] += l_hit  (hit)
  occupancy_buffer_[idx] += l_miss (miss)
  occupancy_buffer_[idx] = clamp(..., l_min, l_max)

判定（与 EGO 三态一致）：
  isOccupied     ↔  occupancy_buffer_[idx] > l_occ
  isKnownFree    ↔  occupancy_buffer_[idx] < l_free
  isUnknown      ↔  l_free ≤ occupancy_buffer_[idx] ≤ l_occ
```

### updateOccPointCloud（prob_map.cpp L258-296）

```cpp
void ProbMap::updateOccPointCloud(const PointCloud& input_cloud) {
    const int cloud_in_size = input_cloud.size();
    Vec3f localmap_min = local_map_bound_min_d_;
    Vec3f localmap_max = local_map_bound_max_d_;

    for (int i = 0; i < cloud_in_size; i++) {
        if (cfg_.intensity_thresh > 0 && input_cloud[i].intensity < cfg_.intensity_thresh)
            continue;                     // 强度阈值过滤

        Vec3f p(input_cloud[i].x, input_cloud[i].y, input_cloud[i].z);
        if (p.z() > cfg_.virtual_ceil_height || p.z() < cfg_.virtual_ground_height)
            continue;

        Vec3i pt_id_g;
        posToGlobalIndex(p, pt_id_g);
        if (insideLocalMap(pt_id_g)) {
            // 关键：如果只是单纯加 l_hit，需要观测多次才能"占据"
            // 这里直接重复加 occ_hit_num 次，使一次激光命中就把栅格变成 occupied
            //（PCD/激光雷达模式下用，因为 LiDAR 只给一次端点）
            const int occ_hit_num = ceil(cfg_.l_occ / cfg_.l_hit);
            for (int j = 0; j < occ_hit_num; j++) insertUpdateCandidate(pt_id_g, true);

            localmap_max = localmap_max.cwiseMax(p);
            localmap_min = localmap_min.cwiseMin(p);
        }
    }
    if (cfg_.map_sliding_en) {
        local_map_bound_max_d_ = localmap_max;
        local_map_bound_min_d_ = localmap_min;
    }
    probabilisticMapFromCache();
    map_empty_ = false;
}
```

### updateProbMap —— 实时点云更新（L308-360）

```cpp
void ProbMap::updateProbMap(const PointCloud& cloud, const Pose& pose) {
    const Vec3f& pos = pose.first;

    // 1) 滑动窗：机体出地图范围 → reset
    if (cfg_.map_sliding_en && !insideLocalMap(pos) && raycast_data_.batch_update_counter == 0) {
        slideAllMap(pos);    // 整体平移（含 inf_map / esdf_map / fcnt_map）
        return;
    }

    // 2) 虚拟天花板/地面检查
    if (pos.z() > cfg_.virtual_ceil_height || pos.z() < cfg_.virtual_ground_height) return;

    // 3) 普通滑动（机体接近边界）
    if (raycast_data_.batch_update_counter == 0 && cfg_.map_sliding_en
        && (map_empty_ || (pos - local_map_origin_d_).norm() > cfg_.map_sliding_thresh)) {
        slideAllMap(pos);
    }

    // 4) 更新 local box（用于 raycast 截断）
    updateLocalBox(pos);

    // 5) 射线投射：把 cloud 中每个点 → 端点 hit、路径 miss
    raycastProcess(cloud, pos);
    raycast_data_.batch_update_counter++;

    // 6) 批量更新（攒够 N 帧再写回 occupancy_buffer_）
    if (raycast_data_.batch_update_counter >= cfg_.batch_update_size) {
        raycast_data_.batch_update_counter = 0;
        probabilisticMapFromCache();
        map_empty_ = false;
    }

    // 7) ESDF 更新（如启用）
    if (cfg_.esdf_en) esdf_map_->updateESDF3D(pos);

    // 8) 第一帧：清空机体周围 raycast_range_min 半径的所有 unknown
    static bool first = true;
    if (first) {
        first = false;
        for (double dx, dy, dz iterating ±raycast_range_min) {
            Vec3f p(dx, dy, dz);
            if (p.norm() <= cfg_.raycast_range_min) {
                missPointUpdate(pos + p, hash_id, 999);     // 强制写 miss × 999
            }
        }
    }
}
```

### probabilisticMapFromCache（L556+）

```cpp
while (!raycast_data_.update_cache_id_g.empty()) {
    Vec3i id_g = raycast_data_.update_cache_id_g.front();
    raycast_data_.update_cache_id_g.pop();

    int hash_id = getHashIndexFromGlobalIndex(id_g);

    // hit：该栅格被光击中 hit_cnt 次
    if (raycast_data_.hit_cnt[hash_id] > 0) {
        hitPointUpdate(pos, hash_id, raycast_data_.hit_cnt[hash_id]);   // += hit_cnt × l_hit
    }
    // miss：被光穿过但未击中（operation - hit）次
    missPointUpdate(pos, hash_id, raycast_data_.operation_cnt[hash_id]
                                  - raycast_data_.hit_cnt[hash_id]);
    raycast_data_.hit_cnt[hash_id] = 0;
    raycast_data_.operation_cnt[hash_id] = 0;
}
```

> 批量更新（`batch_update_size`）的作用：减少同一栅格的反复读写。多帧点云的 hit/miss 累加在 `hit_cnt`、`operation_cnt` 中，最后一次性写回 `occupancy_buffer_`。

---

## InfMap —— 膨胀地图

`inf_map_` 是单独的栅格，分辨率可以与 `prob_map` 不同（`cfg_.inflation_resolution`）。

每次 `prob_map` 中某栅格状态从"非占据"变成"占据"，触发膨胀逻辑：把周围 `obstacles_inflation / inflation_resolution` 格内的所有栅格 `inf_count += 1`；状态从"占据"变成"非占据"则 `inf_count -= 1`。

查询：`isOccupiedInflate(pos)` 等价于 `inf_count[hash(pos)] > 0`。

> 与 EGO 的差异：EGO 每帧重新做一次完整膨胀（O(occupied_grid × inflation_size³)）；ROG-Map 只在状态变化时增量更新（O(变化栅格 × inflation_size³)）。在动态场景下 ROG-Map 节约可观计算量。

---

## ESDFMap（可选，`esdf_en = true`）

实现的是 **3D Brushfire 算法**（FIM-Octomap、MIT esdf 同类）：

1. 维护 `dist[i]`（每个栅格到最近占据格的距离）；
2. 占据栅格变化时把它入队；
3. 从队列中取栅格，更新邻居距离 = dist[self] + cell_size，若有改进则入队；
4. 直到队列清空。

更新 ESDF 是 SUPER 中 **只在 `esdf_en = true` 时才发生** 的可选成本。当前 SUPER 主流程（`super_planner.cpp`、`exp_traj_optimizer_s4.cpp`）**没有显式查询 ESDF**——只在 `corridor_generator` 或 `astar` 的某些场景下间接使用。论文里 ESDF 是为"未来扩展（如 NMPC、避障 cost）"留的接口。

实践经验：高速测试（8 m/s）下关闭 ESDF（`esdf_en: false`）能省 30-50% 的地图更新耗时，且不影响避障性能。

---

## ROS 适配层（rog_map_ros1.hpp）

```cpp
class ROGMapROS1 : public ROGMap {
public:
    void init(rclcpp::Node::SharedPtr nh) {
        ROGMap::init();
        // 订阅点云
        cloud_sub_ = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
            cfg_.cloud_topic, 1,
            std::bind(&ROGMapROS1::cloudCallback, this, _1));
        // 订阅 odom
        odom_sub_ = nh->create_subscription<nav_msgs::msg::Odometry>(
            cfg_.odom_topic, 1,
            std::bind(&ROGMapROS1::odomCallback, this, _1));
        // 定时器：可视化
        viz_timer_ = nh->create_wall_timer(...);
    }
    // 把 PointCloud2 + Odometry 同步成 (cloud, pose) → updateMap
};
```

ROS2 版本的 `rog_map_ros2.hpp` 接口几乎一致，只是 nh 类型和订阅 API 替换为 rclcpp。

---

## ROG-Map 配置示例（`config/click.yaml`）

```yaml
rog_map:
  resolution: 0.15                    # 栅格分辨率（米）
  inflation_resolution: 0.15           # 膨胀图分辨率（与主图相同）
  half_map_size: [20, 20, 5]            # 半地图边长
  map_sliding_en: true
  map_sliding_thresh: 5.0               # 滑窗阈值
  fix_map_origin: [0, 0, 0]
  virtual_ceil_height: 4.0
  virtual_ground_height: -0.5
  obstacles_inflation: 0.3              # 障碍物膨胀半径
  raycast_range_min: 0.4
  raycast_range_max: 12.0
  batch_update_size: 1                  # 攒几帧再写回
  intensity_thresh: -1                  # 关闭强度阈值
  esdf_en: false                        # 关闭 ESDF（节省算力）
  frontier_extraction_en: false
  cloud_topic: "/cloud_registered"
  odom_topic: "/lidar_slam/odom"
  load_pcd_en: false
  ros_callback_en: true                 # 用 ROS 订阅而非外部 API
```
