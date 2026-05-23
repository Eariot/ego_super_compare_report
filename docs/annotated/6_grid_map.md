# grid_map.cpp —— EGO 的环境感知之眼
## 文件路径
`official/Fast-Drone-250/src/planner/plan_env/src/grid_map.cpp`

> GridMap 是 EGO 的地图模块，负责将传感器（深度相机/激光雷达）数据融合为一个占用栅格地图，供 BsplineOptimizer 进行碰撞检测。分为点云路径（L741+）和深度图像路径（L693+）两种输入模式。

---

## 整体架构

```
传感器数据输入（两条路径）:
路径A: PointCloud2 → cloudCallback()         [激光雷达模式]
路径B: depth image + odom → depthPoseCallback() / depthOdomCallback()  [深度相机模式]
                    ↓
              projectDepthImage() / 直接处理点云
                    ↓
              raycastProcess() ——————— 射线投射，更新占用概率
                    ↓
              clearAndInflateLocalMap() ——— 清边界 + 障碍物膨胀
                    ↓
         getInflateOccupancy(pos) ← BsplineOptimizer 在优化时查询碰撞
```

---

## initMap —— 地图初始化（L6-155）

### 整体流程

```cpp
void GridMap::initMap(ros::NodeHandle &nh)
{
    node_ = nh;

    /* ========== 读取所有地图参数 ========== */

    // 地图尺寸参数
    double x_size, y_size, z_size;
    node_.param("grid_map/resolution", mp_.resolution_, -1.0);          // 栅格分辨率（米/格）
    node_.param("grid_map/map_size_x", x_size, -1.0);                     // 地图 X 方向长度
    node_.param("grid_map/map_size_y", y_size, -1.0);
    node_.param("grid_map/map_size_z", z_size, -1.0);

    // 局部更新范围（只更新无人机周围区域，节省计算）
    node_.param("grid_map/local_update_range_x", mp_.local_update_range_(0), -1.0);
    node_.param("grid_map/local_update_range_y", mp_.local_update_range_(1), -1.0);
    node_.param("grid_map/local_update_range_z", mp_.local_update_range_(2), -1.0);

    // 障碍物膨胀半径（无人机体积补偿）
    node_.param("grid_map/obstacles_inflation", mp_.obstacles_inflation_, -1.0);

    // 相机内参（深度图像投影用）
    node_.param("grid_map/fx", mp_.fx_, -1.0);
    node_.param("grid_map/fy", mp_.fy_, -1.0);
    node_.param("grid_map/cx", mp_.cx_, -1.0);
    node_.param("grid_map/cy", mp_.cy_, -1.0);

    // 射线投射参数
    node_.param("grid_map/p_hit", mp_.p_hit_, 0.70);      // 击中概率
    node_.param("grid_map/p_miss", mp_.p_miss_, 0.35);    // 未击中概率
    node_.param("grid_map/p_min", mp_.p_min_, 0.12);      // 概率下限
    node_.param("grid_map/p_max", mp_.p_max_, 0.97);      // 概率上限
    node_.param("grid_map/p_occ", mp_.p_occ_, 0.80);      // 占据阈值
    node_.param("grid_map/min_ray_length", mp_.min_ray_length_, -0.1);  // 最小射线长度
    node_.param("grid_map/max_ray_length", mp_.max_ray_length_, -0.1);  // 最大感知距离

    // 地图原点计算：【关键】地图以世界原点为中心
    mp_.resolution_inv_ = 1 / mp_.resolution_;
    mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
    // 例如：map_size_x=18 → map_origin_.x() = -9（地图左边界在 -9m）
    mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

    // logit 变换：将概率转为对数几率，用于加法更新（避免乘法下溢）
    mp_.prob_hit_log_ = logit(mp_.p_hit_);      // log(0.70/0.30) ≈ +0.847
    mp_.prob_miss_log_ = logit(mp_.p_miss_);    // log(0.35/0.65) ≈ -0.619
    mp_.clamp_min_log_ = logit(mp_.p_min_);      // log(0.12/0.88) ≈ -1.991
    mp_.clamp_max_log_ = logit(mp_.p_max_);     // log(0.97/0.03) ≈ +3.497
    mp_.min_occupancy_log_ = logit(mp_.p_occ_); // log(0.80/0.20) ≈ +1.386
    mp_.unknown_flag_ = 0.01;

    // 计算各维度栅格数量
    for (int i = 0; i < 3; ++i)
        mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

    mp_.map_min_boundary_ = mp_.map_origin_;
    mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;
```

### 缓冲区初始化（L78-93）

```cpp
    /* ========== 初始化数据缓冲区 ========== */
    // 总栅格数 = nx × ny × nz
    int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

    // occupancy_buffer_：每个栅格的对数占用概率（主地图）
    // 初始值 = clamp_min - unknown_flag（表示"未知"）
    md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);

    // occupancy_buffer_inflate_：膨胀后的占据标志（用于快速查询）
    // 0 = 自由，1 = 占据，-1 = 未知
    md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);

    // 用于射线投射优化的计数缓冲
    md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
    md_.count_hit_ = vector<short>(buffer_size, 0);
    md_.flag_rayend_ = vector<char>(buffer_size, -1);
    md_.flag_traverse_ = vector<char>(buffer_size, -1);
    md_.raycast_num_ = 0;

    // 深度图像投影点缓冲（预分配，避免动态分配开销）
    md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
    md_.proj_points_cnt = 0;

    // 相机到机体的外参矩阵（XTDrone 默认值：相机朝前下方向）
    md_.cam2body_ << 0.0, 0.0, 1.0, 0.0,
                     -1.0, 0.0, 0.0, 0.0,
                      0.0,-1.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 1.0;
```

### 订阅器与发布器建立（L100-134）

```cpp
    /* ========== 建立订阅/发布/定时器 ========== */

    // 深度图像订阅（用于深度相机模式）
    depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(
        node_, "grid_map/depth", 50));

    // 相机外参订阅（VINS 输出的相机到机体外参）
    extrinsic_sub_ = node_.subscribe<nav_msgs::Odometry>(
        "/vins_fusion/extrinsic", 10, &GridMap::extrinsicCallback, this);

    // 根据 pose_type 选择融合方式：
    //   POSE_STAMPED(1): 深度图 + geometry_msgs::PoseStamped（人工标注/VO）
    //   ODOMETRY(2): 深度图 + nav_msgs::Odometry（里程计）
    if (mp_.pose_type_ == POSE_STAMPED)
    {
        pose_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(
            node_, "grid_map/pose", 25));
        sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
            SyncPolicyImagePose(100), *depth_sub_, *pose_sub_));
        sync_image_pose_->registerCallback(
            boost::bind(&GridMap::depthPoseCallback, this, _1, _2));
    }
    else if (mp_.pose_type_ == ODOMETRY)
    {
        odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(
            node_, "grid_map/odom", 100, ros::TransportHints().tcpNoDelay()));
        sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
            SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_));
        sync_image_odom_->registerCallback(
            boost::bind(&GridMap::depthOdomCallback, this, _1, _2));
    }

    // 点云订阅（用于激光雷达模式）
    indep_cloud_sub_ = node_.subscribe<sensor_msgs::PointCloud2>(
        "grid_map/cloud", 10, &GridMap::cloudCallback, this);
    indep_odom_sub_ = node_.subscribe<nav_msgs::Odometry>(
        "grid_map/odom", 10, &GridMap::odomCallback, this);

    // 定时器：驱动地图更新
    occ_timer_ = node_.createTimer(ros::Duration(0.05), &GridMap::updateOccupancyCallback, this);
    vis_timer_ = node_.createTimer(ros::Duration(0.11), &GridMap::visCallback, this);

    // 地图发布（RViz 可视化）
    map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy", 10);
    map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("grid_map/occupancy_inflate", 10);
```

---

## cloudCallback —— 点云模式地图更新（L741-839）

### 适用场景

激光雷达（LiDAR）模式：`pcl_render_node` 将 Gazebo 深度图/点云发布到 `/pcl_render_node/points`，EGO 的 `cloudCallback` 处理这些点云，直接将点标记为占据栅格（无需射线投射）。

```cpp
void GridMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &img)
{
    // 1. 解析 PointCloud2 消息为 PCL 点云格式
    pcl::PointCloud<pcl::PointXYZ> latest_cloud;
    pcl::fromROSMsg(*img, latest_cloud);

    md_.has_cloud_ = true;

    // 安全检查：没有里程计数据则跳过
    if (!md_.has_odom_)
    {
        std::cout << "no odom!" << std::endl;
        return;
    }

    // 过滤空点云
    if (latest_cloud.points.size() == 0)
        return;

    // 2. 重置局部区域：清除无人机周围区域的膨胀占据标志
    // 作用：每次更新时，先把局部区域清零，再重新标记
    this->resetBuffer(md_.camera_pos_ - mp_.local_update_range_,
                      md_.camera_pos_ + mp_.local_update_range_);

    // 3. 计算膨胀参数
    int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
    int inf_step_z = 1;  // Z 方向膨胀 1 格（因为无人机上下方向不需要很大膨胀）

    // 4. 遍历点云中的每个点
    for (size_t i = 0; i < latest_cloud.points.size(); ++i)
    {
        pt = latest_cloud.points[i];
        p3d(0) = pt.x; p3d(1) = pt.y; p3d(2) = pt.z;

        Eigen::Vector3d devi = p3d - md_.camera_pos_;  // 到相机距离

        // 只处理在局部更新范围内的点
        if (fabs(devi(0)) < mp_.local_update_range_(0) &&
            fabs(devi(1)) < mp_.local_update_range_(1) &&
            fabs(devi(2)) < mp_.local_update_range_(2))
        {
            // 对每个障碍点，沿 XYZ 各膨胀 inf_step 格
            for (int x = -inf_step; x <= inf_step; ++x)
            for (int y = -inf_step; y <= inf_step; ++y)
            for (int z = -inf_step_z; z <= inf_step_z; ++z)
            {
                p3d_inf(0) = pt.x + x * mp_.resolution_;
                p3d_inf(1) = pt.y + y * mp_.resolution_;
                p3d_inf(2) = pt.z + z * mp_.resolution_;

                // 更新包围盒（用于确定更新区域边界）
                max_x = max(max_x, p3d_inf(0));
                // ...

                // 转换到栅格坐标
                posToIndex(p3d_inf, inf_pt);

                // 检查是否在地图范围内
                if (!isInMap(inf_pt))
                    continue;

                int idx_inf = toAddress(inf_pt);
                md_.occupancy_buffer_inflate_[idx_inf] = 1;  // 标记为占据
            }
        }
    }

    // 5. 更新局部边界（用于可视化发布区域）
    posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
    posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);
    boundIndex(md_.local_bound_min_);
    boundIndex(md_.local_bound_max_);
}
```

**点云模式 vs 深度图模式的区别**：

| 特性 | 点云模式（cloudCallback） | 深度图模式（depthPoseCallback） |
|---|---|---|
| 数据来源 | pcl_render_node / LiDAR | 深度相机（Realsense等） |
| 占据更新 | 直接标记障碍点为占据 | 射线投射（hit 端占据，路径 free） |
| 计算量 | O(点数 × 膨胀半径³) | O(像素数 × 射线长度) |
| 适用场景 | 仿真（Gazebo 点云） | 实机（深度相机） |

---

## updateOccupancyCallback —— 定时驱动地图更新（L649-691）

```cpp
void GridMap::updateOccupancyCallback(const ros::TimerEvent & /*event*/)
{
    // 记录最后更新时刻
    if (md_.last_occ_update_time_.toSec() < 1.0)
        md_.last_occ_update_time_ = ros::Time::now();

    // 如果没有数据需要更新，且开启了深度融合超时检测
    if (!md_.occ_need_update_)
    {
        if (md_.flag_use_depth_fusion &&
            (ros::Time::now() - md_.last_occ_update_time_).toSec() > mp_.odom_depth_timeout_)
        {
            // 超过 timeout 秒没有收到 odom 或 depth → 地图不可信
            ROS_ERROR("odom or depth lost! ...");
            md_.flag_depth_odom_timeout_ = true;  // 触发 EGO 急停
        }
        return;
    }

    md_.last_occ_update_time_ = ros::Time::now();

    /* 三步更新流程 */
    projectDepthImage();       // 步骤1：深度图像 → 世界坐标系 3D 点
    raycastProcess();           // 步骤2：射线投射，更新概率占用栅格

    if (md_.local_updated_)    // 步骤3：清理边界 + 障碍物膨胀
        clearAndInflateLocalMap();

    md_.occ_need_update_ = false;
    md_.local_updated_ = false;
}
```

---

## raycastProcess —— 射线投射（L333-499）

### 核心思想

射线投射是建立 3D 占据栅格地图的标准方法：

```
相机位置 ─────────────────────────▶ 障碍点
     ↓ 射线（路径上的点）都标记为 free
     ├──── free
     ├──── free
     ├──── free
     ●占据 ← 深度图测量的端点
```

- **hit（击中）**：深度图像中每个像素代表一个 3D 点，该点射线端点 → 标记为占据
- **miss（未击中）**：从相机到端点的射线上所有栅格 → 标记为空旷
- **概率融合**：同一个栅格可能被多次观测，用 logit 概率累加

---

## clearAndInflateLocalMap —— 边界清理与障碍物膨胀（L527-639）

```cpp
void GridMap::clearAndInflateLocalMap()
{
    /* ---- 步骤1：清边界区域 ---- */
    // 局部地图四周之外的区域设为"未知"（不在 occupancy_buffer_ 中累积概率）
    // 使用 5 格 margin 避免边界突变

    // Z 方向清理（上下边界）
    for (x = min_cut_m.x; x <= max_cut_m.x; ++x)
    for (y = min_cut_m.y; y <= max_cut_m.y; ++y)
    {
        // Z 方向小于下界 → 设为未知
        for (z = min_cut_m.z; z < min_cut.z; ++z)
            md_.occupancy_buffer_[toAddress(x,y,z)] = mp_.clamp_min_log_ - mp_.unknown_flag_;
        // Z 方向大于上界 → 设为未知
        for (z = max_cut.z+1; z <= max_cut_m.z; ++z)
            md_.occupancy_buffer_[toAddress(x,y,z)] = mp_.clamp_min_log_ - mp_.unknown_flag_;
    }
    // 类似处理 X 和 Y 方向的边界清理

    /* ---- 步骤2：障碍物膨胀 ---- */
    // inf_step = ceil(obstacles_inflation_ / resolution_)
    // 例如：inflation=0.2m, resolution=0.1m → inf_step=2
    // 膨胀范围 = (2×2+1)³ = 125 个栅格/障碍点

    // 先清零局部区域的膨胀缓冲区
    for (x = local_bound_min.x; x <= local_bound_max.x; ++x)
    for (y = local_bound_min.y; y <= local_bound_max.y; ++y)
    for (z = local_bound_min.z; z <= local_bound_max.z; ++z)
        md_.occupancy_buffer_inflate_[toAddress(x,y,z)] = 0;

    // 再对所有占据栅格进行膨胀
    for (x = local_bound_min.x; x <= local_bound_max.x; ++x)
    for (y = local_bound_min.y; y <= local_bound_max.y; ++y)
    for (z = local_bound_min.z; z <= local_bound_max.z; ++z)
    {
        if (md_.occupancy_buffer_[toAddress(x,y,z)] > mp_.min_occupancy_log_)  // 占据栅格
        {
            // 膨胀：将 inf_step 范围内的所有栅格标记为 inflate_=1
            inflatePoint(Eigen::Vector3i(x,y,z), inf_step, inf_pts);
            for (k = 0; k < inf_pts.size(); ++k)
            {
                inf_pt = inf_pts[k];
                int idx_inf = toAddress(inf_pt);
                if (idx_inf >= 0 && idx_inf < buffer_size)
                    md_.occupancy_buffer_inflate_[idx_inf] = 1;
            }
        }
    }
}
```

---

## getInflateOccupancy —— 碰撞查询（内联函数，grid_map.h L326-333）

```cpp
// BsplineOptimizer 在优化时反复调用此函数判断控制点是否在障碍物中
// 这是整个地图系统被查询次数最多的函数，必须是 inline
inline int GridMap::getInflateOccupancy(Eigen::Vector3d pos)
{
    // 检查是否在地图范围内
    if (!isInMap(pos)) return -1;

    // 坐标 → 栅格索引
    Eigen::Vector3i id;
    posToIndex(pos, id);

    // 返回膨胀占据标志：0=自由, 1=占据（含膨胀）, -1=地图外
    return int(md_.occupancy_buffer_inflate_[toAddress(id)]);
}
```

**为什么用 `occupancy_buffer_inflate_` 而非 `occupancy_buffer_`？**
- `occupancy_buffer_`：原始概率栅格，需要和 `min_occupancy_log_` 比较
- `occupancy_buffer_inflate_`：已膨胀的二进制标志（0/1），查询更快
- EGO 碰撞检测只关心"是否占据"，不需要概率值

---

## 关键内联函数

### posToIndex（L367-369）

```cpp
// 世界坐标 → 栅格索引
// id(i) = floor((pos(i) - map_origin_(i)) / resolution_)
inline void GridMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id)
{
    for (int i = 0; i < 3; ++i)
        id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
}
```

### indexToPos（L371-373）

```cpp
// 栅格索引 → 世界坐标（取栅格中心点）
// pos(i) = (id(i) + 0.5) * resolution_ + map_origin_(i)
inline void GridMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos)
{
    for (int i = 0; i < 3; ++i)
        pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}
```

### inflatePoint（L375-402）

```cpp
// 对一个占据栅格，在 inf_step 立方体范围内生成所有邻居栅格
// 用于障碍物膨胀
// 默认使用"全向膨胀"（所有 3 轴方向），注释掉的是"十字膨胀"
inline void GridMap::inflatePoint(const Eigen::Vector3i& pt, int step, vector<Eigen::Vector3i>& pts)
{
    int num = 0;
    for (int x = -step; x <= step; ++x)
    for (int y = -step; y <= step; ++y)
    for (int z = -step; z <= step; ++z)
    {
        pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
    }
}
```

---

## 地图参数与占用概率体系

```
概率 → logit → 存储值：
  p_hit=0.70  → logit(0.70) = +0.847  (被射线击中，占据概率↑)
  p_miss=0.35 → logit(0.35) = -0.619  (射线经过但无击中，占据概率↓)
  p_min=0.12  → logit(0.12) = -1.991  (概率下限，clamp 到此)
  p_max=0.97  → logit(0.97) = +3.497  (概率上限，clamp 到此)
  p_occ=0.80  → logit(0.80) = +1.386  (占据阈值，用于判断占用/空闲)

每次观测更新：
  occupancy_buffer_[idx] += prob_hit_log_   (hit)
  occupancy_buffer_[idx] += prob_miss_log_   (miss)
  occupancy_buffer_[idx] = clamp(occupancy_buffer_[idx], clamp_min_log_, clamp_max_log_)

查询：
  occupancy_buffer_[idx] > min_occupancy_log_ → 占据
  occupancy_buffer_[idx] <= min_occupancy_log_ → 自由
```
