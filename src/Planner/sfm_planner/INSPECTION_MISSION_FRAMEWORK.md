# 隧道掌子面 SfM 任务框架与 SfM-Planner 改造方案

## 1. 目标与范围

本方案在现有 SfM-Planner 的 `state2state` 点到点轨迹规划能力之上，增加一个面向隧道掌子面拍摄的任务层（`InspectionMissionPlanner`）。每次作业只有一个**先验任务目标**，而不是一串需要逐点停车的通行航点。

一次任务的闭环为：

```text
触发任务
  → 连续导航到上一次掌子面的安全接近点
  → 识别当前掌子面
  → 生成并访问覆盖拍摄视点
  → 完成拍照后覆盖保存下一次任务目标
  → 连续导航回 Home
```

本文只定义与规划、任务编排、掌子面几何和视点覆盖有关的框架。飞控起飞/降落、相机 SDK、原始图像存储和 SfM 离线重建由外部模块实现，但须遵循本文定义的输入输出接口。

### 1.1 非目标

- 不将现有 `state2state` 改造成完整的相机或飞控任务系统。
- 不构建“通行航点队列”；前往掌子面和返航均是单目标连续导航。
- 不直接引入 FC-Planner 的骨架分解、全局覆盖和 LKH 路径求解。
- 不直接复制 FC-Planner 源码；其 GPL-3.0 许可证需要在复用源码前单独评估。

## 2. 核心设计原则

1. **运动层与任务层分离**：`state2state` 只负责当前单一目标的轨迹生成、避障和滚动重规划；`InspectionMissionPlanner` 决定下一阶段做什么。
2. **目标覆盖而非追加**：一次任务成功后，以新识别出的掌子面目标覆盖旧目标，供下一次爆破后的任务使用。
3. **中心与飞行点分离**：掌子面中心是表面几何点，不一定可飞；无人机导航到沿法向偏移后的安全接近点。
4. **两阶段提交**：识别成功只创建 `pending_target`；只有覆盖拍摄完成才持久化覆盖现有目标。
5. **所有数据使用同一世界坐标系**：机体位姿、先验目标、掌子面点云、视点、Home 和地图版本必须处于同一 `world/map` 坐标系。

## 3. 总体架构

```text
┌───────────────────────────────────────────────────────────────┐
│                         外部系统                               │
│  地面端 trigger | 在线 SLAM/地图 | FaceDetector | Camera/云台 │
└───────┬─────────────────────┬────────────────────────┬────────┘
        │                     │                        │
        ▼                     ▼                        ▼
┌─────────────────────── SfM-Planner / fsm_node ─────────────────┐
│                                                                  │
│  FsmRos1                                                        │
│  ├─ ROS 服务、订阅和发布                                        │
│  └─ Fsm                                                         │
│     ├─ InspectionMissionPlanner  ← 任务状态和任务数据           │
│     ├─ FaceViewpointPlanner      ← 覆盖视点生成/筛选/排序        │
│     └─ State2StateTaskExecutor   ← 单目标轨迹规划和滚动重规划    │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

### 3.1 调用关系

```text
MissionPlanner
  ├─ 发送 NavigationRequest(APPROACH_TARGET) → state2state
  ├─ 到达后发送 FaceDetectionRequest           → FaceDetector
  ├─ 接收 FaceObservation                      ← FaceDetector
  ├─ 调用 FaceViewpointPlanner
  ├─ 发送 NavigationRequest(CAPTURE_VIEWPOINT) → state2state
  ├─ 到达后发送 CaptureRequest                 → CameraManager
  ├─ 接收 CaptureResult                        ← CameraManager
  └─ 发送 NavigationRequest(HOME)              → state2state
```

`state2state` 始终只看到一个当前目标。前往先验目标和拍摄视点仍用局部直线/A*；`RETURN_HOME` 则采用保守的双层路线：先要求 topo A* 给出**真正连到 Home**的全局骨架；图尚未连通时，反向执行去程和拍摄阶段实际飞过、且已在局部膨胀地图中验证过的 breadcrumb。两者均不可用时，长距离返航拒绝退化为普通直线/A*，保持当前位置并报告失败，而不是穿过未知区域。

### 3.2 Topo 与 verified breadcrumb 返航

```text
起飞 Home
  → 每 0.5 m 记录一次已执行轨迹
  → 对每段执行 KNOWN_FREE + inflated line check
  → accepted segment 同时写入 breadcrumb 与 topo evidence seed
  → RETURN_HOME:
      complete topo A* to Home ?  使用 topo prefix
      : breadcrumb 可附着 ?       反向 retrace breadcrumb prefix
      :                           停止当前返航规划并报告不可达
```

breadcrumb 不是规划器预测路径，也不把 Unknown 标为 Free；它是当前任务生命期内的已验证安全走廊。Topo 图则可在这些已验证 seed 及 LiDAR 状态变化的基础上逐步产生替代通路。每次只交给局部规划器前方有限 prefix，prefix 仍需在当前地图重新验证；新 topo snapshot 发布后，旧 topo 路线在查询间隔后会重新搜索。

## 4. 任务状态机

```cpp
enum class InspectionState {
    IDLE,
    GO_TO_TARGET,
    WAIT_FACE_RESULT,
    PLAN_VIEWS,
    GO_TO_VIEWPOINT,
    WAIT_CAPTURE_RESULT,
    RETURN_HOME,
    FINISHED,
    FAILED
};

enum class NavigationRole {
    EXTERNAL_CLICK,
    APPROACH_TARGET,
    CAPTURE_VIEWPOINT,
    HOME
};
```

| 当前状态 | 事件 | 动作 | 下一状态 |
| --- | --- | --- | --- |
| `IDLE` | `StartInspection` | 记录 Home，读取并冻结当前目标，下发安全接近点 | `GO_TO_TARGET` |
| `GO_TO_TARGET` | 导航成功 | 发布掌子面识别请求 | `WAIT_FACE_RESULT` |
| `WAIT_FACE_RESULT` | 有效掌子面结果 | 构造 `pending_target`，规划覆盖视点 | `PLAN_VIEWS` |
| `PLAN_VIEWS` | 视点集有效 | 下发首个视点 | `GO_TO_VIEWPOINT` |
| `GO_TO_VIEWPOINT` | 导航成功 | 发布该视点的拍照请求 | `WAIT_CAPTURE_RESULT` |
| `WAIT_CAPTURE_RESULT` | 拍照成功 | 下发下一视点，或提交新目标并返航 | `GO_TO_VIEWPOINT` / `RETURN_HOME` |
| `RETURN_HOME` | 导航成功 | 发布任务结果 | `FINISHED` |
| 任意非终态 | 任务失败/取消 | 不覆盖目标，返航 | `RETURN_HOME` |

拍摄视点之间需要悬停，以满足曝光与照片位姿匹配要求；因此每个视点从静止出发并到达静止是有意设计，并非通行导航的性能问题。

## 5. 任务数据与目标覆盖

### 5.1 数据模型

```cpp
struct ChangeRegion {
    Eigen::Vector3d center;
    Eigen::Vector3d normal;
    double width;
    double height;
    double thickness;
};

struct MissionTarget {
    uint32_t version{0};
    std::string scene_id;
    std::string map_version;

    Eigen::Vector3d face_center;  // 掌子面几何中心
    Eigen::Vector3d face_normal;  // 朝向当前已知自由空间
    Eigen::Vector3d nav_goal;     // 下一次实际导航到的安全接近点
    double goal_yaw{0.0};

    double confidence{0.0};
    ChangeRegion previous_face_region;
};

struct MissionContext {
    MissionTarget active_target;       // trigger 时冻结，不可被中途覆盖
    MissionTarget pending_target;      // 本次识别成功后的候选目标
    bool has_pending_target{false};

    Pose home_pose;
    std::vector<CaptureViewpoint> viewpoints;
    std::size_t viewpoint_index{0};
};
```

### 5.2 目标生成

若识别结果为掌子面中心 `c`、法向 `n`，且 `n` 指向无人机所在的自由空间，则：

```text
next_target.face_center = c
next_target.face_normal = n
next_target.nav_goal    = c + d_approach × n
```

`nav_goal` 由沿法向的安全搜索产生，而不是使用任意“最近自由点”投影：

```cpp
for (double d = approach_distance_min;
     d <= approach_distance_max;
     d += approach_distance_step) {
    const Vec3f candidate = center + d * normal;
    if (isKnownFree(candidate) &&
        clearance(candidate) >= safe_radius &&
        withinFlightHeight(candidate)) {
        return candidate;
    }
}
```

这能保证下一次任务的目标位于掌子面前方、可飞且方向可解释。

### 5.3 两阶段提交

```text
掌子面识别成功
  → 构造并验证 pending_target（仅内存）
  → 覆盖拍摄全部完成
  → 写入 target.yaml.tmp
  → flush、校验、原子重命名
  → persistent_target = pending_target
```

若识别失败、视点规划失败、拍摄失败或任务取消，则不更新目标。下一次任务仍使用最后一次完整任务保存的目标。

### 5.4 持久化示例

```yaml
scene_id: qingyuan_tunnel
target_version: 12
map_version: map_0012

face_center: [32.41, -1.28, 2.36]
face_normal: [-0.998, 0.041, 0.027]
face_prior_valid: true
nav_goal: [29.42, -1.16, 2.44]
goal_yaw: 0.03
confidence: 0.91

change_region:
  center: [32.41, -1.28, 2.36]
  normal: [-0.998, 0.041, 0.027]
  width: 8.2
  height: 5.7
  thickness: 1.0
```

## 6. 掌子面识别接口与算法

### 6.1 FaceObservation

```cpp
struct FaceObservation {
    bool valid{false};
    Eigen::Vector3d center;
    Eigen::Vector3d normal;
    pcl::PointCloud<pcl::PointXYZ>::Ptr surface_cloud;
    double width{0.0};
    double height{0.0};
    double area{0.0};
    double confidence{0.0};
};
```

所有字段必须在 `world/map` 坐标系中给出，点云时间戳应与用于识别的无人机位姿一致。

### 6.2 识别流程

```text
实时点云
  → 按隧道前进方向截取前方 ROI
  → 体素降采样、统计离群点移除
  → 欧式聚类
  → 每个聚类执行 RANSAC 平面拟合 + PCA
  → 按方向、面积、平面性、密度、前方位置打分
  → 多帧一致性确认
  → FaceObservation
```

每一轮识别的 ROI 前进方向由该轮导航目标的朝向定义：

```text
tunnel_dir = [cos(active_target.goal_yaw), sin(active_target.goal_yaw), 0]
```

因此首任务可直接使用人工给定的 `(nav_goal, goal_yaw)` 作为观察锚点；不应从
上一次识别面的法向推导方向。`face_prior_valid: false` 表示该锚点没有可用几何先验，
检测器不会以其 `face_center/face_normal` 过滤候选。完整拍摄成功后，任务把新检测结果
覆盖写回为 `face_prior_valid: true` 的目标；后续轮次才可选用中心与法向先验。

候选掌子面应满足：

- 位于无人机前方；
- 平面法向与 `tunnel_dir` 基本平行；
- 具有足够点数、宽高和面积；
- PCA 平面残差较小；
- 不是侧壁、顶板、地面或局部设备；
- 连续多帧的中心、法向稳定。

法向统一指向无人机/自由空间：

```cpp
if (normal.dot(robot_position - center) < 0.0) {
    normal = -normal;
}
```

中心建议在平面局部坐标系计算二维凸包中心，而不直接取点云均值，减少遮挡与点密度不均造成的偏移。

## 7. 掌子面覆盖视点规划

### 7.1 设计选择

参考 [FC-Planner Viewpoint Manager](https://github.com/HKUST-Aerial-Robotics/FC-Planner/tree/master/FC-Planner/src/viewpoint_manager) 的思想：

```text
地图点云 + 待覆盖表面点云 + 表面法向 + 相机模型
→ 候选视点
→ FOV/遮挡/安全筛选
→ 视点裁剪
→ 未覆盖区域检查
```

不使用其面向大型复杂三维场景的骨架空间分解、全局路径和 LKH。掌子面是局部近似平面，采用轻量的平面采样和贪心覆盖即可。

### 7.2 接口

```cpp
class FaceViewpointPlanner {
public:
    CoveragePlan plan(const FaceObservation& face,
                      const MapManager& map,
                      const CameraModel& camera,
                      const Eigen::Vector3d& current_position,
                      const Eigen::Vector3d& home);
};

struct CaptureViewpoint {
    uint32_t id;
    Eigen::Vector3d position;
    double body_yaw;
    double camera_pitch;
    std::vector<int> visible_surface_ids;
    double expected_coverage_gain;
};

struct CoveragePlan {
    bool valid{false};
    std::vector<CaptureViewpoint> ordered_viewpoints;
    double predicted_coverage{0.0};
};
```

### 7.3 候选点与覆盖

在掌子面上建立局部正交坐标系：

```text
n：掌子面法向
u：掌子面水平轴
v：掌子面竖直轴
```

相机在拍摄距离 `d` 下的理论覆盖范围：

```text
footprint_width  = 2 × d × tan(horizontal_fov / 2)
footprint_height = 2 × d × tan(vertical_fov / 2)
```

根据期望图像重叠率，在 `u-v` 平面采样候选视点：

```text
view_position = face_center + offset_u × u + offset_v × v + d × n
```

候选视点必须通过：

- 自由空间和最小安全净距检查；
- 相机距离、FOV、俯仰角、云台范围检查；
- 从视点到表面点的射线无遮挡；
- 目标可由 `state2state` 到达；
- 入射角约束。

对于每个表面点 `p_j` 和候选视点 `v_i`：

```text
visible(i, j) =
    inFov(v_i, p_j)
    && inRange(v_i, p_j)
    && raycastFree(v_i, p_j)
    && incidenceAngleValid(v_i, p_j)
```

由于数据用于 SfM，覆盖条件不应只是“每个点可见一次”，而应要求：

```text
每个表面点至少被 K 个有效视点观察，
并满足最小视角基线或多视角差异。
```

使用贪心 `K-coverage` 选择视点：每轮加入对尚未达到 K 次观测的表面点新增贡献最大的候选视点。最后按以下总代价排序：

```text
当前无人机位置 → 所有视点 → Home
```

视点数较少时，最近邻初始化加 2-opt 局部优化足够；若附近障碍复杂，可使用局部路径长度替代欧氏距离。

## 8. 先验地图与变化区域

上一任务的先验 PCD 中包含旧掌子面；下一次爆破后该墙面已被移除。仅更新目标文件不足以解决问题，因为规划器仍可能被旧墙面阻挡。

每次识别掌子面时保存 `ChangeRegion`。下一次加载先验地图时：

```text
先验 PCD
  → 对 previous_face_region 进行薄层掩膜/清除
  → 该区域作为可在线更新区域
  → 实时雷达重新确认自由空间与新掌子面
```

只清除旧掌子面所在的薄层，不能删除整个末端区域；侧壁、地面、顶板及其他已知静态结构应继续作为先验障碍物。

## 9. ROS 接口

### 9.1 服务、订阅与发布

| 接口 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/inspection/start` | `StartInspection.srv` | 输入 | 触发一次任务 |
| `/inspection/face/request` | `FaceDetectionRequest.msg` | 输出 | 请求外部掌子面识别，并携带任务关联字段 |
| `/inspection/face/result` | `FaceObservation.msg` | 输入 | 返回掌子面几何和表面点云 |
| `/inspection/viewpoints` | `visualization_msgs/MarkerArray` | 输出（latched） | 掌子面边界、编号视点、拍摄顺序和视线 |
| `/inspection/capture/request` | `CaptureRequest.msg` | 输出 | 请求云台在指定视点拍摄 |
| `/inspection/capture/result` | `CaptureResult.msg` | 输入 | 返回拍照是否成功 |
| `/inspection/status` | `MissionStatus.msg` | 输出 | 阶段、视点序号、失败原因和版本 |

### 9.2 建议消息

所有感知和拍摄请求/结果必须携带同一组关联键：`mission_id`、`target_version`、`request_id`。任务层只接收与当前等待请求完全匹配的消息，避免上一轮任务的延迟结果推进新任务。

`FaceDetectionRequest.msg`：

```text
std_msgs/Header header
string mission_id
uint32 target_version
uint32 request_id
```

`FaceObservation.msg`：

```text
std_msgs/Header header
string mission_id
uint32 target_version
uint32 request_id
bool valid
geometry_msgs/Point center
geometry_msgs/Vector3 normal
float64 width
float64 height
float64 area
float64 confidence
sensor_msgs/PointCloud2 surface_cloud
```

`CaptureResult.msg`：

```text
std_msgs/Header header
string mission_id
uint32 target_version
uint32 request_id
uint32 viewpoint_id
bool success
string image_id
string reason
```

`StartInspection.srv`：

```text
---
bool accepted
string mission_id
string reason
```

## 10. SfM-Planner 修改清单

### 10.1 新增文件

```text
src/Planner/sfm_planner/
├── include/mission/
│   ├── mission_types.hpp
│   ├── inspection_mission_planner.hpp
│   └── mission_target_store.hpp
├── include/coverage/
│   ├── face_detector_contract.hpp
│   └── face_viewpoint_planner.hpp
├── src/mission/
│   ├── inspection_mission_planner.cpp
│   └── mission_target_store.cpp
├── src/coverage/
│   └── face_viewpoint_planner.cpp
├── msg/
│   ├── FaceObservation.msg
│   ├── CaptureRequest.msg
│   ├── CaptureResult.msg
│   └── MissionStatus.msg
└── srv/
    └── StartInspection.srv
```

当前 `CMakeLists.txt` 使用 `file(GLOB_RECURSE srcs ...)` 收集 `.cpp` 文件，因此新增源文件会自动编译进 `sfm_planner_core`；消息和服务生成规则仍需显式添加。

### 10.2 `include/fsm/config.hpp`

新增 `InspectionMissionConfig` 或等价配置字段：

- 是否启用 inspection mission；
- 目标文件路径；
- ROS 接口名称；
- 掌子面 ROI、置信度、面积、法向阈值；
- 安全接近距离与安全半径；
- 相机 FOV、拍摄距离、重叠率和最少观测次数；
- 失败重试次数和地图变化区域参数。

### 10.3 `include/fsm/fsm.h`

保留现有 `MACHINE_STATE`，不将识别/拍照直接混入轨迹执行状态。

新增：

```cpp
std::unique_ptr<mission::InspectionMissionPlanner> inspection_mission_;
NavigationRole active_navigation_role_{NavigationRole::EXTERNAL_CLICK};

bool startInspectionMission();
void onFaceObservation(const FaceObservation& observation);
void onCaptureResult(const CaptureResult& result);
bool submitMissionNavigationGoal(const Pose& goal, NavigationRole role);
```

并为 ROS 层增加虚函数：

```cpp
virtual void publishFaceDetectionRequest(const FaceDetectionRequest&) {}
virtual void publishCaptureRequest(const CaptureCommand&) {}
virtual void publishMissionStatus(...) {}
```

### 10.4 `src/general_core/fsm.cpp`

现有 `setGoalPosiAndYaw()` 继续保留给 RViz/人工点击调试；任务目标通过 `submitMissionNavigationGoal()` 下发，并记录 `NavigationRole`。

在 `handleExecutedTrajectoryFinished()` 中，检测任务目标真正到达后向 `InspectionMissionPlanner` 报告结果：

```cpp
inspection_mission_->onNavigationSucceeded(
    active_navigation_role_, robot_state_);
```

不要让原有 `state2state` 自动决定识别、拍照、返航等阶段。完成当前目标后回到 `WAIT_GOAL`，由 MissionPlanner 决定下一条导航请求或外部请求。

### 10.5 `include/ros_interface/ros1/fsm_ros1.hpp`

新增：

```cpp
ros::ServiceServer inspection_start_srv_;
ros::Subscriber face_observation_sub_;
ros::Subscriber capture_result_sub_;

ros::Publisher face_request_pub_;
ros::Publisher capture_request_pub_;
ros::Publisher mission_status_pub_;
```

在 `init()` 中注册服务、订阅与发布；将回调转发给基类 `Fsm`。任务执行期间，外部 RViz 点击目标应被拒绝或解释为显式取消，以防覆盖当前任务目标。

### 10.6 CMake 与 package.xml

在 `CMakeLists.txt` 中增加：

```cmake
add_message_files(
  FILES
  FaceObservation.msg
  FaceDetectionRequest.msg
  CaptureRequest.msg
  CaptureResult.msg
  MissionStatus.msg
)

add_service_files(FILES StartInspection.srv)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)
```

并在 `catkin_package()` 和 `package.xml` 中补充 `message_runtime`；若使用 `std_srvs` 或其他标准服务，也补充对应依赖。

## 11. 配置示例

```yaml
inspection_mission:
  enable: true
  # MaRSIM 激光-only 联调：只验证到达 nav_goal，不触发或伪造识别/拍摄成功。
  navigation_only: false
  # 首帧新鲜里程计到达后自动执行一次任务；launch 全流程联调可设为 true。
  auto_start: false
  # 2D /goal 的 XY 作为本次任务接近航点。
  trigger_from_2d_goal: false
  # 无识别器时用 mission_target 中已有掌子面几何直接生成视点；
  # 此路径不会覆盖更新 target。
  skip_face_detection: false
  # 默认直接飞向 target_file 中明确配置的 nav_goal。
  use_target_nav_goal_directly: true
  # 外部识别、相机回执的最大等待时间；超时后安全返航。
  face_result_timeout_sec: 15.0
  capture_result_timeout_sec: 10.0
  target_file: "config/mission_target.yaml"

  start_service: "/inspection/start"
  face_request_topic: "/inspection/face/request"
  face_result_topic: "/inspection/face/result"
  # MaRSIM can run the LiDAR detector while capture acknowledgement remains
  # simulated.  The debug topic contains only accepted plane detections.
  cloud_topic: "/cloud_registered"
  mock_face_detection: false
  mock_capture: true
  use_internal_detector: true
  face_debug_topic: "/inspection/face/debug"
  viewpoint_debug_topic: "/inspection/viewpoints"
  capture_request_topic: "/inspection/capture/request"
  capture_result_topic: "/inspection/capture/result"
  status_topic: "/inspection/status"

  home_mode: "capture_on_trigger"

  approach_distance_min: 2.0
  approach_distance_max: 4.0
  approach_distance_step: 0.2
  safe_radius: 0.6

  face:
    forward_min: 1.0
    forward_max: 12.0
    min_confidence: 0.75
    min_area: 4.0
    min_points: 300
    normal_alignment_min: 0.8
    cluster_tolerance: 0.35
    cluster_min_size: 200
    ransac_dist: 0.08
    stability_frames: 3
    stability_center_tol: 0.35
    stability_normal_tol: 0.15
    # The detected plane must also agree with the coarse prior from
    # mission_target; this prevents a far tunnel wall from becoming the face.
    prior_center_tolerance: 3.0
    prior_normal_alignment_min: 0.9

  coverage:
    camera_hfov_deg: 70.0
    camera_vfov_deg: 50.0
    capture_distance: 4.0
    image_overlap: 0.7
    min_observation_count: 2
    min_baseline_angle_deg: 8.0
    min_predicted_coverage: 0.95
    max_viewpoints: 60
    # 每张照片在到达视点后先悬停；例如设为 2.0 即悬停两秒。
    capture_settle_time_sec: 0.5
```

以上数值仅用于说明配置结构，必须通过现场基准测试确定。

## 12. 实施阶段与验收

### 阶段 A：任务导航闭环

在没有相机模型的 MaRSIM 中，存在两种互不混淆的测试路径：

1. 设置 `navigation_only: true`，只验证 `GO_TO_TARGET → RETURN_HOME → FINISHED(navigation_only_complete)`；绝不写回目标文件。
2. 设置 `use_internal_detector: true`、`mock_face_detection: false`、`mock_capture: true`，从 MaRSIM 的 `/cloud_registered`（`world` 系）检测掌子面平面，生成视点；仅相机回执为模拟。成功检测会发布到 `/inspection/face/debug`，可用 `rostopic echo` 记录中心、法向、面积和置信度。该模式可验证点云检测和视点规划，但不能把模拟拍摄解释为真实影像覆盖。

若设置 `auto_start: true`，节点会等待第一帧新鲜里程计后自动触发一次任务；若设置 `trigger_from_2d_goal: true`，则由每个 2D `/goal` 消息触发一次任务。

当前 `inspection_mission.launch` 默认复用 `click_cave1.launch` 的 MaRSIM 场景
基准：`cave1.yaml`（起飞位置 `(68.5, -29.0, 1.5)`、`cave1.pcd`）和
`click_real_highspeed.yaml` 的规划参数。巡检层使用其副本
`click_inspection.yaml`，并以 `mission_target_marsim_cave1.yaml` 保存 PCD 标定的
首个接近航点与掌子面先验。ROS1 环境中的最小复现实验为：

```bash
roslaunch sfm_planner inspection_mission.launch
rosservice call /inspection/start "{}"
rostopic echo /inspection/status
rostopic echo /inspection/face/debug
rostopic echo /inspection/viewpoints
```

验收时记录 `/inspection/face/debug` 的 `center`、`normal`、`area`、`confidence`
和 `surface_cloud` 点数，并核对状态顺序为 `WAIT_FACE_RESULT → PLAN_VIEWS →
GO_TO_VIEWPOINT → RETURN_HOME → FINISHED`。该配置的 `mock_capture: true` 会让完整
任务在视点到达后继续执行并覆盖写回**仿真专用** target 文件；若要反复使用同一初始
先验，在每轮实验前从版本库还原该文件。

`capture_settle_time_sec` 是每张照片在已到达且航向稳定后、发出相机触发前的悬停时间；
例如改为 `2.0` 即每张照片悬停两秒。每次执行会输出检测到的宽、高、面积、视点数和
预测覆盖率；最后一张照片确认后输出 `Capture workflow complete`（从覆盖计划生成到最后
一张回执的耗时），返回 Home 后再输出 `Mission timing`（包含接近、拍摄和返航的全任务耗时）。
`/inspection/viewpoints` 为 latched `MarkerArray`，在 RViz 中添加同名 MarkerArray 后可查看
掌子面轮廓、绿色视点、橙色拍摄顺序线、黄色视线和编号。

若要验证“到航点后生成视点”但暂时没有掌子面识别器，设置 `navigation_only: false`、`skip_face_detection: true`、`trigger_from_2d_goal: true`。任务会把 2D `/goal` 作为接近航点，读取 `mission_target.yaml` 的 `face_center`、`face_normal` 和 `change_region.width/height` 生成先验视点；该路径不提交新的 `MissionTarget`。

完整巡检模式设置 `trigger_from_2d_goal: false`、`skip_face_detection: false`、`use_target_nav_goal_directly: true`，通过 `/inspection/start` 触发。任务在启动时记录当前位置为 home，然后严格按 `GO_TO_TARGET → WAIT_FACE_RESULT → PLAN_VIEWS → GO_TO_VIEWPOINT/WAIT_CAPTURE_RESULT → RETURN_HOME` 执行；仅在全部拍照成功后才覆盖写回新的目标文件。

每个巡检导航腿的完成条件是位置误差小于 0.1 m，且航向误差小于
`inspection_mission.arrival_yaw_tolerance_rad`。这保证在目标点仍在转向时不会提前开始
识别或触发相机；状态机将等待（必要时重规划）原地转向至目标 yaw。

1. 实现 `MissionTarget` 的加载、原子保存和版本覆盖。
2. 实现 `InspectionMissionPlanner`。
3. 识别和拍照模块先以模拟消息替代。
4. 验证：`target → Home` 的完整状态转换。

### 阶段 B：掌子面识别与目标更新

1. 接入前方 ROI 点云。
2. 完成聚类、平面/PCA、稳定性和置信度判断。
3. 验证：识别成功后仅在拍摄成功时覆盖目标。

### 阶段 C：视点覆盖规划

1. 实现候选视点、地图安全检查和遮挡检查。
2. 实现 `K-coverage` 和视点排序。
3. 验证：输出视点覆盖掌子面且保留未覆盖区域报告。

### 阶段 D：先验地图变化区域

1. 保存旧掌子面 `ChangeRegion`。
2. 下一次任务加载地图时掩膜旧掌子面薄层。
3. 验证：爆破后旧墙面不会阻断到达新作业区的规划。

### 验收主线

```text
给定 target_0
→ 到达 target_0.nav_goal
→ 识别 face_0
→ 生成并完成 face_0 的多视点覆盖
→ target_1 原子覆盖 target_0
→ 返回 Home
→ 重启节点后加载 target_1
→ 下一次任务正确从 target_1 开始
```
