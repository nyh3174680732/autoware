# Autoware Launch 启动与导航流程详解

## 目录
- [1. 系统概述](#1-系统概述)
- [2. 主启动文件 autoware.launch.xml](#2-主启动文件-autowarelaunchxml)
- [3. 各模块启动详解](#3-各模块启动详解)
- [4. 数据流与话题关系](#4-数据流与话题关系)
- [5. 导航流程](#5-导航流程)
- [6. 关键配置文件](#6-关键配置文件)

---

## 1. 系统概述

Autoware 是一个模块化的自动驾驶软件栈，主要由以下核心模块组成：

```
┌─────────────────────────────────────────────────────────────────┐
│                        Autoware 系统架构                         │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐        │
│  │ Vehicle  │  │  System  │  │   Map    │  │ Sensing  │        │
│  │  车辆    │  │  系统    │  │   地图   │  │  感知    │        │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘        │
│                                                                  │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐        │
│  │Localization│ │Perception│  │ Planning │  │ Control  │        │
│  │  定位    │  │  感知    │  │  规划    │  │  控制    │        │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘        │
│                                                                  │
│  ┌──────────┐  ┌──────────┐                                     │
│  │   API    │  │  Tools   │                                     │
│  │  接口    │  │  工具    │                                     │
│  └──────────┘  └──────────┘                                     │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. 主启动文件 autoware.launch.xml

**文件位置**: `src/launcher/autoware_launch/autoware_launch/launch/autoware.launch.xml`

### 2.1 关键参数配置

```xml
<!-- 基本参数 -->
<arg name="map_path" default="src/autoware_map/swjtu"/>           <!-- 地图路径 -->
<arg name="vehicle_model" default="yunle_vehicle"/>                <!-- 车辆模型 -->
<arg name="sensor_model" default="yunle_sensor_kit"/>              <!-- 传感器模型 -->
<arg name="use_sim_time" default="false"/>                         <!-- 仿真时间 -->

<!-- 模块开关 -->
<arg name="launch_vehicle" default="true"/>
<arg name="launch_system" default="true"/>
<arg name="launch_map" default="true"/>
<arg name="launch_sensing" default="true"/>
<arg name="launch_sensing_driver" default="false"/>
<arg name="launch_localization" default="true"/>
<arg name="launch_perception" default="true"/>
<arg name="launch_planning" default="true"/>
<arg name="launch_control" default="true"/>
<arg name="launch_api" default="true"/>
```

### 2.2 启动顺序

```
1. Global Parameters (全局参数)
       ↓
2. Pointcloud Container (点云容器)
       ↓
3. Vehicle (车辆接口)
       ↓
4. System (系统监控)
       ↓
5. Map (地图加载)
       ↓
6. Sensing (传感器)
       ↓
7. Localization (定位)
       ↓
8. Perception (感知)
       ↓
9. Planning (规划)
       ↓
10. Control (控制)
       ↓
11. API (接口服务)
       ↓
12. Tools/RViz (可视化工具)
```

---

## 3. 各模块启动详解

### 3.1 Global Parameters (全局参数)

**启动文件**: `autoware_global_parameter_loader/launch/global_params.launch.py`

加载车辆配置和仿真时间设置到 ROS2 参数服务器。

### 3.2 Pointcloud Container (点云容器)

**启动文件**: `autoware_launch/launch/pointcloud_container.launch.py`

```
创建多线程组件容器，用于运行点云处理相关的 composable nodes
├── container_name: pointcloud_container
└── use_multithread: true
```

### 3.3 Vehicle (车辆模块)

**启动文件**: `tier4_vehicle_launch/launch/vehicle.launch.xml`

```
Vehicle
├── vehicle_interface        # 车辆接口 (可选)
├── raw_vehicle_cmd_converter # 原始车辆命令转换器
└── vehicle_description      # 车辆描述 (URDF, TF)
```

**输出话题**:
- `/vehicle/status/velocity_status` - 车辆速度状态
- `/vehicle/status/steering_status` - 转向状态
- `/vehicle/status/gear_status` - 档位状态
- `/vehicle/status/control_mode` - 控制模式

### 3.4 System (系统模块)

**启动文件**: `tier4_system_launch/launch/system.launch.xml`

```
System
├── system_monitor           # 系统监控 (CPU, GPU, HDD, Memory, Network)
├── diagnostic_graph_aggregator  # 诊断图聚合器
├── mrm_handler             # MRM (Minimum Risk Maneuver) 处理器
├── mrm_comfortable_stop_operator  # 舒适停车操作器
├── mrm_emergency_stop_operator    # 紧急停车操作器
├── component_state_monitor  # 组件状态监控
├── duplicated_node_checker  # 重复节点检查
├── processing_time_checker  # 处理时间检查
└── logging_diag_graph      # 诊断日志
```

**关键话题**:
- `/system/operation_mode/state` - 操作模式状态
- `/system/fail_safe/mrm_state` - MRM 状态
- `/autoware/state` - Autoware 整体状态

### 3.5 Map (地图模块)

**启动文件**: `tier4_map_launch/launch/map.launch.xml`

```
Map
├── pointcloud_map_loader   # 点云地图加载器
│   ├── 支持全量加载
│   └── 支持动态分块加载 (differential_map_loader)
├── lanelet2_map_loader     # Lanelet2 矢量地图加载器
├── map_tf_generator        # 地图 TF 生成器
└── map_projection_loader   # 地图投影加载器
```

**输入文件**:
- `pointcloud_map.pcd` - 点云地图
- `lanelet2_map.osm` - Lanelet2 矢量地图
- `map_projector_info.yaml` - 地图投影信息

**输出话题**:
- `/map/pointcloud_map` - 点云地图
- `/map/vector_map` - 矢量地图
- `/map/pointcloud_map_metadata` - 点云地图元数据

### 3.6 Sensing (传感器模块)

**启动文件**: `tier4_sensing_launch/launch/sensing.launch.xml`

```
Sensing
├── LiDAR
│   ├── velodyne_node_container    # Velodyne 驱动
│   ├── pointcloud_preprocessor    # 点云预处理
│   │   ├── crop_box_filter        # 裁剪滤波
│   │   ├── ring_outlier_filter    # 环形离群点滤波
│   │   └── concatenate_and_time_sync  # 多雷达拼接与时间同步
│   └── distortion_corrector       # 畸变校正
│
├── Camera (可选)
│   └── image_transport            # 图像传输
│
├── GNSS
│   ├── gnss_poser                 # GNSS 位姿估计
│   └── gnss_to_pose               # GNSS 转位姿
│
└── IMU
    └── imu_corrector              # IMU 校正
```

**输出话题**:
- `/sensing/lidar/concatenated/pointcloud` - 拼接后的点云
- `/sensing/gnss/pose_with_covariance` - GNSS 位姿
- `/sensing/imu/imu_data` - IMU 数据

### 3.7 Localization (定位模块)

**启动文件**: `tier4_localization_launch/launch/localization.launch.xml`

```
Localization
├── pose_twist_estimator
│   ├── pose_estimator (位姿估计器)
│   │   ├── NDT Scan Matcher (默认)     # NDT 扫描匹配
│   │   │   ├── pointcloud_preprocessor  # 点云预处理
│   │   │   │   ├── crop_box_filter
│   │   │   │   ├── voxel_grid_downsample
│   │   │   │   └── random_downsample
│   │   │   └── ndt_scan_matcher_core    # NDT 核心算法
│   │   ├── YabLoc (可选)               # 视觉定位
│   │   ├── Eagleye (可选)              # GNSS/IMU 融合定位
│   │   ├── AR Tag Localizer (可选)     # AR 标签定位
│   │   └── LiDAR Marker Localizer (可选)
│   │
│   └── twist_estimator (速度估计器)
│       ├── gyro_odometer (默认)        # 陀螺仪里程计
│       └── eagleye (可选)
│
├── pose_twist_fusion_filter
│   └── EKF Localizer                   # 扩展卡尔曼滤波器
│       ├── 融合 pose_estimator 输出
│       ├── 融合 twist_estimator 输出
│       └── 输出最终位姿
│
├── localization_error_monitor          # 定位误差监控
│
└── util
    ├── pose_initializer                # 位姿初始化器
    └── automatic_pose_initializer      # 自动位姿初始化
```

**NDT Scan Matcher 流程**:
```
输入点云 ─→ crop_box_filter ─→ voxel_grid_downsample ─→ NDT匹配 ─→ 位姿输出
              (裁剪)            (体素降采样)           (与地图匹配)
```

**EKF Localizer 流程**:
```
                  ┌─────────────────┐
pose_estimator ──→│                 │
                  │  EKF Localizer  │──→ /localization/kinematic_state
twist_estimator ─→│   (融合滤波)    │──→ /localization/acceleration
                  └─────────────────┘
```

**关键输出话题**:
- `/localization/kinematic_state` - 运动学状态 (位置+速度)
- `/localization/acceleration` - 加速度
- `/localization/pose_estimator/pose_with_covariance` - NDT 位姿估计

### 3.8 Perception (感知模块)

**启动文件**: `tier4_perception_launch/launch/perception.launch.xml`

```
Perception
├── obstacle_segmentation (障碍物分割)
│   ├── ground_segmentation          # 地面分割
│   │   ├── ray_ground_filter
│   │   └── scan_ground_filter
│   └── pointcloud_based_occupancy_grid_map  # 占用栅格地图
│
├── object_recognition (目标识别)
│   ├── detection (检测)
│   │   ├── centerpoint / transfusion  # 3D目标检测 (深度学习)
│   │   ├── euclidean_cluster          # 欧几里得聚类
│   │   ├── voxel_grid_based_euclidean_cluster
│   │   ├── roi_cluster_fusion         # ROI 融合
│   │   ├── object_lanelet_filter      # Lanelet 过滤
│   │   └── detection_by_tracker       # 跟踪辅助检测
│   │
│   ├── tracking (跟踪)
│   │   ├── multi_object_tracker       # 多目标跟踪
│   │   └── radar_object_tracker       # 雷达目标跟踪
│   │
│   └── prediction (预测)
│       └── map_based_prediction       # 基于地图的轨迹预测
│
└── traffic_light_recognition (交通灯识别, 可选)
    ├── traffic_light_map_based_detector  # 基于地图的检测
    ├── traffic_light_classifier          # 交通灯分类
    └── traffic_light_arbiter            # 交通灯仲裁
```

**感知流程**:
```
点云 ──→ 地面分割 ──→ 目标检测 ──→ 目标跟踪 ──→ 轨迹预测
              ↓
         占用栅格地图
```

**关键输出话题**:
- `/perception/object_recognition/objects` - 识别的目标
- `/perception/obstacle_segmentation/pointcloud` - 障碍物点云
- `/perception/occupancy_grid_map/map` - 占用栅格地图
- `/perception/traffic_light_recognition/traffic_signals` - 交通灯状态

### 3.9 Planning (规划模块)

**启动文件**: `tier4_planning_launch/launch/planning.launch.xml`

```
Planning
├── mission_planning (任务规划)
│   ├── mission_planner              # 任务规划器
│   │   └── 根据目标点生成全局路径 (route)
│   └── goal_pose_visualizer         # 目标点可视化
│
├── scenario_planning (场景规划)
│   ├── scenario_selector            # 场景选择器
│   │   ├── lane_driving             # 车道行驶场景
│   │   └── parking                  # 泊车场景
│   │
│   ├── lane_driving
│   │   ├── behavior_planning (行为规划)
│   │   │   ├── behavior_path_planner    # 行为路径规划器
│   │   │   │   ├── lane_change          # 换道
│   │   │   │   ├── avoidance            # 避障
│   │   │   │   ├── goal_planner         # 目标规划
│   │   │   │   ├── start_planner        # 起步规划
│   │   │   │   └── side_shift           # 侧移
│   │   │   │
│   │   │   └── behavior_velocity_planner # 行为速度规划器
│   │   │       ├── crosswalk            # 人行横道
│   │   │       ├── traffic_light        # 交通灯
│   │   │       ├── intersection         # 交叉口
│   │   │       ├── stop_line            # 停止线
│   │   │       ├── detection_area       # 检测区域
│   │   │       ├── blind_spot           # 盲区
│   │   │       ├── no_stopping_area     # 禁停区
│   │   │       └── run_out              # 冲出检测
│   │   │
│   │   └── motion_planning (运动规划)
│   │       ├── path_smoother            # 路径平滑
│   │       ├── path_optimizer           # 路径优化
│   │       ├── motion_velocity_planner  # 运动速度规划
│   │       │   ├── obstacle_stop        # 障碍物停车
│   │       │   ├── obstacle_slow_down   # 障碍物减速
│   │       │   ├── obstacle_cruise      # 障碍物巡航
│   │       │   └── dynamic_obstacle_stop # 动态障碍物停车
│   │       └── surround_obstacle_checker # 周围障碍物检查
│   │
│   ├── parking
│   │   └── freespace_planner        # 自由空间规划器 (泊车)
│   │
│   ├── velocity_smoother            # 速度平滑器
│   └── external_velocity_limit_selector  # 外部速度限制选择器
│
└── planning_validator               # 规划验证器
    ├── latency_checker              # 延迟检查
    ├── trajectory_checker           # 轨迹检查
    └── collision_checker            # 碰撞检查
```

**规划流程**:
```
目标点 ──→ mission_planner ──→ route (全局路径)
                                  ↓
                         scenario_selector
                          ↙          ↘
              lane_driving            parking
                  ↓                      ↓
          behavior_planning       freespace_planner
                  ↓
          motion_planning
                  ↓
          velocity_smoother
                  ↓
          planning_validator
                  ↓
              trajectory
```

**关键话题**:
- `/planning/mission_planning/route` - 全局路线
- `/planning/scenario_planning/lane_driving/trajectory` - 车道行驶轨迹
- `/planning/scenario_planning/parking/trajectory` - 泊车轨迹
- `/planning/trajectory` - 最终规划轨迹

### 3.10 Control (控制模块)

**启动文件**: `tier4_control_launch/launch/control.launch.xml`

```
Control
├── trajectory_follower (轨迹跟踪)
│   ├── lateral_controller (横向控制)
│   │   └── MPC (模型预测控制, 默认)
│   │       └── 控制转向角
│   │
│   └── longitudinal_controller (纵向控制)
│       └── PID (PID 控制, 默认)
│           └── 控制加速度/制动
│
├── shift_decider                    # 档位决策
│   └── 根据控制命令决定档位
│
├── vehicle_cmd_gate                 # 车辆命令门
│   ├── 接收自动/手动/紧急命令
│   ├── 命令仲裁与切换
│   └── 输出最终控制命令
│
├── operation_mode_transition_manager # 操作模式转换管理器
│   └── 管理 Manual/Autonomous/Remote 模式切换
│
├── control_check_container (控制检查)
│   ├── lane_departure_checker       # 车道偏离检查
│   ├── control_validator            # 控制验证
│   ├── autonomous_emergency_braking (AEB)  # 自动紧急制动
│   ├── collision_detector           # 碰撞检测
│   └── obstacle_collision_checker   # 障碍物碰撞检查
│
└── external_cmd_selector            # 外部命令选择器
    └── 选择外部控制命令源
```

**控制流程**:
```
/planning/trajectory
        ↓
 trajectory_follower
   ├── lateral_controller (MPC)
   │       ↓
   │   steering_cmd
   │
   └── longitudinal_controller (PID)
           ↓
       acceleration_cmd
           ↓
    shift_decider ──→ gear_cmd
           ↓
    vehicle_cmd_gate
           ↓
    /control/command/control_cmd
           ↓
      Vehicle Interface
```

**关键话题**:
- `/control/trajectory_follower/control_cmd` - 轨迹跟踪控制命令
- `/control/command/control_cmd` - 最终控制命令
- `/control/command/gear_cmd` - 档位命令
- `/control/command/turn_indicators_cmd` - 转向灯命令

### 3.11 API (接口模块)

**启动文件**: `tier4_autoware_api_launch/launch/autoware_api.launch.xml`

提供外部接口服务:
- 车辆状态 API
- 路线设置 API
- 自动驾驶启停 API

---

## 4. 数据流与话题关系

### 4.1 主要数据流

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              Autoware 数据流                                 │
└─────────────────────────────────────────────────────────────────────────────┘

[Sensors]                    [Localization]              [Perception]
    │                             │                           │
    │  /sensing/lidar/           │                           │
    │  concatenated/pointcloud   │                           │
    ├─────────────────────────────→ NDT Scan Matcher         │
    │                             │      ↓                    │
    │  /sensing/imu/imu_data     │  EKF Localizer            │
    ├─────────────────────────────→      ↓                    │
    │                             │ /localization/            │
    │  /sensing/gnss/pose        │ kinematic_state           │
    ├─────────────────────────────→                           │
    │                             └────────────┬──────────────┤
    │                                          │              │
    │  /sensing/lidar/                         │              │
    │  concatenated/pointcloud                 │              │
    ├──────────────────────────────────────────┼──────────────→ Object Detection
    │                                          │              │      ↓
    │                                          │              │ Object Tracking
    │                                          │              │      ↓
    │                                          │              │ Prediction
    │                                          │              │      ↓
    │                                          │              │ /perception/
    │                                          │              │ object_recognition/
    │                                          │              │ objects
    │                                          │              │
    └──────────────────────────────────────────┼──────────────┘
                                               │
                                               ↓
┌──────────────────────────────────────────────────────────────────────────────┐
│                                [Planning]                                     │
│                                                                               │
│  Goal Pose ──→ Mission Planner ──→ /planning/mission_planning/route          │
│                                              ↓                                │
│                              Behavior Path Planner                            │
│                                              ↓                                │
│                              Behavior Velocity Planner                        │
│                                              ↓                                │
│                              Motion Velocity Planner                          │
│                                              ↓                                │
│                              Velocity Smoother                                │
│                                              ↓                                │
│                              /planning/trajectory                             │
└──────────────────────────────────┬───────────────────────────────────────────┘
                                   │
                                   ↓
┌──────────────────────────────────────────────────────────────────────────────┐
│                                [Control]                                      │
│                                                                               │
│  /planning/trajectory ──→ Trajectory Follower                                 │
│                                    ├── Lateral Controller (MPC)               │
│                                    └── Longitudinal Controller (PID)          │
│                                              ↓                                │
│                               Vehicle Cmd Gate                                │
│                                              ↓                                │
│                           /control/command/control_cmd                        │
└──────────────────────────────────┬───────────────────────────────────────────┘
                                   │
                                   ↓
┌──────────────────────────────────────────────────────────────────────────────┐
│                               [Vehicle]                                       │
│                                                                               │
│  /control/command/control_cmd ──→ Vehicle Interface ──→ CAN Bus / 实际车辆    │
│                                                                               │
└──────────────────────────────────────────────────────────────────────────────┘
```

### 4.2 关键话题列表

| 话题 | 类型 | 描述 |
|-----|------|------|
| `/sensing/lidar/concatenated/pointcloud` | PointCloud2 | 拼接后的激光雷达点云 |
| `/sensing/imu/imu_data` | Imu | IMU 数据 |
| `/sensing/gnss/pose_with_covariance` | PoseWithCovarianceStamped | GNSS 位姿 |
| `/map/pointcloud_map` | PointCloud2 | 点云地图 |
| `/map/vector_map` | HADMapBin | 矢量地图 |
| `/localization/kinematic_state` | Odometry | 定位结果 (位置+速度) |
| `/localization/acceleration` | AccelWithCovarianceStamped | 加速度 |
| `/perception/object_recognition/objects` | PredictedObjects | 感知目标 |
| `/planning/mission_planning/route` | LaneletRoute | 全局路线 |
| `/planning/trajectory` | Trajectory | 规划轨迹 |
| `/control/command/control_cmd` | Control | 控制命令 |
| `/system/operation_mode/state` | OperationModeState | 操作模式 |

---

## 5. 导航流程

### 5.1 系统启动流程

```
1. 启动 Autoware
   ros2 launch autoware_launch autoware.launch.xml

2. 等待所有模块初始化完成
   - Map 加载完成
   - Sensing 数据正常
   - Localization 初始化

3. 设置初始位姿 (如果需要)
   - 在 RViz 中使用 "2D Pose Estimate"
   - 或通过 /initialpose 话题

4. 等待定位收敛
   - NDT 分数达到阈值
   - EKF 状态稳定
```

### 5.2 导航执行流程

```
┌────────────────────────────────────────────────────────────────┐
│                        导航执行流程                             │
└────────────────────────────────────────────────────────────────┘

1. 设置目标点
   ┌─────────────────┐
   │ 用户在 RViz 中  │
   │ 点击 "2D Goal" │
   └────────┬────────┘
            │
            ↓
   /planning/mission_planning/goal
            │
            ↓
2. 任务规划
   ┌─────────────────┐
   │ Mission Planner │
   │ 搜索全局路径    │
   └────────┬────────┘
            │
            ↓
   /planning/mission_planning/route
            │
            ↓
3. 行为规划
   ┌─────────────────────────────────┐
   │ Behavior Path Planner           │
   │ - 换道决策                      │
   │ - 避障决策                      │
   │ - 路口行为                      │
   └────────┬────────────────────────┘
            │
            ↓
   ┌─────────────────────────────────┐
   │ Behavior Velocity Planner       │
   │ - 交通灯响应                    │
   │ - 人行横道减速                  │
   │ - 停止线停车                    │
   └────────┬────────────────────────┘
            │
            ↓
4. 运动规划
   ┌─────────────────────────────────┐
   │ Motion Velocity Planner         │
   │ - 障碍物避让                    │
   │ - 速度平滑                      │
   └────────┬────────────────────────┘
            │
            ↓
   /planning/trajectory
            │
            ↓
5. 轨迹跟踪
   ┌─────────────────────────────────┐
   │ Trajectory Follower             │
   │ - MPC 横向控制                  │
   │ - PID 纵向控制                  │
   └────────┬────────────────────────┘
            │
            ↓
6. 命令输出
   ┌─────────────────────────────────┐
   │ Vehicle Cmd Gate                │
   │ - 命令仲裁                      │
   │ - 安全检查                      │
   └────────┬────────────────────────┘
            │
            ↓
   /control/command/control_cmd
            │
            ↓
7. 车辆执行
   ┌─────────────────────────────────┐
   │ Vehicle Interface               │
   │ - 发送到车辆 CAN                │
   └─────────────────────────────────┘
```

### 5.3 操作模式切换

```
操作模式状态机:

    ┌──────────┐     engage     ┌──────────────┐
    │  MANUAL  │ ─────────────→ │  AUTONOMOUS  │
    │  手动    │ ←───────────── │   自动       │
    └──────────┘   disengage    └──────────────┘
         ↑                             │
         │                             │
         │         emergency           │
         └─────────────────────────────┘
                      │
                      ↓
               ┌──────────────┐
               │  EMERGENCY   │
               │    紧急      │
               └──────────────┘
```

**切换条件**:
- Manual → Autonomous:
  - 定位正常 (localization OK)
  - 路线已设置 (route set)
  - 无紧急状态 (no emergency)
  - 用户 engage

- Autonomous → Emergency:
  - 定位丢失
  - 传感器故障
  - 系统错误

---

## 6. 关键配置文件

### 6.1 定位配置

**NDT Scan Matcher**:
`config/localization/ndt_scan_matcher/ndt_scan_matcher.param.yaml`

```yaml
ndt:
  resolution: 2.0                    # 体素分辨率
  max_iterations: 30                 # 最大迭代次数

score_estimation:
  converged_param_type: 1            # 使用 NVTL 评分
  converged_param_nearest_voxel_transformation_likelihood: 2.3  # 分数阈值
```

**EKF Localizer**:
`config/localization/ekf_localizer.param.yaml`

```yaml
pose_measurement:
  pose_gate_dist: 49.5               # 马氏距离阈值
```

### 6.2 感知配置

**CenterPoint (3D目标检测)**:
`config/perception/object_recognition/detection/lidar_model/centerpoint.param.yaml`

**Multi Object Tracker**:
`config/perception/object_recognition/tracking/multi_object_tracker/`

### 6.3 规划配置

**Behavior Path Planner**:
`config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/`

**Behavior Velocity Planner**:
`config/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/`

### 6.4 控制配置

**MPC (横向控制)**:
`config/control/trajectory_follower/lateral/mpc.param.yaml`

**PID (纵向控制)**:
`config/control/trajectory_follower/longitudinal/pid.param.yaml`

**Vehicle Cmd Gate**:
`config/control/vehicle_cmd_gate/vehicle_cmd_gate.param.yaml`

---

## 附录: 常见问题排查

### A.1 定位问题

**症状**: `Score is below the threshold`

**原因**: NDT 扫描匹配分数不足

**解决**:
1. 检查点云地图质量
2. 调整 `converged_param_nearest_voxel_transformation_likelihood` 阈值
3. 检查初始位姿是否准确

### A.2 Emergency 状态

**症状**: `Emergency!` 警告

**原因**:
- 定位状态异常
- 传感器数据中断
- 系统诊断失败

**解决**:
1. 检查 `/system/operation_mode/state`
2. 检查 diagnostic graph 日志
3. 验证传感器数据正常

### A.3 轨迹跟踪问题

**症状**: 车辆行驶不平稳

**解决**:
1. 调整 MPC 参数
2. 调整 PID 增益
3. 检查 `/planning/trajectory` 是否平滑

---

*文档生成时间: 2026-01-07*
*基于 Autoware 代码库分析*




  第27行: use_sim_time 默认为 false

  <arg name="use_sim_time" default="false" description="use_sim_time"/> <!--e2m defult=true-->

  | 参数值       | 时间模式 | 适用场景               |
  |--------------|----------|------------------------|
  | false (当前) | 真实时间 | 真车运行、实时传感器   |
  | true         | 仿真时间 | Gazebo仿真、rosbag回放 |

  ---
  如需切换到仿真时间

  启动时添加参数：

  ros2 launch autoware_launch autoware.launch.xml use_sim_time:=true

  注意: 使用仿真时间时，需要有时钟源发布 /clock 话题（如 Gazebo 或 ros2 bag play --clock）。



● 雷达和IMU标定配置位置

  主要配置文件

  在你的 yunle_sensor_kit 中，标定关系定义在以下文件：

  | 文件                                                                                                                            | 作用                                           |
  |---------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------|
  | src/launcher/autoware_launch/sensor_kit/yunle_sensor_kit_launch/yunle_sensor_kit_description/config/sensor_kit_calibration.yaml | 定义LiDAR和IMU相对于sensor_kit_base_link的位置 |
  | src/launcher/autoware_launch/sensor_kit/yunle_sensor_kit_launch/yunle_sensor_kit_description/config/sensors_calibration.yaml    | 定义sensor_kit_base_link相对于base_link的位置  |

  当前坐标关系

  base_link
      │  (z: 1.0m)
  sensor_kit_base_link
      ├── velodyne_top_base_link  (x: 0.6, y: 0.0, z: 0.16)
      ├── velodyne_left_base_link (x: 0.6, y: 0.0, z: 0.16)
      ├── velodyne_right_base_link(x: 0.6, y: 0.0, z: 0.16)
      └── tamagawa/imu_link       (x: 0.6, y: 0.0, z: 0.07)

  ---
  如何标定雷达和IMU

  方法一：手动测量（简单场景）

  1. 测量物理位置：用卷尺测量LiDAR和IMU在车辆上的安装位置
  2. 编辑配置文件：修改 sensor_kit_calibration.yaml
  sensor_kit_base_link:
    velodyne_top_base_link:
      x: <前后距离>
      y: <左右距离>
      z: <高度>
      roll: <横滚角>
      pitch: <俯仰角>
      yaw: <航向角>
    tamagawa/imu_link:
      x: <前后距离>
      y: <左右距离>
      z: <高度>
      roll: 0.0
      pitch: 0.0
      yaw: 0.0

  方法二：使用标定工具（精确标定）

  Autoware提供了一些标定工具，位于：
  - src/tools/common/tier4_calibration_msgs/ - 标定消息
  - src/tools/common/tier4_calibration_rviz_plugin/ - RViz标定插件

  对于LiDAR-IMU外参标定，常用方法：

  1. 基于运动的标定：采集车辆运动数据，通过优化算法求解外参
  2. 使用开源工具：如 https://github.com/APRIL-ZJU/lidar_IMU_calib 或 https://github.com/hku-mars/LiDAR_IMU_Init

  验证标定结果

  修改配置后重启Autoware，在RViz中检查：
  1. TF树是否正确显示各传感器坐标系
  2. 点云是否与IMU数据对齐
  3. 定位模块是否工作正常

  需要我帮你查看或修改具体的标定参数吗？
