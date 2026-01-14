# 单线雷达 OGM (Occupancy Grid Map) 功能修改文档

## 概述

本次修改实现了直接使用单线雷达 (Single-line LiDAR) 的 LaserScan 数据生成 Occupancy Grid Map 的功能，无需经过点云转换步骤。

### 修改日期
2026-01-13

### 修改目的
- 支持直接使用单线雷达的 `sensor_msgs/msg/LaserScan` 消息
- 跳过默认的 `pointcloud_to_laserscan` 转换节点
- 简化单线雷达的 OGM 生成流程

---

## 原始流程 vs 修改后流程

### 原始流程 (默认)
```
3D 点云 (PointCloud2)
    ↓
pointcloud_to_laserscan 节点
    ↓
LaserScan (2D)
    ↓
LaserscanBasedOccupancyGridMapNode
    ↓
Occupancy Grid Map
```

### 修改后流程 (use_raw_laserscan=true)
```
单线雷达 LaserScan (/sensing/lidar/front/scan)
    ↓
直接输入 LaserscanBasedOccupancyGridMapNode
    ↓
Occupancy Grid Map
```

---

## 修改的文件清单

| 序号 | 文件路径 | 修改类型 |
|------|---------|---------|
| 1 | `src/universe/autoware_universe/perception/autoware_probabilistic_occupancy_grid_map/launch/laserscan_based_occupancy_grid_map.launch.py` | 核心逻辑修改 |
| 2 | `src/universe/autoware_universe/launch/tier4_perception_launch/launch/occupancy_grid_map/probabilistic_occupancy_grid_map.launch.xml` | 参数传递 |
| 3 | `src/universe/autoware_universe/launch/tier4_perception_launch/launch/perception.launch.xml` | 参数传递 |
| 4 | `src/launcher/autoware_launch/autoware_launch/launch/components/tier4_perception_component.launch.xml` | 参数声明与传递 |

---

## 详细修改内容

### 1. laserscan_based_occupancy_grid_map.launch.py

**文件路径:**
```
src/universe/autoware_universe/perception/autoware_probabilistic_occupancy_grid_map/launch/laserscan_based_occupancy_grid_map.launch.py
```

**修改内容:**

#### 1.1 修改 `launch_setup` 函数

**原始代码:**
```python
def launch_setup(context, *args, **kwargs):
    # load parameter files
    param_file = LaunchConfiguration("param_file").perform(context)
    with open(param_file, "r") as f:
        laserscan_based_occupancy_grid_map_node_params = yaml.safe_load(f)["/**"]["ros__parameters"]

    updater_param_file = LaunchConfiguration("updater_param_file").perform(context)
    with open(updater_param_file, "r") as f:
        occupancy_grid_map_updater_params = yaml.safe_load(f)["/**"]["ros__parameters"]

    composable_nodes = [
        ComposableNode(
            package="pointcloud_to_laserscan",
            plugin="pointcloud_to_laserscan::PointCloudToLaserScanNode",
            name="pointcloud_to_laserscan_node",
            remappings=[
                (
                    "~/input/pointcloud",
                    LaunchConfiguration("input/obstacle_pointcloud"),
                ),
                (
                    "~/output/laserscan",
                    LaunchConfiguration("output/laserscan"),
                ),
                (
                    "~/output/pointcloud",
                    LaunchConfiguration("output/pointcloud"),
                ),
                ("~/output/ray", LaunchConfiguration("output/ray")),
                ("~/output/stixel", LaunchConfiguration("output/stixel")),
            ],
            parameters=[
                {
                    "target_frame": laserscan_based_occupancy_grid_map_node_params[
                        "scan_origin_frame"
                    ],
                    "transform_tolerance": 0.01,
                    "min_height": 0.0,
                    "max_height": 2.0,
                    "angle_min": -3.141592,
                    "angle_max": 3.141592,
                    "angle_increment": 0.00349065850,
                    "scan_time": 0.0,
                    "range_min": 1.0,
                    "range_max": 200.0,
                    "use_inf": True,
                    "inf_epsilon": 1.0,
                    "concurrency_level": 1,
                }
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        ),
        ComposableNode(
            package="autoware_probabilistic_occupancy_grid_map",
            plugin="autoware::occupancy_grid_map::LaserscanBasedOccupancyGridMapNode",
            name="occupancy_grid_map_node",
            remappings=[
                ("~/input/laserscan", LaunchConfiguration("output/laserscan")),
                (
                    "~/input/obstacle_pointcloud",
                    LaunchConfiguration("input/obstacle_pointcloud"),
                ),
                (
                    "~/input/raw_pointcloud",
                    LaunchConfiguration("input/raw_pointcloud"),
                ),
                ("~/output/occupancy_grid_map", LaunchConfiguration("output")),
            ],
            parameters=[
                laserscan_based_occupancy_grid_map_node_params,
                occupancy_grid_map_updater_params,
                {
                    "input_obstacle_pointcloud": LaunchConfiguration("input_obstacle_pointcloud"),
                    "input_obstacle_and_raw_pointcloud": LaunchConfiguration(
                        "input_obstacle_and_raw_pointcloud"
                    ),
                    "updater_type": LaunchConfiguration("updater_type"),
                },
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        ),
    ]
```

**修改后代码:**
```python
def launch_setup(context, *args, **kwargs):
    # load parameter files
    param_file = LaunchConfiguration("param_file").perform(context)
    with open(param_file, "r") as f:
        laserscan_based_occupancy_grid_map_node_params = yaml.safe_load(f)["/**"]["ros__parameters"]

    updater_param_file = LaunchConfiguration("updater_param_file").perform(context)
    with open(updater_param_file, "r") as f:
        occupancy_grid_map_updater_params = yaml.safe_load(f)["/**"]["ros__parameters"]

    # Check if using raw laserscan directly (e.g., from single-line lidar)
    use_raw_laserscan = LaunchConfiguration("use_raw_laserscan").perform(context).lower() == "true"

    composable_nodes = []

    # Only add pointcloud_to_laserscan node if not using raw laserscan
    if not use_raw_laserscan:
        composable_nodes.append(
            ComposableNode(
                package="pointcloud_to_laserscan",
                plugin="pointcloud_to_laserscan::PointCloudToLaserScanNode",
                name="pointcloud_to_laserscan_node",
                remappings=[
                    (
                        "~/input/pointcloud",
                        LaunchConfiguration("input/obstacle_pointcloud"),
                    ),
                    (
                        "~/output/laserscan",
                        LaunchConfiguration("output/laserscan"),
                    ),
                    (
                        "~/output/pointcloud",
                        LaunchConfiguration("output/pointcloud"),
                    ),
                    ("~/output/ray", LaunchConfiguration("output/ray")),
                    ("~/output/stixel", LaunchConfiguration("output/stixel")),
                ],
                parameters=[
                    {
                        "target_frame": laserscan_based_occupancy_grid_map_node_params[
                            "scan_origin_frame"
                        ],
                        "transform_tolerance": 0.01,
                        "min_height": 0.0,
                        "max_height": 2.0,
                        "angle_min": -3.141592,
                        "angle_max": 3.141592,
                        "angle_increment": 0.00349065850,
                        "scan_time": 0.0,
                        "range_min": 1.0,
                        "range_max": 200.0,
                        "use_inf": True,
                        "inf_epsilon": 1.0,
                        "concurrency_level": 1,
                    }
                ],
                extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
            )
        )

    # Determine laserscan input topic based on use_raw_laserscan
    if use_raw_laserscan:
        laserscan_input = LaunchConfiguration("input/laserscan")
    else:
        laserscan_input = LaunchConfiguration("output/laserscan")

    composable_nodes.append(
        ComposableNode(
            package="autoware_probabilistic_occupancy_grid_map",
            plugin="autoware::occupancy_grid_map::LaserscanBasedOccupancyGridMapNode",
            name="occupancy_grid_map_node",
            remappings=[
                ("~/input/laserscan", laserscan_input),
                (
                    "~/input/obstacle_pointcloud",
                    LaunchConfiguration("input/obstacle_pointcloud"),
                ),
                (
                    "~/input/raw_pointcloud",
                    LaunchConfiguration("input/raw_pointcloud"),
                ),
                ("~/output/occupancy_grid_map", LaunchConfiguration("output")),
            ],
            parameters=[
                laserscan_based_occupancy_grid_map_node_params,
                occupancy_grid_map_updater_params,
                {
                    "input_obstacle_pointcloud": LaunchConfiguration("input_obstacle_pointcloud"),
                    "input_obstacle_and_raw_pointcloud": LaunchConfiguration(
                        "input_obstacle_and_raw_pointcloud"
                    ),
                    "updater_type": LaunchConfiguration("updater_type"),
                },
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        )
    )
```

#### 1.2 添加新的 Launch 参数声明

**原始代码:**
```python
    return LaunchDescription(
        [
            add_launch_arg("use_multithread", "false"),
            add_launch_arg("use_intra_process", "false"),
            add_launch_arg("input/obstacle_pointcloud", "no_ground/oneshot/pointcloud"),
            add_launch_arg("input/raw_pointcloud", "concatenated/pointcloud"),
            add_launch_arg("output", "occupancy_grid"),
            add_launch_arg("output/laserscan", "virtual_scan/laserscan"),
```

**修改后代码:**
```python
    return LaunchDescription(
        [
            add_launch_arg("use_multithread", "false"),
            add_launch_arg("use_intra_process", "false"),
            add_launch_arg("use_raw_laserscan", "false"),  # Set to true to use raw laserscan from single-line lidar
            add_launch_arg("input/laserscan", "/sensing/lidar/front/scan"),  # Raw laserscan topic (used when use_raw_laserscan is true)
            add_launch_arg("input/obstacle_pointcloud", "no_ground/oneshot/pointcloud"),
            add_launch_arg("input/raw_pointcloud", "concatenated/pointcloud"),
            add_launch_arg("output", "occupancy_grid"),
            add_launch_arg("output/laserscan", "virtual_scan/laserscan"),
```

---

### 2. probabilistic_occupancy_grid_map.launch.xml

**文件路径:**
```
src/universe/autoware_universe/launch/tier4_perception_launch/launch/occupancy_grid_map/probabilistic_occupancy_grid_map.launch.xml
```

**修改内容:**

#### 2.1 添加参数声明

在 `<arg name="use_pointcloud_container" .../>` 后添加:

```xml
  <!-- laserscan based OGM parameters for single-line lidar -->
  <arg name="use_raw_laserscan" default="false" description="use raw laserscan from single-line lidar instead of converting from pointcloud"/>
  <arg name="input/laserscan" default="/sensing/lidar/front/scan" description="raw laserscan topic (used when use_raw_laserscan is true)"/>
```

#### 2.2 在 laserscan_based include 中传递参数

**原始代码:**
```xml
  <!--laserscan based method-->
  <group if="$(eval &quot;'$(var occupancy_grid_map_method)'=='laserscan_based_occupancy_grid_map'&quot;)">
    <include file="$(find-pkg-share autoware_probabilistic_occupancy_grid_map)/launch/laserscan_based_occupancy_grid_map.launch.py">
      <arg name="input/obstacle_pointcloud" value="$(var input/obstacle_pointcloud)"/>
      <arg name="input/raw_pointcloud" value="$(var input/raw_pointcloud)"/>
      <arg name="output" value="$(var output)"/>
      <arg name="use_intra_process" value="$(var use_intra_process)"/>
      <arg name="use_multithread" value="$(var use_multithread)"/>
      <arg name="use_pointcloud_container" value="$(var use_pointcloud_container)"/>
      <arg name="pointcloud_container_name" value="$(var pointcloud_container_name)"/>
      <arg name="param_file" value="$(var occupancy_grid_map_param_path)"/>
      <arg name="updater_type" value="$(var occupancy_grid_map_updater)"/>
      <arg name="updater_param_file" value="$(var occupancy_grid_map_updater_param_path)"/>
      <arg name="input_obstacle_pointcloud" value="$(var input_obstacle_pointcloud)"/>
      <arg name="input_obstacle_and_raw_pointcloud" value="$(var input_obstacle_and_raw_pointcloud)"/>
    </include>
  </group>
```

**修改后代码:**
```xml
  <!--laserscan based method-->
  <group if="$(eval &quot;'$(var occupancy_grid_map_method)'=='laserscan_based_occupancy_grid_map'&quot;)">
    <include file="$(find-pkg-share autoware_probabilistic_occupancy_grid_map)/launch/laserscan_based_occupancy_grid_map.launch.py">
      <arg name="input/obstacle_pointcloud" value="$(var input/obstacle_pointcloud)"/>
      <arg name="input/raw_pointcloud" value="$(var input/raw_pointcloud)"/>
      <arg name="output" value="$(var output)"/>
      <arg name="use_intra_process" value="$(var use_intra_process)"/>
      <arg name="use_multithread" value="$(var use_multithread)"/>
      <arg name="use_pointcloud_container" value="$(var use_pointcloud_container)"/>
      <arg name="pointcloud_container_name" value="$(var pointcloud_container_name)"/>
      <arg name="param_file" value="$(var occupancy_grid_map_param_path)"/>
      <arg name="updater_type" value="$(var occupancy_grid_map_updater)"/>
      <arg name="updater_param_file" value="$(var occupancy_grid_map_updater_param_path)"/>
      <arg name="input_obstacle_pointcloud" value="$(var input_obstacle_pointcloud)"/>
      <arg name="input_obstacle_and_raw_pointcloud" value="$(var input_obstacle_and_raw_pointcloud)"/>
      <arg name="use_raw_laserscan" value="$(var use_raw_laserscan)"/>
      <arg name="input/laserscan" value="$(var input/laserscan)"/>
    </include>
  </group>
```

---

### 3. perception.launch.xml

**文件路径:**
```
src/universe/autoware_universe/launch/tier4_perception_launch/launch/perception.launch.xml
```

**修改内容:**

#### 3.1 添加参数声明

在 `<arg name="occupancy_grid_map_updater_param_path"/>` 后添加:

```xml
  <arg name="use_raw_laserscan" default="false" description="use raw laserscan from single-line lidar"/>
  <arg name="input/laserscan" default="/sensing/lidar/front/scan" description="raw laserscan topic"/>
```

#### 3.2 在 include 中传递参数

**原始代码:**
```xml
      <include file="$(find-pkg-share tier4_perception_launch)/launch/occupancy_grid_map/probabilistic_occupancy_grid_map.launch.xml">
        <arg name="input/obstacle_pointcloud" value="$(var unfiltered_obstacle_pointcloud)"/>
        <arg name="input/raw_pointcloud" value="$(var perception_pointcloud)"/>
        <arg name="output" value="/perception/occupancy_grid_map/map"/>
        <arg name="use_intra_process" value="true"/>
        <arg name="use_multithread" value="true"/>
        <arg name="pointcloud_container_name" value="$(var pointcloud_container_name)"/>
        <arg name="occupancy_grid_map_method" value="$(var occupancy_grid_map_method)"/>
        <arg name="occupancy_grid_map_param_path" value="$(var occupancy_grid_map_param_path)"/>
        <arg name="occupancy_grid_map_updater" value="$(var occupancy_grid_map_updater)"/>
        <arg name="occupancy_grid_map_updater_param_path" value="$(var occupancy_grid_map_updater_param_path)"/>
      </include>
```

**修改后代码:**
```xml
      <include file="$(find-pkg-share tier4_perception_launch)/launch/occupancy_grid_map/probabilistic_occupancy_grid_map.launch.xml">
        <arg name="input/obstacle_pointcloud" value="$(var unfiltered_obstacle_pointcloud)"/>
        <arg name="input/raw_pointcloud" value="$(var perception_pointcloud)"/>
        <arg name="output" value="/perception/occupancy_grid_map/map"/>
        <arg name="use_intra_process" value="true"/>
        <arg name="use_multithread" value="true"/>
        <arg name="pointcloud_container_name" value="$(var pointcloud_container_name)"/>
        <arg name="occupancy_grid_map_method" value="$(var occupancy_grid_map_method)"/>
        <arg name="occupancy_grid_map_param_path" value="$(var occupancy_grid_map_param_path)"/>
        <arg name="occupancy_grid_map_updater" value="$(var occupancy_grid_map_updater)"/>
        <arg name="occupancy_grid_map_updater_param_path" value="$(var occupancy_grid_map_updater_param_path)"/>
        <arg name="use_raw_laserscan" value="$(var use_raw_laserscan)"/>
        <arg name="input/laserscan" value="$(var input/laserscan)"/>
      </include>
```

---

### 4. tier4_perception_component.launch.xml

**文件路径:**
```
src/launcher/autoware_launch/autoware_launch/launch/components/tier4_perception_component.launch.xml
```

**修改内容:**

#### 4.1 添加参数声明

在 `<arg name="occupancy_grid_map_updater" .../>` 后添加:

```xml
  <arg name="use_raw_laserscan" default="false" description="use raw laserscan from single-line lidar for OGM"/>
  <arg name="input/laserscan" default="/sensing/lidar/front/scan" description="raw laserscan topic for OGM"/>
```

#### 4.2 在 include 中传递参数

**原始代码:**
```xml
    <arg name="pointcloud_container_name" value="$(var pointcloud_container_name)"/>
    <arg name="occupancy_grid_map_method" value="$(var occupancy_grid_map_method)_occupancy_grid_map"/>
    <arg name="occupancy_grid_map_updater" value="$(var occupancy_grid_map_updater)"/>
```

**修改后代码:**
```xml
    <arg name="pointcloud_container_name" value="$(var pointcloud_container_name)"/>
    <arg name="occupancy_grid_map_method" value="$(var occupancy_grid_map_method)_occupancy_grid_map"/>
    <arg name="occupancy_grid_map_updater" value="$(var occupancy_grid_map_updater)"/>
    <arg name="use_raw_laserscan" value="$(var use_raw_laserscan)"/>
    <arg name="input/laserscan" value="$(var input/laserscan)"/>
```

---

## 新增参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `use_raw_laserscan` | bool | `false` | 是否直接使用单线雷达的 LaserScan 数据。设为 `true` 时跳过 pointcloud_to_laserscan 转换 |
| `input/laserscan` | string | `/sensing/lidar/front/scan` | 单线雷达的 LaserScan topic 名称 (仅当 `use_raw_laserscan=true` 时有效) |

---

## 使用方法

### 基本启动命令

```bash
ros2 launch autoware_launch autoware.launch.xml \
    vehicle_model:=yunle_vehicle \
    sensor_model:=yunle_sensor_kit \
    map_path:=src/autoware_map/gazebo_map/ \
    use_sim_time:=true \
    occupancy_grid_map_method:=laserscan_based \
    use_raw_laserscan:=true \
    input/laserscan:=/sensing/lidar/front/scan
```

### 参数说明

| 参数 | 值 | 说明 |
|------|-----|------|
| `occupancy_grid_map_method` | `laserscan_based` | 使用 laserscan 方式生成 OGM |
| `use_raw_laserscan` | `true` | 启用直接 LaserScan 输入模式 |
| `input/laserscan` | `/sensing/lidar/front/scan` | 你的单线雷达 topic |

### 自定义 LaserScan Topic

如果你的单线雷达发布的 topic 名称不同，修改 `input/laserscan` 参数即可:

```bash
input/laserscan:=/your/custom/scan/topic
```

---

## 注意事项

1. **TF 变换**: 确保单线雷达的 frame_id 有正确的 TF 变换到 `base_link` 和 `map`

2. **scan_origin_frame 配置**: 可能需要修改参数文件中的 `scan_origin_frame`:
   ```
   src/launcher/autoware_launch/autoware_launch/config/perception/occupancy_grid_map/laserscan_based_occupancy_grid_map.param.yaml
   ```
   将 `scan_origin_frame` 设置为你的单线雷达 frame_id

3. **障碍物点云**: 即使使用 raw laserscan，OGM 节点仍然需要 `obstacle_pointcloud` 和 `raw_pointcloud` 输入来进行障碍物标记。如果没有多线雷达，可以设置:
   ```
   input_obstacle_pointcloud:=false
   input_obstacle_and_raw_pointcloud:=false
   ```

---

## 参考资料

- OGM 节点源码: `src/universe/autoware_universe/perception/autoware_probabilistic_occupancy_grid_map/`
- LaserScan 消息定义: `sensor_msgs/msg/LaserScan`
- Autoware 感知模块文档: https://autowarefoundation.github.io/autoware-documentation/
