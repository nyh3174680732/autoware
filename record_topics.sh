#!/bin/bash

# 设置输出目录
OUTPUT_DIR=~/Desktop/autoware/src/autoware_map
OUTPUT_FILE=gazebo_rosbag

# 创建输出目录（如果不存在）
mkdir -p $OUTPUT_DIR

# 录制所有主题
ros2 bag record -o $OUTPUT_DIR/$OUTPUT_FILE \
/clock \
/cmd_vel \
/control/command/control_cmd \
/distance \
/odom \
/parameter_events \
/performance_metrics \
/rosout \
/sensing/camera/traffic_light/camera_info \
/sensing/camera/traffic_light/depth/camera_info \
/sensing/camera/traffic_light/depth/image_raw \
/sensing/camera/traffic_light/depth/image_raw/compressed \
/sensing/camera/traffic_light/depth/image_raw/compressedDepth \
/sensing/camera/traffic_light/depth/image_raw/theora \
/sensing/camera/traffic_light/image_raw \
/sensing/camera/traffic_light/image_raw/compressed \
/sensing/camera/traffic_light/image_raw/compressedDepth \
/sensing/camera/traffic_light/image_raw/theora \
/sensing/camera/traffic_light/points \
/sensing/gnss/pose_with_covariance \
/sensing/imu/tamagawa/imu_raw \
/sensing/lidar/front/scan \
/sensing/lidar/left/pointcloud_raw \
/sensing/lidar/left/pointcloud_raw_ex \
/sensing/lidar/right/pointcloud_raw \
/sensing/lidar/right/pointcloud_raw_ex \
/sensing/lidar/top/pointcloud_raw \
/sensing/lidar/top/pointcloud_raw_ex \
/vehicle/status/control_mode \
/vehicle/status/gear_status \
/vehicle/status/hazard_lights_status \
/vehicle/status/steering_status \
/vehicle/status/turn_indicators_status \
/vehicle/status/velocity_status
