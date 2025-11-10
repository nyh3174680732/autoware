colcon build --packages-select tier4_automatic_goal_rviz_plugin --cmake-clean-cache

ros2 service call /map/get_differential_pointcloud_paths   autoware_map_msgs/srv/GetNewDifferentialPaths   "{area: {center_x: 0.0, center_y: 0.0, radius: 10.0}}"

