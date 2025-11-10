#ifndef UPDATE_PARTIAL_MAP_HPP
#define UPDATE_PARTIAL_MAP_HPP

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <autoware_map_msgs/srv/get_new_differential_paths.hpp>
#include <autoware_adapi_v1_msgs/msg/vehicle_kinematics.hpp>

namespace autoware::map_loader
{
class UpdatePartialMap
{
    using GetNewDifferentialPaths = autoware_map_msgs::srv::GetNewDifferentialPaths;
public:
    UpdatePartialMap(rclcpp::Node * node);

private:
    void send_request();
    rclcpp::Logger logger_;
    rclcpp::Client<GetNewDifferentialPaths>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr differential_map_pub_;
    mutable sensor_msgs::msg::PointCloud2 differential_pcd_; //要发布的点云
    rclcpp::Subscription<autoware_adapi_v1_msgs::msg::VehicleKinematics>::SharedPtr kinematics_sub_;
    void kinematics_callback(const autoware_adapi_v1_msgs::msg::VehicleKinematics::SharedPtr msg);
    double vehicle_x_ = 0.0;
    double vehicle_y_ = 0.0;
    bool position_received_ = false; 

    [[nodiscard]] sensor_msgs::msg::PointCloud2 load_pcd_files(
    const std::vector<std::string> & pcd_paths) const;
};

}  // namespace autoware::map_loader

#endif  // UPDATE_PARTIAL_MAP_HPP