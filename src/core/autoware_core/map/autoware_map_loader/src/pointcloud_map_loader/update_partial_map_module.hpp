#ifndef UPDATE_PARTIAL_MAP_HPP
#define UPDATE_PARTIAL_MAP_HPP

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <autoware_map_msgs/srv/get_differential_point_cloud_map.hpp>
#include <autoware_map_msgs/srv/get_new_differential_paths.hpp>
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
};

}  // namespace autoware::map_loader

#endif  // UPDATE_PARTIAL_MAP_HPP