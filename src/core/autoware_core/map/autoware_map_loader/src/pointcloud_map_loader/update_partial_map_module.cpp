#include "update_partial_map_module.hpp"

using namespace std::chrono_literals;

namespace autoware::map_loader
{
UpdatePartialMap::UpdatePartialMap(rclcpp::Node * node) : logger_(node->get_logger())
{
    RCLCPP_INFO(logger_, "地图路径更新初始化");
    // 创建服务客户端
    client_ = node->create_client<GetNewDifferentialPaths>(
        "/map/get_differential_pointcloud_paths");

    // 定时器，每隔5秒调用一次
    timer_ = node->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&UpdatePartialMap::send_request, this));
}

void UpdatePartialMap::send_request()
{
    // 创建请求
    GetNewDifferentialPaths::Request::SharedPtr request;
    
    // 设置 AreaInfo 的固定值
    request->area.center_x = 0.0f;  // 固定值
    request->area.center_y = 0.0f;  // 固定值
    request->area.radius = 10.0f;   // 固定值

    // 发送请求
     // 检查客户端是否可用
    if (!client_->wait_for_service(1s)) {
        RCLCPP_ERROR(logger_, "服务未可用");
        return;
    }
    auto response_future = client_->async_send_request(request);
    // 等待响应
    std::future_status status = response_future.wait_for(1s);
    if (status == std::future_status::ready) {
        // 获取响应
        auto response = response_future.get();
        // if (response->new_pointcloud_with_ids.empty()) {
        //     RCLCPP_INFO(logger_, "没有新的点云地图");
        // } else {
        //     RCLCPP_INFO(logger_, "成功加载新的点云地图");
        //     // 处理新加载的点云地图
        //     for (const auto &pointcloud : response->new_pointcloud_with_ids) {
        //         RCLCPP_INFO(logger_, "加载点云地图 ID: %s", pointcloud.cell_id.c_str());
        //         // 此处可以添加处理点云数据的逻辑
        //     }
        // }
    } else {
        RCLCPP_ERROR(logger_, "请求超时");
    }
}

}  // namespace autoware::map_loader
