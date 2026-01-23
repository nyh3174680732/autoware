#include "update_partial_map_module.hpp"

using namespace std::chrono_literals;

namespace autoware::map_loader
{
UpdatePartialMap::UpdatePartialMap(rclcpp::Node * node) : logger_(node->get_logger())
{
    RCLCPP_INFO(logger_, "地图路径更新初始化");
    // 创建服务客户端
    client_ = node->create_client<GetNewDifferentialPaths>(
        "service/get_differential_pcd_paths");

    differential_map_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "output/differential_pointcloud_map",
        rclcpp::QoS(1).transient_local());

    // 订阅车辆位置
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(rclcpp::ReliabilityPolicy::BestEffort)  // 改为 BEST_EFFORT
        .durability(rclcpp::DurabilityPolicy::Volatile);     // VOLATILE
    kinematics_sub_ =  node->create_subscription<autoware_adapi_v1_msgs::msg::VehicleKinematics>(
        "/api/vehicle/kinematics", qos,
        std::bind(&UpdatePartialMap::kinematics_callback, this, std::placeholders::_1));

    // 定时器，每隔5秒调用一次
    timer_ = node->create_wall_timer(
        std::chrono::milliseconds(10000),
        std::bind(&UpdatePartialMap::send_request, this));
}

void UpdatePartialMap::kinematics_callback(const autoware_adapi_v1_msgs::msg::VehicleKinematics::SharedPtr msg)
{
    // 更新车辆位置
    vehicle_x_ = msg->pose.pose.pose.position.x;
    vehicle_y_ = msg->pose.pose.pose.position.y;
    
    // 标记已收到位置信息
    if (!position_received_) {
        position_received_ = true;
        RCLCPP_INFO(logger_, 
                    "✅ 首次接收到车辆位置: x=%.2f, y=%.2f [frame: %s]", 
                    vehicle_x_, vehicle_y_,
                    msg->pose.header.frame_id.c_str());
    }
    
    // RCLCPP_INFO(logger_, "车辆位置更新: x=%.2f, y=%.2f", vehicle_x_, vehicle_y_);
}

void UpdatePartialMap::send_request()
{
    // 创建请求
    //GetNewDifferentialPaths::Request::SharedPtr request;
    auto request = std::make_shared<GetNewDifferentialPaths::Request>();
    
    // 设置 AreaInfo 的固定值
    // request->area.center_x = 0.0f;  // 固定值
    // request->area.center_y = 0.0f;  // 固定值
    // request->area.radius = 30.0f;   // 固定值

    // 使用当前车辆位置替换固定值
    request->area.center_x = static_cast<float>(vehicle_x_);
    request->area.center_y = static_cast<float>(vehicle_y_);
    request->area.radius = 100.0f;  // 

    auto callback = [this](rclcpp::Client<GetNewDifferentialPaths>::SharedFuture future) {
        try {
            auto response = future.get();
            //RCLCPP_INFO(logger_, "请求成功，获取到响应");
            // 处理响应数据
            if (response->new_paths.empty()) {
                RCLCPP_INFO(logger_, "没有新的地图路径");
            } else {
                std::vector<std::string> pcd_paths;
                //RCLCPP_INFO(logger_, "成功加载 %zu 个新的地图路径", response->new_paths.size());
                for (const auto &path : response->new_paths) {
                    //RCLCPP_INFO(logger_, "加载地图路径: %s", path.c_str());
                    pcd_paths.push_back(path);
                }
                differential_pcd_ = load_pcd_files(pcd_paths);
                differential_pcd_.header.frame_id = "map";
                differential_pcd_.header.stamp = rclcpp::Clock().now();
                differential_map_pub_->publish(differential_pcd_);
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(logger_, "获取响应时出错: %s", e.what());
        }
    };
    client_->async_send_request(request, callback);

    // RCLCPP_INFO(logger_, "进入函数");
    // // 发送请求
    //  // 检查客户端是否可用
    // if (!client_->wait_for_service(1s)) {
    //     RCLCPP_ERROR(logger_, "服务未可用");
    //     return;
    // }
    // RCLCPP_INFO(logger_, "准备发送请求...");
    // auto response_future = client_->async_send_request(request);
    // RCLCPP_INFO(logger_, "请求已发送，等待响应...");
    // // 等待响应
    // std::future_status status = response_future.wait_for(5s);
    // if (status == std::future_status::ready) {
    //     RCLCPP_ERROR(logger_, "请求成功");
    //     // RCLCPP_INFO(logger_, "获取地图路径成功");
    //     // // 获取响应
    //     // auto response = response_future.get();
    //     //  // 打印路径信息
    //     // if (response->new_paths.empty()) {
    //     //     RCLCPP_INFO(logger_, "没有新的地图路径");
    //     // } else {
    //     //     RCLCPP_INFO(logger_, "成功加载新的地图路径");
    //     //     for (const auto &path : response->new_paths) {
    //     //         RCLCPP_INFO(logger_, "加载地图路径: %s", path.c_str());
    //     //         // 此处可以添加处理路径数据的逻辑
    //     //     }
    //     // }
    // }else if (status == std::future_status::timeout) {
    // // 请求超时
    //     RCLCPP_ERROR(logger_, "请求超时");
    // } else if (status == std::future_status::deferred) {
    //     // 请求被延迟
    //     RCLCPP_WARN(logger_, "请求被延迟");
    // }
}

sensor_msgs::msg::PointCloud2 UpdatePartialMap::load_pcd_files(
  const std::vector<std::string> & pcd_paths) const
{
  sensor_msgs::msg::PointCloud2 whole_pcd;
  sensor_msgs::msg::PointCloud2 partial_pcd;

  for (size_t i = 0; i < pcd_paths.size(); ++i) {
    auto & path = pcd_paths[i];

    if (pcl::io::loadPCDFile(path, partial_pcd) == -1) {
      RCLCPP_ERROR_STREAM(logger_, "PCD load failed: " << path);
    }

    if (whole_pcd.width == 0) {
      whole_pcd = partial_pcd;
    } else {
      whole_pcd.width += partial_pcd.width;
      whole_pcd.row_step += partial_pcd.row_step;
      whole_pcd.data.insert(whole_pcd.data.end(), partial_pcd.data.begin(), partial_pcd.data.end());
    }
  }

  whole_pcd.header.frame_id = "map";

  return whole_pcd;
}


}  // namespace autoware::map_loader
