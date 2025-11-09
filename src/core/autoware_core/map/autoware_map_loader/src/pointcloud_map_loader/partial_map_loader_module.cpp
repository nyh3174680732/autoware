// Copyright 2022 The Autoware Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "partial_map_loader_module.hpp"

#include <map>
#include <string>
#include <utility>

namespace autoware::map_loader
{
PartialMapLoaderModule::PartialMapLoaderModule(
  rclcpp::Node * node, std::map<std::string, PCDFileMetadata> pcd_file_metadata_dict)
: logger_(node->get_logger()), all_pcd_file_metadata_dict_(std::move(pcd_file_metadata_dict))  //移动语义避免拷贝，保存所有PCD元数据
{
  get_partial_pcd_maps_service_ = node->create_service<GetPartialPointCloudMap>(
    "service/get_partial_pcd_map",
    std::bind(
      &PartialMapLoaderModule::on_service_get_partial_point_cloud_map, this, std::placeholders::_1,
      std::placeholders::_2));

  // 创建发布器
  partial_map_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "output/partial_pointcloud_map",
    rclcpp::QoS(1).transient_local());

  // timer_ = node->create_wall_timer(
  //   std::chrono::milliseconds(100),
  //   std::bind(&PartialMapLoaderModule::timer_callback, this));
    
  RCLCPP_INFO(logger_, "✅ Partial map loader module initialized");
  RCLCPP_INFO(logger_, "   📡 Publishing to: /map/partial_pointcloud_map");
  RCLCPP_INFO(logger_, "   🔧 Service available: service/get_partial_pcd_map");
}

void PartialMapLoaderModule::timer_callback()
{
  if (has_data_) {
    // 更新时间戳
    cached_pcd_.header.stamp = rclcpp::Clock().now();
    partial_map_pub_->publish(cached_pcd_);
  }
}

void PartialMapLoaderModule::partial_area_load( //区域加载函数
  const autoware_map_msgs::msg::AreaInfo & area,
  const GetPartialPointCloudMap::Response::SharedPtr & response) const
{
  // iterate over all the available pcd map grids
  for (const auto & ele : all_pcd_file_metadata_dict_) {
    std::string path = ele.first;
    PCDFileMetadata metadata = ele.second;

    // assume that the map ID = map path (for now)
    const std::string & map_id = path;

    // skip if the pcd file is not within the queried area
    if (!is_grid_within_queried_area(area, metadata)) continue; //检查每个地图网格是否在查询区域内

    autoware_map_msgs::msg::PointCloudMapCellWithID pointcloud_map_cell_with_id =
      load_point_cloud_map_cell_with_id(path, map_id);
    pointcloud_map_cell_with_id.metadata.min_x = metadata.min.x;
    pointcloud_map_cell_with_id.metadata.min_y = metadata.min.y;
    pointcloud_map_cell_with_id.metadata.max_x = metadata.max.x;
    pointcloud_map_cell_with_id.metadata.max_y = metadata.max.y;

    response->new_pointcloud_with_ids.push_back(pointcloud_map_cell_with_id);
  }
}

bool PartialMapLoaderModule::on_service_get_partial_point_cloud_map(
  GetPartialPointCloudMap::Request::SharedPtr req,
  GetPartialPointCloudMap::Response::SharedPtr res) const
{
  auto area = req->area;
  partial_area_load(area, res);
  res->header.frame_id = "map";

  std::vector<std::string> pcd_paths;
  // 合并所有点云并发布
  if(!res->new_pointcloud_with_ids.empty()) {
    for (const auto & cell : res->new_pointcloud_with_ids) {
      pcd_paths.push_back(cell.cell_id);
      RCLCPP_INFO(logger_, "分开加载的地图路径是: %s", cell.cell_id.c_str());
    }
    // ✅ 更新缓存的点云数据
    cached_pcd_ = load_pcd_files(pcd_paths);
    cached_pcd_.header.frame_id = "map";
    cached_pcd_.header.stamp = rclcpp::Clock().now();

    partial_map_pub_->publish(cached_pcd_);
  }
  return true;
}

autoware_map_msgs::msg::PointCloudMapCellWithID
PartialMapLoaderModule::load_point_cloud_map_cell_with_id(
  const std::string & path, const std::string & map_id) const
{
  sensor_msgs::msg::PointCloud2 pcd;
  if (pcl::io::loadPCDFile(path, pcd) == -1) {
    RCLCPP_ERROR_STREAM(logger_, "PCD load failed: " << path);
  }
  autoware_map_msgs::msg::PointCloudMapCellWithID pointcloud_map_cell_with_id;
  pointcloud_map_cell_with_id.pointcloud = pcd;
  pointcloud_map_cell_with_id.cell_id = map_id;
  return pointcloud_map_cell_with_id;
}

sensor_msgs::msg::PointCloud2 PartialMapLoaderModule::load_pcd_files(
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
