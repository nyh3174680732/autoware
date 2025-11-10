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

#include "differential_map_loader_module.hpp"

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace autoware::map_loader
{
DifferentialMapLoaderModule::DifferentialMapLoaderModule(
  rclcpp::Node * node, std::map<std::string, PCDFileMetadata> pcd_file_metadata_dict)
: logger_(node->get_logger()), all_pcd_file_metadata_dict_(std::move(pcd_file_metadata_dict))
{
  get_differential_pcd_maps_service_ = node->create_service<GetDifferentialPointCloudMap>(
    "service/get_differential_pcd_map",
    std::bind(
      &DifferentialMapLoaderModule::on_service_get_differential_point_cloud_map, this,
      std::placeholders::_1, std::placeholders::_2));

  get_differential_pcd_paths_service_ = node->create_service<GetNewDifferentialPaths>(
    "service/get_differential_pcd_paths",
    std::bind(
      &DifferentialMapLoaderModule::on_service_get_differential_point_cloud_paths, this,
      std::placeholders::_1, std::placeholders::_2));
}

bool DifferentialMapLoaderModule::on_service_get_differential_point_cloud_paths(
  GetNewDifferentialPaths::Request::SharedPtr req,
  GetNewDifferentialPaths::Response::SharedPtr res) const
{
  auto area = req->area;
  RCLCPP_INFO(logger_, "收到的参数为:x=%f,y=%f,r=%f",area.center_x,area.center_y,area.radius);
  auto start_time = std::chrono::steady_clock::now();
  
  differential_paths_load(area, res);
  res->header.frame_id = "map";

  auto end_time = std::chrono::steady_clock::now();
  RCLCPP_INFO(logger_, "差分地图路径加载完成，耗时: %f秒", std::chrono::duration<double>(end_time - start_time).count());


  return true;
}

void DifferentialMapLoaderModule::differential_paths_load(
  const autoware_map_msgs::msg::AreaInfo & area_info,
  const GetNewDifferentialPaths::Response::SharedPtr & response) const
{
  // iterate over all the available pcd map grids
  for (const auto & ele : all_pcd_file_metadata_dict_) {
    std::string path = ele.first;
    PCDFileMetadata metadata = ele.second;

    // assume that the map ID = map path (for now)
    const std::string & map_id = path;

    // skip if the pcd file is not within the queried area
    if (!is_grid_within_queried_area(area_info, metadata)){
      continue;
    }else{
      response->new_paths.push_back(map_id);
    } 
  }
  response->header.stamp = rclcpp::Clock().now();
}


void DifferentialMapLoaderModule::differential_area_load(
  const autoware_map_msgs::msg::AreaInfo & area_info, const std::vector<std::string> & cached_ids,
  const GetDifferentialPointCloudMap::Response::SharedPtr & response) const
{
  // iterate over all the available pcd map grids
  std::vector<bool> should_remove(static_cast<int>(cached_ids.size()), true);
  for (const auto & ele : all_pcd_file_metadata_dict_) {
    std::string path = ele.first;
    PCDFileMetadata metadata = ele.second;

    // assume that the map ID = map path (for now)
    const std::string & map_id = path;

    // skip if the pcd file is not within the queried area
    if (!is_grid_within_queried_area(area_info, metadata)) continue;

    auto id_in_cached_list = std::find(cached_ids.begin(), cached_ids.end(), map_id);
    if (id_in_cached_list != cached_ids.end()) {
      int index = static_cast<int>(id_in_cached_list - cached_ids.begin());
      should_remove[index] = false;
    } else {
      autoware_map_msgs::msg::PointCloudMapCellWithID pointcloud_map_cell_with_id =
        load_point_cloud_map_cell_with_id(path, map_id);
      pointcloud_map_cell_with_id.metadata.min_x = metadata.min.x;
      pointcloud_map_cell_with_id.metadata.min_y = metadata.min.y;
      pointcloud_map_cell_with_id.metadata.max_x = metadata.max.x;
      pointcloud_map_cell_with_id.metadata.max_y = metadata.max.y;
      response->new_pointcloud_with_ids.push_back(pointcloud_map_cell_with_id);
    }
  }

  for (size_t i = 0; i < cached_ids.size(); ++i) {
    if (should_remove[i]) {
      response->ids_to_remove.push_back(cached_ids[i]);
    }
  }
}

bool DifferentialMapLoaderModule::on_service_get_differential_point_cloud_map(
  GetDifferentialPointCloudMap::Request::SharedPtr req,
  GetDifferentialPointCloudMap::Response::SharedPtr res) const
{
  auto area = req->area;
  std::vector<std::string> cached_ids = req->cached_ids;
  differential_area_load(area, cached_ids, res);
  res->header.frame_id = "map";
  RCLCPP_INFO(logger_, "差分地图加载");
  return true;
}

autoware_map_msgs::msg::PointCloudMapCellWithID
DifferentialMapLoaderModule::load_point_cloud_map_cell_with_id(
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
}  // namespace autoware::map_loader
