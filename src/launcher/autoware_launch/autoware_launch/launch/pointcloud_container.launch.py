# Copyright 2021 Tier IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

#创建了一个可组合节点容器
def generate_launch_description():
    def add_launch_arg(name: str, default_value=None):
        return DeclareLaunchArgument(name, default_value=default_value)
    #条件设置容器类型（单线程）
    set_container_executable = SetLaunchConfiguration(
        "container_executable", #变量名
        "component_container", #单线程容器可执行文件名
        condition=UnlessCondition(LaunchConfiguration("use_multithread")), #如果use_multithread为false则使用单线程容器
    )
    #条件设置容器类型（多线程）
    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable", #变量名
        "component_container_mt", #多线程容器可执行文件名
        condition=IfCondition(LaunchConfiguration("use_multithread")), #如果use_multithread为true则使用多线程容器
    )
    #定义glog组件节点
    glog_component = ComposableNode( #可组合节点（可以加载到容器中的节点）
        package="autoware_glog_component", #组件所在的包
        plugin="autoware::glog_component::GlogComponent", #组件的插件名称
        name="glog_component",#组件节点名称
        namespace="pointcloud_container",#组件节点命名空间
    )
    #定义可组合节点容器
    pointcloud_container = ComposableNodeContainer(#创建一个可组合节点容器
        name=LaunchConfiguration("container_name"),#容器名称
        namespace="/",#容器命名空间
        package="rclcpp_components",#ROS2组件系统包
        executable=LaunchConfiguration("container_executable"),#容器可执行文件（单线程或多线程）
        composable_node_descriptions=[glog_component],#初始加载的组件列表
        output="both",#输出到终端和日志文件
    )

    return LaunchDescription(
        [
            add_launch_arg("use_multithread", "false"),
            add_launch_arg("container_name", "pointcloud_container"),
            set_container_executable,
            set_container_mt_executable,
            pointcloud_container,
        ]
    )
