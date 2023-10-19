// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0


#include <cuda_runtime_api.h>
#include <string>

#include "isaac_ros_nitros_bridge_ros2/image_converter_node.hpp"
#include "sensor_msgs/image_encodings.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros_bridge
{

ImageConverterNode::ImageConverterNode(const rclcpp::NodeOptions options)
: rclcpp::Node("image_converter_node", options),
  nitros_pub_{std::make_shared<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<
        nvidia::isaac_ros::nitros::NitrosImage>>(
      this, "ros2_output_image",
      nvidia::isaac_ros::nitros::nitros_image_rgb8_t::supported_type_name)},
  bridge_image_pub_{create_publisher<isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeImage>(
      "ros2_output_bridge_image", 10)},
  nitros_sub_{std::make_shared<nvidia::isaac_ros::nitros::ManagedNitrosSubscriber<
        nvidia::isaac_ros::nitros::NitrosImageView>>(
      this, "ros2_input_image", nvidia::isaac_ros::nitros::nitros_image_rgb8_t::supported_type_name,
      std::bind(&ImageConverterNode::ROS2To1Callback, this,
      std::placeholders::_1))},
  bridge_image_sub_{create_subscription<isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeImage>(
      "ros2_input_bridge_image", 10, std::bind(&ImageConverterNode::ROS1To2Callback, this,
      std::placeholders::_1))},
  num_blocks_(declare_parameter<int64_t>("num_blocks", 40)) {}

ImageConverterNode::~ImageConverterNode() = default;

void ImageConverterNode::ROS1To2Callback(
  const isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeImage::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Received bridge image message in ROS1To2Callback.");

  if (sizeof(cudaIpcMemHandle_t) != msg->data.size()) {
    RCLCPP_ERROR(this->get_logger(), "The message data size does not match with IPC handle size");
  }

  void * gpu_buffer = nullptr;
  cudaIpcMemHandle_t ipc_mem_handle;
  // Try to get CUDA pointer from local map first
  if (handle_ptr_map_.find(msg->data) != handle_ptr_map_.end()) {
    gpu_buffer = handle_ptr_map_[msg->data];
    RCLCPP_DEBUG(this->get_logger(), "Found handle in map.");
  } else {
    // CUDA Pointer is not found in map, open from IPC handle and save into map
    memcpy(&ipc_mem_handle, msg->data.data(), sizeof(cudaIpcMemHandle_t));
    auto err = cudaIpcOpenMemHandle(
      &gpu_buffer, ipc_mem_handle,
      cudaIpcMemLazyEnablePeerAccess);
    if (cudaSuccess != err) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to open CUDA IPC handle: %s",
        cudaGetErrorString(err));
    }
    handle_ptr_map_[msg->data] = gpu_buffer;
  }

  nvidia::isaac_ros::nitros::NitrosImage nitros_image =
    nvidia::isaac_ros::nitros::NitrosImageBuilder()
    .WithHeader(msg->header)
    .WithEncoding(img_encodings::RGB8)
    .WithDimensions(msg->height, msg->width)
    .WithGpuData(gpu_buffer)
    .Build();

  nitros_pub_->publish(nitros_image);
  RCLCPP_DEBUG(this->get_logger(), "Sent CUDA buffer with memory at: %p", gpu_buffer);
}

void ImageConverterNode::ROS2To1Callback(const nvidia::isaac_ros::nitros::NitrosImageView view)
{
  RCLCPP_DEBUG(this->get_logger(), "Received image message in ROS2To1Callback.");
  if (first_msg_received_ == false) {
    ipc_buffer_manager_ = std::make_shared<IPCBufferManager>(num_blocks_, view.GetSizeInBytes());
    first_msg_received_ = true;
  }

  isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeImage img_msg;
  img_msg.header.frame_id = view.GetFrameId();
  img_msg.header.stamp.sec = view.GetTimestampSeconds();
  img_msg.header.stamp.nanosec = view.GetTimestampNanoseconds();
  img_msg.height = view.GetHeight();
  img_msg.width = view.GetWidth();
  img_msg.encoding = view.GetEncoding();
  img_msg.step = view.GetSizeInBytes() / view.GetHeight();

  auto cuda_err = cudaMemcpy(
    ipc_buffer_manager_->get_cur_buffer_ptr(),
    view.GetGpuData(),
    view.GetSizeInBytes(),
    cudaMemcpyDeviceToDevice);
  if (cudaSuccess != cuda_err) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to copy msg data to GPU: %s",
      cudaGetErrorString(cuda_err));
  }

  img_msg.data.resize(sizeof(cudaIpcMemHandle_t));
  memcpy(
    img_msg.data.data(), ipc_buffer_manager_->get_cur_ipc_mem_handle(),
    sizeof(cudaIpcMemHandle_t));

  ipc_buffer_manager_->next();
  bridge_image_pub_->publish(img_msg);
}

}  // namespace nitros_bridge
}  // namespace isaac_ros
}  // namespace nvidia

// Register as component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::nitros_bridge::ImageConverterNode)
