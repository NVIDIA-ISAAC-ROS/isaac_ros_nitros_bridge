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

#include "ros1_image_converter.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros_bridge
{

void ImageConverterNode::onInit()
{
  ros::NodeHandle & private_nh = getPrivateNodeHandle();

  // Create publisher topics
  bridge_image_pub_ = private_nh.advertise<isaac_ros_nitros_bridge_msgs::NitrosBridgeImage>(
    "ros1_output_bridge_image", 10);
  image_pub_ = private_nh.advertise<sensor_msgs::Image>("ros1_output_image", 10);

  // Create subscriber topics
  image_sub_ = private_nh.subscribe(
    "ros1_input_image", 10, &ImageConverterNode::ROS1To2Callback,
    this);
  bridge_image_sub_ = private_nh.subscribe(
    "ros1_input_bridge_image", 10,
    &ImageConverterNode::ROS2To1Callback, this);
  private_nh.param("num_blocks", num_blocks_, 40);
}

void ImageConverterNode::ROS1To2Callback(const sensor_msgs::Image::ConstPtr & input_msg)
{
  ROS_DEBUG("Received image message in ROS1To2Callback.");
  if (first_msg_received_ == false) {
    ipc_buffer_manager_ =
      std::make_shared<IPCBufferManager>(num_blocks_, input_msg->step * input_msg->height);
    first_msg_received_ = true;
  }

  isaac_ros_nitros_bridge_msgs::NitrosBridgeImage msg;
  msg.header = input_msg->header;
  msg.height = input_msg->height;
  msg.width = input_msg->width;
  msg.encoding = input_msg->encoding;
  msg.step = input_msg->step;
  msg.data.resize(sizeof(cudaIpcMemHandle_t));

  auto cuda_err = cudaMemcpy(
    ipc_buffer_manager_->get_cur_buffer_ptr(),
    input_msg->data.data(),
    msg.step * msg.height,
    cudaMemcpyHostToDevice);
  if (cudaSuccess != cuda_err) {
    ROS_ERROR("Failed to copy msg data to GPU: %s", cudaGetErrorString(cuda_err));
  }

  memcpy(
    msg.data.data(), ipc_buffer_manager_->get_cur_ipc_mem_handle(),
    sizeof(cudaIpcMemHandle_t));

  ipc_buffer_manager_->next();
  bridge_image_pub_.publish(msg);
}

void ImageConverterNode::ROS2To1Callback(
  const isaac_ros_nitros_bridge_msgs::NitrosBridgeImage::ConstPtr & input_msg)
{
  ROS_DEBUG("Received bridge image message in ROS2To1Callback.");

  if (sizeof(cudaIpcMemHandle_t) != input_msg->data.size()) {
    ROS_ERROR("The message data size does not match with IPC handle size");
  }

  sensor_msgs::Image msg;
  msg.header = input_msg->header;
  msg.height = input_msg->height;
  msg.width = input_msg->width;
  msg.encoding = input_msg->encoding;
  msg.step = input_msg->step;
  msg.data.resize(input_msg->step * input_msg->height);

  void * gpu_buffer = NULL;
  cudaIpcMemHandle_t ipc_mem_handle;

  // Try to get CUDA pointer from local map first
  if (handle_ptr_map_.find(input_msg->data) != handle_ptr_map_.end()) {
    gpu_buffer = handle_ptr_map_[input_msg->data];
    ROS_DEBUG("Found handle in map.");
  } else {
    // CUDA Pointer is not found in map, open from IPC handle and save into map
    memcpy(&ipc_mem_handle, input_msg->data.data(), sizeof(cudaIpcMemHandle_t));
    auto err = cudaIpcOpenMemHandle(
      &gpu_buffer, ipc_mem_handle,
      cudaIpcMemLazyEnablePeerAccess);
    if (cudaSuccess != err) {
      ROS_ERROR("Failed to open CUDA IPC handle: %s", cudaGetErrorString(err));
    }
    handle_ptr_map_[input_msg->data] = gpu_buffer;
  }

  auto err = cudaMemcpy(
    msg.data.data(), gpu_buffer, input_msg->step * input_msg->height, cudaMemcpyDeviceToHost);
  if (cudaSuccess != err) {
    ROS_ERROR("Failed to copy data from gpu buffer: %s", cudaGetErrorString(err));
  }

  image_pub_.publish(msg);
}

}  // namespace nitros_bridge
}  // namespace isaac_ros
}  // namespace nvidia

PLUGINLIB_EXPORT_CLASS(nvidia::isaac_ros::nitros_bridge::ImageConverterNode, nodelet::Nodelet);
