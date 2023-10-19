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

#ifndef ISAAC_ROS_NITROS_BRIDGE_ROS1__IMAGE_CONVERTER_NODE_HPP_
#define ISAAC_ROS_NITROS_BRIDGE_ROS1__IMAGE_CONVERTER_NODE_HPP_

#include <sstream>
#include <chrono>
#include <cuda_runtime_api.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"

#include "ipc_buffer_manager.hpp"
#include "isaac_ros_nitros_bridge_msgs/NitrosBridgeImage.h"

namespace nvidia
{
namespace isaac_ros
{
namespace nitros_bridge
{

class ImageConverterNode : public nodelet::Nodelet
{
public:
  ImageConverterNode() {}

private:
  virtual void onInit() override;

  // Copy ros message data into GPU and convert to NITROS bridge message
  void ROS1To2Callback(const sensor_msgs::Image::ConstPtr & msg);

  // Convert bridge message into ros message and copy back to CPU
  void ROS2To1Callback(const isaac_ros_nitros_bridge_msgs::NitrosBridgeImage::ConstPtr & msg);

  // Publisher for output image messages
  ros::Publisher image_pub_;
  // Publisher for output bridge messages
  ros::Publisher bridge_image_pub_;
  // Subscription to input image messages
  ros::Subscriber image_sub_;
  // Subscription to input bridge messages
  ros::Subscriber bridge_image_sub_;

  // Numboer of blocks of device memory pool
  int num_blocks_;
  // Map between CUDA memory handle bytes to device pointer
  std::map<std::vector<uint8_t>, void *> handle_ptr_map_;
  // CUDA IPC memory pool manager
  std::shared_ptr<IPCBufferManager> ipc_buffer_manager_;
  // If received first message
  bool first_msg_received_ = false;
};

}  // namespace nitros_bridge
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS_BRIDGE_ROS1__IMAGE_CONVERTER_NODE_HPP_
