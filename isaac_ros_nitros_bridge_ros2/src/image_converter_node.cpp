// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include <sys/un.h>
#include <string>

#include "isaac_ros_nitros_bridge_ros2/image_converter_node.hpp"
#include "sensor_msgs/image_encodings.hpp"


#define SYS_pidfd_getfd_nitros_bridge 438


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
  num_blocks_(declare_parameter<int64_t>("num_blocks", 40))
{
  cudaSetDevice(0);
  cuDevicePrimaryCtxRetain(&m_ctx, 0);
}

ImageConverterNode::~ImageConverterNode() = default;


void ImageConverterNode::ROS1To2Callback(
  const isaac_ros_nitros_bridge_interfaces::msg::NitrosBridgeImage::SharedPtr msg)
{
  cuCtxSetCurrent(m_ctx);

  CUdeviceptr gpu_buffer = 0ULL;
  CUmemGenericAllocationHandle generic_allocation_handle;

  if (handle_ptr_map_.find(msg->data.data()[1]) != handle_ptr_map_.end()) {
    gpu_buffer = handle_ptr_map_[msg->data.data()[1]];
    RCLCPP_DEBUG(this->get_logger(), "Found FD in local map.");
  } else {
    int pidfd = syscall(SYS_pidfd_open, msg->data.data()[0], 0);
    if (pidfd <= 0) {
      perror("SYS_pidfd_open failed");
    }
    int fd = syscall(SYS_pidfd_getfd_nitros_bridge, pidfd, msg->data.data()[1], 0);
    if (fd <= 0) {
      perror("SYS_pidfd_getfd failed");
    }

    auto cuda_err = cuMemImportFromShareableHandle(
      &generic_allocation_handle,
      reinterpret_cast<void *>((uintptr_t)fd),
      CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR);
    if (CUDA_SUCCESS != cuda_err) {
      const char * error_str = NULL;
      cuGetErrorString(cuda_err, &error_str);
      RCLCPP_ERROR(
        this->get_logger(), "Failed to call cuMemImportFromShareableHandle %s",
        error_str);
    }

    CUmemAllocationProp prop = {};
    prop.type = CU_MEM_ALLOCATION_TYPE_PINNED;
    prop.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
    prop.location.id = 0;
    prop.requestedHandleTypes = CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR;
    size_t granularity = 0;

    cuda_err = cuMemGetAllocationGranularity(
      &granularity, &prop, CU_MEM_ALLOC_GRANULARITY_MINIMUM);
    if (CUDA_SUCCESS != cuda_err) {
      const char * error_str = NULL;
      cuGetErrorString(cuda_err, &error_str);
      RCLCPP_ERROR(
        this->get_logger(), "Failed to call cuMemGetAllocationGranularity %s",
        error_str);
    }

    auto alloc_size = msg->height * msg->step;
    // The alloc size must be the integral multiple of granularity
    alloc_size = alloc_size - (alloc_size % granularity) + granularity;

    cuda_err = cuMemAddressReserve(&gpu_buffer, alloc_size, 0, 0, 0);
    if (CUDA_SUCCESS != cuda_err) {
      const char * error_str = NULL;
      cuGetErrorString(cuda_err, &error_str);
      RCLCPP_ERROR(this->get_logger(), "Failed to call cuMemCreate %s", error_str);
    }

    cuda_err = cuMemMap(gpu_buffer, alloc_size, 0, generic_allocation_handle, 0);
    if (CUDA_SUCCESS != cuda_err) {
      const char * error_str = NULL;
      cuGetErrorString(cuda_err, &error_str);
      RCLCPP_ERROR(this->get_logger(), "Failed to call cuMemCreate %s", error_str);
    }

    CUmemAccessDesc accessDesc = {};
    accessDesc.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
    accessDesc.location.id = 0;
    accessDesc.flags = CU_MEM_ACCESS_FLAGS_PROT_READWRITE;
    cuda_err = cuMemSetAccess(gpu_buffer, alloc_size, &accessDesc, 1);
    if (CUDA_SUCCESS != cuda_err) {
      const char * error_str = NULL;
      cuGetErrorString(cuda_err, &error_str);
      RCLCPP_ERROR(this->get_logger(), "Failed to call cuMemSetAccess %s", error_str);
    }
    handle_ptr_map_[msg->data.data()[1]] = gpu_buffer;
  }

  nvidia::isaac_ros::nitros::NitrosImage nitros_image =
    nvidia::isaac_ros::nitros::NitrosImageBuilder()
    .WithHeader(msg->header)
    .WithEncoding(img_encodings::RGB8)
    .WithDimensions(msg->height, msg->width)
    .WithGpuData(reinterpret_cast<void *>(gpu_buffer))
    .Build();

  nitros_pub_->publish(nitros_image);
}

void ImageConverterNode::ROS2To1Callback(const nvidia::isaac_ros::nitros::NitrosImageView view)
{
  cuCtxSetCurrent(m_ctx);

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

  auto cuda_err = cuMemcpyDtoD(
    ipc_buffer_manager_->get_cur_buffer_ptr(),
    (CUdeviceptr)(view.GetGpuData()),
    view.GetSizeInBytes());
  if (CUDA_SUCCESS != cuda_err) {
    const char * error_str = NULL;
    cuGetErrorString(cuda_err, &error_str);
    RCLCPP_ERROR(
      rclcpp::get_logger("IPCBufferManager"), "Failed to call cuMemcpyDtoD %s",
      error_str);
  }

  // cuMemcpyDtoD is an aysnchronize call, wait until it complete.
  cuCtxSynchronize();

  img_msg.data.push_back(ipc_buffer_manager_->get_cur_ipc_mem_handle()[0]);
  img_msg.data.push_back(ipc_buffer_manager_->get_cur_ipc_mem_handle()[1]);

  ipc_buffer_manager_->next();
  bridge_image_pub_->publish(img_msg);
}

}  // namespace nitros_bridge
}  // namespace isaac_ros
}  // namespace nvidia

// Register as component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::nitros_bridge::ImageConverterNode)
