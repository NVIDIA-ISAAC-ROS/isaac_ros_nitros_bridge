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

#include <fcntl.h>
#include <errno.h>
#include <sys/un.h>

#include "ros1_image_converter.hpp"

#define SYS_pidfd_getfd 438


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
  cudaSetDevice(0);
  cuDevicePrimaryCtxRetain(&m_ctx, 0);
}

void ImageConverterNode::ROS1To2Callback(const sensor_msgs::Image::ConstPtr & input_msg)
{
  cuCtxSetCurrent(m_ctx);

  if (first_msg_received_ == false) {
    ipc_buffer_manager_ = std::make_shared<IPCBufferManager>(num_blocks_, input_msg->step * input_msg->height);
    first_msg_received_ = true;
  }

  isaac_ros_nitros_bridge_msgs::NitrosBridgeImage msg;
  msg.header = input_msg->header;
  msg.height = input_msg->height;
  msg.width = input_msg->width;
  msg.encoding = input_msg->encoding;
  msg.step = input_msg->step;

  auto cuda_err = cuMemcpyHtoD(
    ipc_buffer_manager_->get_cur_buffer_ptr(),
    input_msg->data.data(),
    msg.step * msg.height);
  if (CUDA_SUCCESS != cuda_err) {
    const char *errorStr = NULL;
    cuGetErrorString(cuda_err, &errorStr);
    ROS_ERROR("Failed to copy msg data to GPU: %s", errorStr);
  }

  msg.data.push_back(ipc_buffer_manager_->get_cur_ipc_mem_handle()[0]);
  msg.data.push_back(ipc_buffer_manager_->get_cur_ipc_mem_handle()[1]);

  ipc_buffer_manager_->next();
  bridge_image_pub_.publish(msg);
}

void ImageConverterNode::ROS2To1Callback(
  const isaac_ros_nitros_bridge_msgs::NitrosBridgeImage::ConstPtr & input_msg)
{
  cuCtxSetCurrent(m_ctx);

  sensor_msgs::Image msg;
  msg.header = input_msg->header;
  msg.height = input_msg->height;
  msg.width = input_msg->width;
  msg.encoding = input_msg->encoding;
  msg.step = input_msg->step;
  msg.data.resize(input_msg->step * input_msg->height);

  CUdeviceptr gpu_buffer = 0ULL;
  cudaIpcMemHandle_t ipc_mem_handle;
  CUmemGenericAllocationHandle generic_allocation_handle;

  if (handle_ptr_map_.find(input_msg->data.data()[1]) != handle_ptr_map_.end()) {
    gpu_buffer = handle_ptr_map_[input_msg->data.data()[1]];
    ROS_DEBUG("Found FD in local map.");
  } else {
    int pidfd = syscall(SYS_pidfd_open, input_msg->data.data()[0], 0);
    if (pidfd <= 0) {
      perror("SYS_pidfd_open failed");
    }
    int fd = syscall(SYS_pidfd_getfd, pidfd, input_msg->data.data()[1], 0);
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
      ROS_ERROR( "Failed to cuMemImportFromShareableHandle %s", error_str);
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
      ROS_ERROR("Failed to call cuMemGetAllocationGranularity %s", error_str);
    }

    auto alloc_size = input_msg->height * input_msg->step;
    alloc_size = alloc_size - (alloc_size % granularity) + granularity;

    cuda_err = cuMemAddressReserve(&gpu_buffer, alloc_size, 0, 0, 0);
    if (CUDA_SUCCESS != cuda_err) {
      const char * error_str = NULL;
      cuGetErrorString(cuda_err, &error_str);
      ROS_ERROR( "Failed to call cuMemCreate %s", error_str);
    }

    cuda_err = cuMemMap(gpu_buffer, alloc_size, 0, generic_allocation_handle, 0);
    if (CUDA_SUCCESS != cuda_err) {
      const char * error_str = NULL;
      cuGetErrorString(cuda_err, &error_str);
      ROS_ERROR("Failed to call cuMemCreate %s", error_str);
    }

    CUmemAccessDesc accessDesc = {};
    accessDesc.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
    accessDesc.location.id = 0;
    accessDesc.flags = CU_MEM_ACCESS_FLAGS_PROT_READWRITE;
    cuda_err = cuMemSetAccess(gpu_buffer, alloc_size, &accessDesc, 1);
    if (CUDA_SUCCESS != cuda_err) {
      const char * error_str = NULL;
      cuGetErrorString(cuda_err, &error_str);
      ROS_ERROR("Failed to call cuMemSetAccess %s", error_str);
    }
    handle_ptr_map_[input_msg->data.data()[1]] = gpu_buffer;
  }

  auto cuda_err = cuMemcpyDtoH(
    msg.data.data(), gpu_buffer, input_msg->step * input_msg->height);
    if (CUDA_SUCCESS != cuda_err) {
      const char * error_str = NULL;
      cuGetErrorString(cuda_err, &error_str);
      ROS_ERROR("Failed to call cuMemcpyDtoH %s", error_str);
    }
  image_pub_.publish(msg);
}

}  // namespace nitros_bridge
}  // namespace isaac_ros
}  // namespace nvidia

PLUGINLIB_EXPORT_CLASS(nvidia::isaac_ros::nitros_bridge::ImageConverterNode, nodelet::Nodelet);
