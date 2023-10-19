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

#ifndef ISAAC_ROS_NITROS_BRIDGE_ROS2__IPC_BUFFER_MANAGER_HPP_
#define ISAAC_ROS_NITROS_BRIDGE_ROS2__IPC_BUFFER_MANAGER_HPP_

#include <cuda_runtime_api.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace nitros_bridge
{

class IPCBufferManager
{
public:
  IPCBufferManager() = default;

  // Constructor, Create device memory buffers and get IPC handles
  IPCBufferManager(size_t size, size_t buffer_step)
  {
    buffer_size_ = size;
    buffer_step_ = buffer_step;
    cudaIpcMemHandle_t * ipc_mem_handle;

    for (size_t i = 0; i < buffer_size_; i++) {
      void * buffer_ptr = nullptr;
      auto cuda_err = cudaMalloc(&buffer_ptr, buffer_step_);
      if (cudaSuccess != cuda_err) {
        RCLCPP_ERROR(
          rclcpp::get_logger("IPCBufferManager"), "Failed to call cudaMalloc %s",
          cudaGetErrorString(cuda_err));
      }
      ipc_mem_handle = new cudaIpcMemHandle_t;
      cuda_err = cudaIpcGetMemHandle(ipc_mem_handle, buffer_ptr);
      if (cudaSuccess != cuda_err) {
        RCLCPP_ERROR(
          rclcpp::get_logger("IPCBufferManager"), "Failed to get CUDA IPC handle %s",
          cudaGetErrorString(cuda_err));
      }
      ipc_mem_handles_.push_back(ipc_mem_handle);
      buffer_ptrs_.push_back(buffer_ptr);
    }
  }

  // Destrutor, free the alloacted device memory pool
  ~IPCBufferManager()
  {
    for (size_t i = 0; i < buffer_size_; i++) {
      cudaFree(buffer_ptrs_[i]);
    }
  }

  // Move the index to next available device memory block
  void next()
  {
    current_handle_index_ += 1;
    if (current_handle_index_ == buffer_size_) {
      current_handle_index_ = 0;
      RCLCPP_DEBUG(rclcpp::get_logger("IPCBufferManager"), "Reset buffer manager index to start.");
    }
  }

  // Get the device pointer to the current device memory block
  void * get_cur_buffer_ptr()
  {
    return buffer_ptrs_[current_handle_index_];
  }

  // Get the CUDA IPC handle points to current device memory block
  cudaIpcMemHandle_t * get_cur_ipc_mem_handle()
  {
    return ipc_mem_handles_[current_handle_index_];
  }

private:
  size_t buffer_size_;
  size_t buffer_step_;
  size_t current_handle_index_ = 0;
  std::vector<cudaIpcMemHandle_t *> ipc_mem_handles_;
  std::vector<void *> buffer_ptrs_;
};

}  // namespace nitros_bridge
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS_BRIDGE_ROS2__IPC_BUFFER_MANAGER_HPP_
