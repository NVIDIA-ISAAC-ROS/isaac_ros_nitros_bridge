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

#ifndef ISAAC_ROS_NITROS_BRIDGE_ROS1__IPC_BUFFER_MANAGER_HPP_
#define ISAAC_ROS_NITROS_BRIDGE_ROS1__IPC_BUFFER_MANAGER_HPP_

#include <cuda_runtime_api.h>
#include <cuda.h>
#include <sys/syscall.h>
#include <unistd.h> 
#include <vector>

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

  // Constructor, Create device memory buffers and export to FD
  IPCBufferManager(size_t size, size_t buffer_step)
  {
    buffer_size_ = size;
    buffer_step_ = buffer_step;

    CUmemAllocationProp prop = {};
    prop.type = CU_MEM_ALLOCATION_TYPE_PINNED;
    prop.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
    prop.location.id = 0;
    prop.requestedHandleTypes = CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR;
    size_t granularity = 0;
    auto cuda_err = cuMemGetAllocationGranularity(
      &granularity, &prop, CU_MEM_ALLOC_GRANULARITY_MINIMUM);
    if (CUDA_SUCCESS != cuda_err) {
      const char *errorStr = NULL;
      cuGetErrorString(cuda_err, &errorStr);
      ROS_ERROR( "Failed to call cuMemGetAllocationGranularity %s", errorStr);
    }
    alloc_size_ = buffer_step - (buffer_step % granularity) + granularity;

    for (size_t i = 0; i < buffer_size_; i++) {
      CUmemGenericAllocationHandle generic_allocation_handle;
      auto cuda_err = cuMemCreate(&generic_allocation_handle, alloc_size_, &prop, 0);
      if (CUDA_SUCCESS != cuda_err) {
        const char *errorStr = NULL;
        cuGetErrorString(cuda_err, &errorStr);
        ROS_ERROR( "Failed to call cuMemCreate %s", errorStr);
      }

      int fd = -1;
      cuda_err = cuMemExportToShareableHandle(
        reinterpret_cast<void *>(&fd),
        generic_allocation_handle,
        CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR, 0);
      if (CUDA_SUCCESS != cuda_err) {
        const char *errorStr = NULL;
        cuGetErrorString(cuda_err, &errorStr);
        ROS_ERROR( "Failed to call cuMemExportToShareableHandle %s", errorStr);
      }

      CUdeviceptr d_ptr = 0ULL;
      cuda_err = cuMemAddressReserve(&d_ptr, alloc_size_, 0, 0, 0);
      if (CUDA_SUCCESS != cuda_err) {
        const char *errorStr = NULL;
        cuGetErrorString(cuda_err, &errorStr);
        ROS_ERROR( "Failed to call cuMemCreate %s", errorStr);
      }

      cuda_err = cuMemMap(d_ptr, alloc_size_, 0, generic_allocation_handle, 0);
      if (CUDA_SUCCESS != cuda_err) {
        const char *errorStr = NULL;
        cuGetErrorString(cuda_err, &errorStr);
        ROS_ERROR("Failed to call cuMemCreate %s", errorStr);
      }

      CUmemAccessDesc accessDesc = {};
      accessDesc.location.type = CU_MEM_LOCATION_TYPE_DEVICE;
      accessDesc.location.id = 0;
      accessDesc.flags = CU_MEM_ACCESS_FLAGS_PROT_READWRITE;
      cuda_err = cuMemSetAccess(d_ptr, alloc_size_, &accessDesc, 1);
      if (CUDA_SUCCESS != cuda_err) {
        const char *errorStr = NULL;
        cuGetErrorString(cuda_err, &errorStr);
        ROS_ERROR( "Failed to call cuMemSetAccess %s", errorStr);
      }

      buffer_ptrs_.push_back(d_ptr);
      shareable_handles_.push_back({getpid(), fd});
      generic_handles.push_back(generic_allocation_handle);
    }
  }

  // Destructor, free the alloacted device memory pool
  ~IPCBufferManager()
  {
    for (size_t i = 0; i < buffer_size_; i++) {
      cuMemRelease(generic_handles[i]);
      cuMemUnmap(buffer_ptrs_[i], alloc_size_);
    }
  }

  // Move the index to next available device memory block
  void next()
  {
    current_handle_index_ += 1;
    if (current_handle_index_ == buffer_size_) {
      current_handle_index_ = 0;
    }
  }

  // Get the device pointer to the current device memory block
  CUdeviceptr get_cur_buffer_ptr()
  {
    return buffer_ptrs_[current_handle_index_];
  }

  // Get the FD exported from current device memory block
  std::vector<int> &get_cur_ipc_mem_handle()
  {
    return shareable_handles_[current_handle_index_];
  }

private:
  size_t buffer_size_;
  size_t buffer_step_;
  size_t current_handle_index_ = 0;
  size_t alloc_size_;

  std::vector<std::vector<int>> shareable_handles_;
  std::vector<CUmemGenericAllocationHandle> generic_handles;
  std::vector<CUdeviceptr> buffer_ptrs_;
};

}  // namespace nitros_bridge
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_NITROS_BRIDGE_ROS1__IPC_BUFFER_MANAGER_HPP_
