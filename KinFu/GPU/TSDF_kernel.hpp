#ifndef TSDF_KERNEL_HPP
#define TSDF_KERNEL_HPP

#include <cstdint>
#include "cu_common.hpp"

__global__
void integrate_kernel(  float * m_voxels, float * m_weights,
                        dim3 voxel_grid_size, float3 voxel_space_size, float3 offset, const float trunc_distance,
                        Mat44 inv_pose, Mat33 k, Mat33 kinv,
                        uint32_t width, uint32_t height, const uint16_t * depth_map);

#endif
