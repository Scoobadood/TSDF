#ifndef TSDF_KERNEL_HPP
#define TSDF_KERNEL_HPP

#include <cstdint>

typedef struct {
    float m11, m21, m31, m41;
    float m12, m22, m32, m42;
    float m13, m23, m33, m43;
    float m14, m24, m34, fm44;
} Mat44;


__global__
void integrate_kernel(  float * m_voxels, const dim3& voxel_grid_size, const float3& voxel_space_size, const float3& offset,
                        const Mat44& pose,
                        uint32_t width, uint32_t height, const uint16_t * depth_map);

#endif
