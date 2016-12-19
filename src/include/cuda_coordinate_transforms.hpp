#ifndef CUDA_COORDINATE_TRANSFORMS_HPP
#define CUDA_COORDINATE_TRANSFORMS_HPP

#include "cuda_utilities.hpp"
/**
 * Convert global coordinates into pixel coordinates
 * Multiply by pose.inverse(), then K
 * @param world_coordinate The 3D point in world space
 * @return pixel_coordinate The 2D point in pixel space
 */
__device__
int3 world_to_pixel( const float3 & world_coordinate, const Mat44 & inv_pose, const Mat33 & k );

/**
 * Convert a world coordinate into camera space by
 * by multiplying by pose matrix inverse
 * @param world_coordinate The world coordinate to convert
 * @param inv_pose The 4x4 inverse pose matrix
 * @return a 3D coordinate in camera space
 */
__device__
float3 world_to_camera( const float3& world_coordinate, const Mat44& inv_pose );

/**
 * Convert pixel coordinates into world coordinates via a depth
 * Multiply by k.inverse(), project by depth, then pose
 * @param pixel The 2d pixel coordinate
 * @param depth The depth value of
 * @return world coordinate in 3D space
 */
__device__
float3 pixel_to_world( const int3 pixel, const Mat44 & pose, const Mat33 & inv_k, float depth );

/**
 * Convert pixel coordinates into camera coordinates via a depth
 * Multiply by k.inverse(), project by depth
 * @param pixel The 2d pixel coordinate
 * @param depth The depth value of
 * @return world coordinate in 3D space
 */
__device__
float3 pixel_to_camera( const int3 pixel, const Mat33 & inv_k, float depth );

#endif