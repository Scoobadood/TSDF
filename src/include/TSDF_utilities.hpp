

#ifndef TSDF_UTILITIES_HPP
#define TSDF_UTILITIES_HPP

#include <cstdint>

extern "C" {
	/**
	* @param x The horizontal coord (0-width-1)
	* @param y The vertical coord (0-height - 1)
	* @param z The depth coord (0-depth - 1)
	* @return The coordinate of the centre of the given voxel in world coords (mm)
	*/
	__device__
	float3 centre_of_voxel_at( int x, int y, int z, const float3& voxel_size, const float3& offset=float3{0.0f, 0.0f, 0.0f});


	/**
	 * @param x The voxel x coord
	 * @param y The voxel y coord
	 * @param z The voxel z coord
	 * @param tsdf_values The values
	 * @param voxel_grid_size The size of the voxel grid
	 * @return the value at voxel (x,y,z)
	 */
	__device__
	float tsdf_value_at( uint16_t x, uint16_t y, uint16_t z, const float * const tsdf_values, const dim3 voxel_grid_size );

	/**
	 * Determine the voxel in which a point lies
	 * @param point The point in voxel space coordinates (0,0,0) -> (max_x, max_y, max_z)
	 * @return The voxel in which the point lies.
	 */
	__device__
	int3 voxel_for_point( const float3 point, const float3 voxel_size );
}


#endif
