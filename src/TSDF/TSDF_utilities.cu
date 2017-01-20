#include "../include/TSDF_utilities.hpp"

/**
* @param x The horizontal coord (0-width-1)
* @param y The vertical coord (0-height - 1)
* @param z The depth coord (0-depth - 1)
* @return The coordinate of the centre of the given voxel in world coords (mm)
*/
__device__
float3 centre_of_voxel_at( int x, int y, int z, const float3& voxel_size, const float3& offset )  {
    float3 centre {
        (x + 0.5f) * voxel_size.x + offset.x,
        (y + 0.5f) * voxel_size.y + offset.y,
        (z + 0.5f) * voxel_size.z + offset.z
    };
    return centre;
}


/**
 * @param x The voxel x coord
 * @param y The voxel y coord
 * @param z The voxel z coord
 * @param tsdf_values The values
 * @param voxel_grid_size The size of the voxel grid
 * @return the value at voxel (x,y,z)
 */
__device__
float tsdf_value_at( uint16_t x, uint16_t y, uint16_t z, const float * const tsdf_values, const dim3 voxel_grid_size ) {
    // Force out of bounds coords back in
    x = min( max(x,0), voxel_grid_size.x-1);
    y = min( max(y,0), voxel_grid_size.y-1);
    z = min( max(z,0), voxel_grid_size.z-1);

    size_t idx = voxel_grid_size.x * voxel_grid_size.y * z + voxel_grid_size.x * y + x;
    return tsdf_values[idx];
}

/**
 * Determine the voxel in which a point lies
 * @param point The point in voxel space coordinates (0,0,0) -> (max_x, max_y, max_z)
 * @return The voxel in which the point lies.
 */
__device__
int3 voxel_for_point( const float3 point, const float3 voxel_size ) {
    int3 voxel {
        int(floor( point.x / voxel_size.x )),
        int(floor( point.y / voxel_size.y )),
        int(floor( point.z / voxel_size.z ))
    };

    return voxel;
}