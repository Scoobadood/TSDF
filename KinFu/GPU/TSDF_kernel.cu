#include "TSDF_kernel.hpp"
#include <cfloat>

__global__
void integrate_kernel(  float * m_voxels, const dim3& voxel_grid_size, const float3& voxel_space_size, const float3& offset,
                        const Mat44& pose,
                        uint32_t width, uint32_t height, const uint16_t * depth_map) {

}


#pragma mark - Coordinate manipulation

/**
* @param voxel The voxel to consider
* @param lower_bounds Coordinates (in global) of rear left lower corner of voxel
* @param upper_bounds Coordinates (in global) of front right upper corner of voxel
* @throw std::invalid_argument if voxel coords are out of range
*/
__device__
void voxel_bounds( const dim3& voxel, float3& lower_bounds, float3& upper_bounds ) {
}


/**
* @param x The horizontal coord (0-width-1)
* @param y The vertical coord (0-height - 1)
* @param z The depth coord (0-depth - 1)
* @return The coordinate of the centre of the given voxel in world coords (mm)
*/
__device__
float3 centre_of_voxel_at( const float3& voxel_size, const float3& offset, int x, int y, int z )  {
    float3 centre {
        (x + 0.5f) * voxel_size.x + offset.x,
        (y + 0.5f) * voxel_size.y + offset.y,
        (z + 0.5f) * voxel_size.z + offset.z
    };
    return centre;
}

/**
* @return The coordinate of the centre of the given voxel in world coords (mm)
*/
__device__
float3 centre_of_volume( const float3& physical_size, const float3 offset ) {
    return float3{
               (physical_size.x / 2.0f) + offset.x,
               (physical_size.y / 2.0f) + offset.y,
               (physical_size.z / 2.0f) + offset.z
           };
}


/**
* Convert a point in global coordinates into voxel coordinates
* Logs an error if the point is outside of the grid by a sufficient margin
* @param point The point to obtain as a voxel
* @return voxel The voxel coordinate containing that point
*/
__device__
dim3 point_to_voxel( const float3& m_physical_size, const int3& m_size, const float3& voxel_size,  const float3& point, const float3& offset) {

    // Convert from global to Volume coords
    float3 grid_point{
        point.x - offset.x,
        point.y - offset.y,
        point.z - offset.z
    };

    // FRactional voxel
    float3 fractional_voxel{
        grid_point.x / voxel_size.x,
        grid_point.y / voxel_size.y,
        grid_point.z / voxel_size.z,
    };

    dim3 voxel{
        static_cast<unsigned int>(floor( fractional_voxel.x )),
        static_cast<unsigned int>(floor( fractional_voxel.y )),
        static_cast<unsigned int>(floor( fractional_voxel.z ))
    };

    bool bad_point = ( grid_point.x < -0.01) || ( grid_point.x > m_physical_size.x + 0.01 ) ||
                     ( grid_point.y < -0.01) || ( grid_point.y > m_physical_size.y + 0.01 ) ||
                     ( grid_point.z < -0.01) || ( grid_point.z > m_physical_size.z + 0.01 ) ;


    if( ! bad_point ) {
        voxel.x = min( max( 0, voxel.x), m_size.x - 1 );
        voxel.y = min( max( 0, voxel.y), m_size.y - 1 );
        voxel.z = min( max( 0, voxel.z), m_size.z - 1 );
    } else {
        // Error point
        voxel.x = voxel.y = voxel.z = INT_MAX;
    }

    return voxel;
}

/**
* @retrun true if the given point is contained in this TSDF volume
*/
__device__ __forceinline__
bool point_is_in_tsdf( const float3& m_offset, const float3& m_physical_size, const float3& point ) {
    float3 gridmax { m_physical_size.x + m_offset.x,
                     m_physical_size.y + m_offset.y,
                     m_physical_size.z + m_offset.z};

    bool is_in = (( point.x >= m_offset.x) && ( point.x <= gridmax.x) &&
                  ( point.y >= m_offset.y) && ( point.y <= gridmax.y) &&
                  ( point.z >= m_offset.z) && ( point.z <= gridmax.z));
    return is_in;
}


__device__
bool could_intersect( const float& m_offset, const float& m_physical_size, const float& origin, const float& ray_direction, float& t_near, float& t_far) {
    bool could_intersect = true;

    if(  ray_direction == 0 ) {
        if( ( origin < m_offset ) || ( origin > (m_offset + m_physical_size ) ) ) {
            could_intersect = false;
        }
    } else {
        // compute intersection distance of the planes
        float t1 = ( m_offset                   - origin ) / ray_direction;
        float t2 = ( m_offset + m_physical_size - origin ) / ray_direction;

        // If t1 > t2 swap (t1, t2) since t1 intersection with near plane
        if( t1 > t2 ) {
            float temp_t = t1;
            t1 = t2;
            t2 = temp_t;
        }

        // if t1 > t_near set t_near = t1 : We want largest t_near
        if( t1 > t_near ) {
            t_near = t1;
        }

        //If t2 < t_far set t_far="t2"  want smallest t_far
        if( t2 < t_far ) {
            t_far = t2;
        }

        // If Tnear > Tfar box is missed so return false
        if( t_near > t_far ) {
            could_intersect = false;
        }


        // If Tfar < 0 box is behind ray return false end
        if( t_far < 0 ) {
            could_intersect = false;
        }
    }

    return could_intersect;
}

/**
* Find the point where the given ray first intersects the TSDF space in global coordinates
* @param origin The source of the ray
* @param ray_direction A unit vector in the direction of the ray
* @param entry_point The point at which the ray enters the TSDF which may be the origin
* @param t The ray parameter for the intersection; entry_point = origin + (t * ray_direction)
* @return true if the ray intersects the TSDF otherwise false
*/
__device__
bool is_intersected_by_ray( const float3& m_offset, const float3& m_physical_size, const float3& origin, const float3& ray_direction, float3& entry_point, float& t )  {
    bool intersects = false;


    // Check for the case where the origin is inside the voxel grid
    if( point_is_in_tsdf(m_offset, m_physical_size, origin) ) {
        intersects = true;
        entry_point = origin;
        t = 0.0f;
    } else {

        float t_near = FLT_MIN;
        float t_far = FLT_MAX;

        // Check X
        if ( could_intersect( m_offset.x, m_physical_size.x, origin.x, ray_direction.x, t_near, t_far ) &&
                could_intersect( m_offset.y, m_physical_size.y, origin.y, ray_direction.y, t_near, t_far ) &&
                could_intersect( m_offset.z, m_physical_size.z, origin.z, ray_direction.z, t_near, t_far )  ) {
            intersects = true;
            t = t_near;
            entry_point.x = origin.x + ( t * ray_direction.x);
            entry_point.y = origin.y + ( t * ray_direction.y);
            entry_point.z = origin.z + ( t * ray_direction.z);
        }
    }

    return intersects;
}


__device__
/**
* Clear the voxel and weight data
*/
void clear( ) {
}

/**
* Get the upper and lower bounding voxels for a trilinear interpolation at the given point in
* global space.
* @param point The point in global coordinates
* @param lower_bound The voxel forming the lower, left, near bound
* @param upper_bound The voxel forming the upper, right, far bound
*/
__device__
void get_interpolation_bounds( const float3& m_physical_size, const int3& m_size, const float3& voxel_size,  const float3& point, const float3& offset, dim3& lower_bounds, dim3 & upper_bounds ) {
    // Obtain current voxel
    dim3 current_voxel = point_to_voxel(m_physical_size, m_size, voxel_size,  point, offset );


    // And central point
    float3 voxel_centre = centre_of_voxel_at( m_physical_size, offset, current_voxel.x, current_voxel.y, current_voxel.z );

    // For each coordinate axis, determine whether point is below or above and
    // select the appropriate bounds
    if( point.x < voxel_centre.x ) {
        upper_bounds.x = current_voxel.x;
        lower_bounds.x = max( 0, current_voxel.x - 1);
    } else {
        upper_bounds.x = min( m_size.x - 1, current_voxel.x + 1);
        lower_bounds.x = current_voxel.x;
    }

    if( point.y < voxel_centre.y ) {
        upper_bounds.y = current_voxel.y;
        lower_bounds.y = max( 0, current_voxel.y - 1);
    } else {
        upper_bounds.y = min( m_size.y - 1, current_voxel.y + 1);
        lower_bounds.y = current_voxel.y;
    }

    if( point.z < voxel_centre.z ) {
        upper_bounds.z = current_voxel.z;
        lower_bounds.z = max( 0, current_voxel.z - 1);
    } else {
        upper_bounds.z = min( m_size.z - 1, current_voxel.z + 1);
        lower_bounds.z = current_voxel.z;
    }

}

/**
* Trilinearly interpolate the point p in the voxel grid using the tsdf values
* of the surrounding voxels. At edges we assume that the voxel values continue
* @param point the point
* @return the interpolated value
*/
__device__
float trilinearly_interpolate_sdf_at( const float3& point ) {
    return 0.0f;
}

/**
* Return a pointer to the TSDF data ordered Slice,Col,Row major
* @return the data
*/
__device__
const float * data()  {
    return NULL;
}


