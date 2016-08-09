//
//  CPUCPUTSDFVolume.cpp
//  TSDF
//
//  Created by Dave on 11/03/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include <stdexcept>
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <boost/weak_ptr.hpp>

#include "CPUTSDFVolume.hpp"
#include "CPURaycaster.hpp"
#include "../Utilities/Definitions.hpp"
#include "../Utilities/TSDFLoader.hpp"

namespace phd {

CPUTSDFVolume::~CPUTSDFVolume() {
    std::cout << "CPUTSDFVolume::dtor called. On entry m_voxels:= " << m_voxels << std::endl;
    if ( m_voxels ) {
        delete [] m_voxels;
        m_voxels = 0;
    }
    if ( m_weights ) {
        delete [] m_weights;
        m_weights = 0;
    }
}

/**
 * Constructor with specified number of voxels in each dimension
 * @param size
 * @param physical_size
 */
CPUTSDFVolume::CPUTSDFVolume( const Eigen::Vector3i & size, const Eigen::Vector3f & physical_size ) : m_offset { Eigen::Vector3f::Zero()}, m_voxels {NULL}, m_weights {NULL} {
    std::cout << "CPUTSDFVolume::ctor called. On entry m_voxels:= " << m_voxels << std::endl;

    m_voxels = NULL;
    m_weights = NULL;
    if ( ( size.x() > 0 ) && ( size.y() > 0 ) && ( size.z() > 0 ) && ( physical_size.x() > 0 ) && ( physical_size.y() > 0 ) && ( physical_size.z() > 0 ) ) {
        set_size( size.x(), size.y(), size.z() , physical_size.x(), physical_size.y(), physical_size.z() );
    } else {
        throw std::invalid_argument( "Attempt to construct CPUTSDFVolume with zero or negative size" );
    }

    std::cout << "CPUTSDFVolume::ctor called. On exit m_voxels:= " << m_voxels << std::endl;
}


/**
 * Set the size of the volume. This will delete any existing values and resize the volume, clearing it when done.
 * Volume offset is maintained
 * @param volume_x X dimension in voxels
 * @param volume_y Y dimension in voxels
 * @param volume_z Z dimension in voxels
 * @param psize_x Physical size in X dimension in mm
 * @param psize_y Physical size in Y dimension in mm
 * @param psize_z Physical size in Z dimension in mm
 */
void CPUTSDFVolume::set_size( uint16_t volume_x, uint16_t volume_y, uint16_t volume_z, float psize_x, float psize_y, float psize_z) {
    using namespace Eigen;
    std::cout << "CPUTSDFVolume::set_size called. On entry m_voxels:= " << m_voxels << std::endl;
    if ( ( volume_x != 0 && volume_y != 0 && volume_z != 0 ) && ( psize_x != 0 && psize_y != 0 && psize_z != 0 ) ) {


        // Remove existing data
        if ( m_voxels ) {
            delete [] m_voxels;
            m_voxels = 0;
        }
        if ( m_weights ) {
            delete [] m_weights;
            m_weights = 0;
        }


        m_size = Vector3i { volume_x, volume_y, volume_z };

        m_x_size = volume_x;
        m_xy_slice_size = volume_x * volume_y;

        m_physical_size = Vector3f { psize_x, psize_y, psize_z };

        // Compute truncation distance - must be at least 2x max voxel size
        float cx = m_physical_size[0] / m_size[0];
        float cy = m_physical_size[1] / m_size[1];
        float cz = m_physical_size[2] / m_size[2];

        m_voxel_size = Eigen::Vector3f( cx, cy, cz );

        // Set t > diagonal of voxel
        m_truncation_distance = 1.1f * m_voxel_size.norm();

        // Create the volume storage
        m_voxels  = new float[volume_x * volume_y * volume_z];
        if ( !m_voxels) {
            throw std::bad_alloc( );
        }
        m_weights = new float[volume_x * volume_y * volume_z];
        if ( !m_weights ) {
            delete [] m_voxels;
            throw std::bad_alloc( );
        }

        clear();

        // Max weight for integrating depth images
        m_max_weight = 20.0f;

    } else {
        throw std::invalid_argument( "Attempt to set TSDF size to zero" );
    }
    std::cout << "CPUTSDFVolume::set_size called. On exit m_voxels:= " << m_voxels << std::endl;

}


#pragma mark - Coordinate manipulation

/**
 * @param voxel The voxel to consider
 * @param lower_bounds Coordinates (in global) of rear left lower corner of voxel
 * @param upper_bounds Coordinates (in global) of front right upper corner of voxel
 * @throw std::invalid_argument if voxel coords are out of range
 */
void CPUTSDFVolume::voxel_bounds( const Eigen::Vector3i & voxel, Eigen::Vector3f & lower_bounds, Eigen::Vector3f & upper_bounds ) const {
    if ( contains_voxel( voxel ) ) {
        lower_bounds.x() = voxel.x() * m_voxel_size.x();
        upper_bounds.x() = lower_bounds.x() + m_voxel_size.x();

        lower_bounds.y() = voxel.y() * m_voxel_size.y();
        upper_bounds.y() = lower_bounds.y() + m_voxel_size.y();

        lower_bounds.z() = voxel.z() * m_voxel_size.z();
        upper_bounds.z() = lower_bounds.z() + m_voxel_size.z();
    } else {
        throw std::invalid_argument( "Attempt to access invalid voxel" );
    }
}


/**
 * @param x The horizontal coord (0-width-1)
 * @param y The vertical coord (0-height - 1)
 * @param z The depth coord (0-depth - 1)
 * @return The coordinate of the centre of the given voxel in world coords (mm)
 */
Eigen::Vector3f CPUTSDFVolume::centre_of_voxel_at( int x, int y, int z ) const {
    using namespace Eigen;

    Vector3f centre {
        (x + 0.5f) * m_voxel_size[0] + m_offset[0],
        (y + 0.5f) * m_voxel_size[1] + m_offset[1],
        (z + 0.5f) * m_voxel_size[2] + m_offset[2]
    };
    return centre;
}

/**
 * @return The coordinate of the centre of the given voxel in world coords (mm)
 */
Eigen::Vector3f CPUTSDFVolume::centre_of_volume(  ) const {
    using namespace Eigen;

    return (m_physical_size / 2.0f) + m_offset;
}


/**
 * Convert a point in global coordinates into voxel coordinates
 * Logs an error if the point is outside of the grid by a sufficient margin
 * @param point The point to obtain as a voxel
 * @return voxel The voxel coordinate containing that point
 */
Eigen::Vector3i CPUTSDFVolume::point_to_voxel( const Eigen::Vector3f & point) const {

    // Convert from global to Volume coords
    Eigen::Vector3f grid_point = point - m_offset;

    // FRactional voxel
    Eigen::Vector3f fractional_voxel = grid_point.array() / m_voxel_size.array();

    Eigen::Vector3i voxel;
    voxel.x() = std::floor( fractional_voxel.x() );
    voxel.y() = std::floor( fractional_voxel.y() );
    voxel.z() = std::floor( fractional_voxel.z() );

    bool bad_point = false;
    for ( int i = 0; i < 3; i++ ) {
        if ( ( grid_point[i] < -0.01) || ( grid_point[i] > m_physical_size[i] + 0.01 ) ) {
            bad_point = true;
            break;
        }
    }

    if ( ! bad_point ) {
        for ( int i = 0; i < 3; i++ ) {
            voxel [i] = std::min( std::max( 0, voxel[i]), m_size[i] - 1);
        }
    } else {
        std::cerr << "point_to_voxel called for a point too far outside volume " << point << std::endl ;
    }

    return voxel;
}


/**
 * @return the size of this space.
 */
Eigen::Vector3i CPUTSDFVolume::size( ) const {
    return m_size;
}



/**
 * @return the dimensions of each voxel in mm
 */
Eigen::Vector3f CPUTSDFVolume::voxel_size( ) const {
    return m_voxel_size;
}

/**
 * @return the physical size of the volume in world coords (mm)
 */
Eigen::Vector3f CPUTSDFVolume::physical_size( ) const {
    return m_physical_size;
}

/**
 * @return the truncation distance (mm)
 */
float CPUTSDFVolume::truncation_distance( ) const {
    return m_truncation_distance;
}

/**
 * Offset the TSDF volume in space by the given offset. By default, the bottom, left, front corner of
 * voxel (0,0,0) is at world coordinate (0,0,0). This moves that point to the new world coordinate by a
 * @param ox X offset in mm
 * @param oy Y offset in mm
 * @param oz Z offset in mm
 */
void CPUTSDFVolume::offset( float ox, float oy, float oz ) {
    m_offset = Eigen::Vector3f(ox, oy, oz);
}

/**
 * @return the offset f the TSDF volume in space
 */
Eigen::Vector3f CPUTSDFVolume::offset( ) const {
    return m_offset;
}

/**
 * @retrun true if the given point is contained in this TSDF volume
 */
bool CPUTSDFVolume::point_is_in_tsdf( const Eigen::Vector3f & point ) const {
    using namespace Eigen;

    Vector3f gridmax = m_physical_size + m_offset;
    bool is_in = (( point.x() >= m_offset.x()) && ( point.x() <= gridmax.x()) &&
                  ( point.y() >= m_offset.y()) && ( point.y() <= gridmax.y()) &&
                  ( point.z() >= m_offset.z()) && ( point.z() <= gridmax.z()));
    return is_in;
}

/**
 * Find the point where the given ray first intersects the TSDF space in global coordinates
 * @param origin The source of the ray
 * @param ray_direction A unit vector in the direction of the ray
 * @param entry_point The point at which the ray enters the TSDF which may be the origin
 * @param t The ray parameter for the intersection; entry_point = origin + (t * ray_direction)
 * @return true if the ray intersects the TSDF otherwise false
 */
bool CPUTSDFVolume::is_intersected_by_ray( const Eigen::Vector3f & origin, const Eigen::Vector3f & ray_direction, Eigen::Vector3f & entry_point, float & t ) const {
    // Check for the case where the origin is inside the voxel grid
    if ( point_is_in_tsdf(origin) ) {
        entry_point = origin;
        t = 0.0f;
        return true;
    }



    float t_near = std::numeric_limits<float>::lowest( );
    float t_far = std::numeric_limits<float>::max();

    bool intersects = true;

    for ( int dim = 0; dim < 3; dim++ ) {
        if (  ray_direction[dim] == 0 ) {
            if ( ( origin[dim] < m_offset[dim] ) || ( origin[dim] > (m_offset[dim] + m_physical_size[dim] ) ) ) {
                intersects = false;
                break;
            }
        } else {
            // compute intersection distance of the planes
            float t1 = ( m_offset[dim]                        - origin[dim] ) / ray_direction[dim];
            float t2 = ( m_offset[dim] + m_physical_size[dim] - origin[dim] ) / ray_direction[dim];

            // If t1 > t2 swap (t1, t2) since t1 intersection with near plane
            if ( t1 > t2 ) {
                float temp_t = t1;
                t1 = t2;
                t2 = temp_t;
            }

            // if t1 > t_near set t_near = t1 : We want largest t_near
            if ( t1 > t_near ) {
                t_near = t1;
            }

            //If t2 < t_far set t_far="t2"  want smallest t_far
            if ( t2 < t_far ) {
                t_far = t2;
            }

            // If Tnear > Tfar box is missed so return false
            if ( t_near > t_far ) {
                intersects = false;
                break;
            }


            // If Tfar < 0 box is behind ray return false end
            if ( t_far < 0 ) {
                intersects = false;
                break;
            }
        }
    } // end of all dims
    // If Box survived all above tests, return true with intersection point Tnear and exit point Tfar.
    t = t_near;
    entry_point = origin + ( t * ray_direction);

    return intersects;
}


#pragma mark - Data access
/**
 * Clear the voxel and weight data
 */
void CPUTSDFVolume::clear( ) {
    size_t maxIdx = m_size[0] * m_size[1] * m_size[2];
    for ( size_t idx = 0; idx < maxIdx; idx ++ ) {
        m_voxels[idx] =  0.0f;
        m_weights[idx] = 0.0f;
    }
}

/**
 * @param x The horizontal voxel coord
 * @param y The vertical voxel coord
 * @param z The depth voxel coord
 * @return The distance to the surface at that voxel
 */
float CPUTSDFVolume::distance( int x, int y, int z ) const {
    return m_voxels[ index(x, y, z) ];
}

/**
 * @param x The horizontal voxel coord
 * @param y The vertical voxel coord
 * @param z The depth voxel coord
 * @param distance The distance to set
 * @return The distance to the surface at that voxel
 */
void CPUTSDFVolume::set_distance( int x, int y, int z, float distance ) {
    size_t idx = index( x, y, z );
    m_voxels[ idx ] = distance;
}


/**
 * @param x The horizontal voxel coord
 * @param y The vertical voxel coord
 * @param z The depth voxel coord
 * @return The weight at that voxel
 */
float CPUTSDFVolume::weight( int x, int y, int z ) const {
    return m_weights[ index(x, y, z) ];
}

/**
 * @param x The horizontal voxel coord
 * @param y The vertical voxel coord
 * @param z The depth voxel coord
 * @param weight The weight to set
 * @return The weight at that voxel
 */
void CPUTSDFVolume::set_weight( int x, int y, int z, float weight ) {
    m_weights[ index(x, y, z) ] = weight;
}


/**
 * @return the length (number of elements) in this space. Used to index
 * raw data returned by other member functions
 */
size_t CPUTSDFVolume::length( ) const {
    return m_size[0] * m_size[1] * m_size[2];
}


/**
 * Get the upper and lower bounding voxels for a trilinear interpolation at the given point in
 * global space.
 * @param point The point in global coordinates
 * @param lower_bound The voxel forming the lower, left, near bound
 * @param upper_bound The voxel forming the upper, right, far bound
 */
void CPUTSDFVolume::get_interpolation_bounds( const Eigen::Vector3f & point, Eigen::Vector3i & lower_bounds, Eigen::Vector3i & upper_bounds ) const {
    using namespace Eigen;

    // Obtain current voxel
    Vector3i current_voxel = point_to_voxel(point);


    // And central point
    Vector3f voxel_centre = centre_of_voxel_at( current_voxel.x(), current_voxel.y(), current_voxel.z() );

    // For each coordinate axis, determine whether point is below or above and
    // select the appropriate bounds
    for ( int i = 0; i < 3; i++ ) {
        if ( point[i] < voxel_centre[i] ) {
            // Point is left/below/nearer so UPPER bound in this direction is current_voxel
            upper_bounds[i] = current_voxel[i];
            lower_bounds[i] = std::max( 0, current_voxel[i] - 1 );
        } else {
            // Point is right/above/further so LOWER bound in this direction is current_voxel
            lower_bounds[i] = current_voxel[i];
            upper_bounds[i] = std::min( current_voxel[i] + 1, m_size[i] - 1);
        }
    }
}

/**
 * Trilinearly interpolate the point p in the voxel grid using the tsdf values
 * of the surrounding voxels. At edges we assume that the voxel values continue
 * @param point the point
 * @return the interpolated value
 */
float CPUTSDFVolume::trilinearly_interpolate_sdf_at( const Eigen::Vector3f & point ) const {
    using namespace Eigen;

    float value = 0;

    // Determine which voxels bound this point
    Vector3i lower_bounds;
    Vector3i upper_bounds;
    get_interpolation_bounds(point, lower_bounds, upper_bounds);

    // Compute uvw
    Vector3f centre_of_lower_bound = centre_of_voxel_at(lower_bounds.x(), lower_bounds.y() , lower_bounds.z() );
    Vector3f uvw = (point - centre_of_lower_bound).array() / m_voxel_size.array();


    // Local vars for clarity
    float u = uvw[0];
    float v = uvw[1];
    float w = uvw[2];

    float u_prime = 1.0f - u;
    float v_prime = 1.0f - v;
    float w_prime = 1.0f - w;

    float c000 = distance( lower_bounds.x(), lower_bounds.y(), lower_bounds.z() );
    float c001 = distance( lower_bounds.x(), lower_bounds.y(), upper_bounds.z() );
    float c010 = distance( lower_bounds.x(), upper_bounds.y(), lower_bounds.z() );
    float c011 = distance( lower_bounds.x(), upper_bounds.y(), upper_bounds.z() );
    float c100 = distance( upper_bounds.x(), lower_bounds.y(), lower_bounds.z() );
    float c101 = distance( upper_bounds.x(), lower_bounds.y(), upper_bounds.z() );
    float c110 = distance( upper_bounds.x(), upper_bounds.y(), lower_bounds.z() );
    float c111 = distance( upper_bounds.x(), upper_bounds.y(), upper_bounds.z() );

    // Interpolate X
    float c00 = c000 * u_prime + c100 * u;
    float c01 = c001 * u_prime + c101 * u;
    float c10 = c010 * u_prime + c110 * u;
    float c11 = c011 * u_prime + c111 * u;

    // Interpolate Y
    float c0 = c00 * v_prime + c10 * v;
    float c1 = c01 * v_prime + c11 * v;

    // Interpolate Z
    value = c0 * w_prime + c1 * w;

    return value;
}

/**
 * Return a pointer to the TSDF data ordered Slice,Col,Row major
 * @return the data
 */
const float * CPUTSDFVolume::data() const {
    return m_voxels;
}

/**
 * Set the data
 */
void CPUTSDFVolume::set_distance_data( const float * distance_data ) {
    size_t data_size = m_size[0] * m_size[1] * m_size[2] * sizeof( float );
    memcpy( m_voxels, distance_data, data_size );
}
/**
 * Set the data
 */
void CPUTSDFVolume::set_weight_data( const float * weight_data ) {
    size_t data_size = m_size[0] * m_size[1] * m_size[2] * sizeof( float );
    memcpy( m_weights, weight_data, data_size );
}


#pragma mark - Integrate new depth data
/**
 * Integrate a range map into the TSDF
 * This follows the approach in Cohen, N.S.V. 2013, 'Open Fusion', pp. 1–35.
 * whereby new maps have less weight than existing maps
 * @param depth_map Pointer to width*height depth values where 0 is an invalid depth and positive values are expressed in mm
 * @param width The horiontal dimension of the depth_map
 * @param height The height of the depth_map
 * @param camera The camera from which the depth_map was taken
 */
void CPUTSDFVolume::integrate( const uint16_t * depth_map, uint32_t width, uint32_t height, const Camera & camera ) {
    using namespace Eigen;

    // First convert the depth_map into CAMERA space
    Matrix<float, 3, Dynamic> vertices;
    Matrix<float, 3, Dynamic> normals;
    camera.depth_image_to_vertices_and_normals(depth_map, width, height, vertices, normals);

    // For each voxel in the space
    for ( int vy = 0; vy < m_size.y(); vy++ ) {
        for ( int vx = 0; vx < m_size.x(); vx++ ) {
            for ( int vz = 0; vz < m_size.z(); vz++ ) {


                // Work out where in the image, the centre of this voxel projects
                // This gives us a pixel in the depth map
                Vector3f centre_of_voxel = centre_of_voxel_at( vx, vy, vz );
                Vector2i cov_in_pixels = camera.world_to_pixel( centre_of_voxel);
                uint16_t voxel_pixel_x = cov_in_pixels.x();
                uint16_t voxel_pixel_y = cov_in_pixels.y();

                // if this point is in the camera view frustum...
                if ( ( voxel_pixel_x >= 0 && voxel_pixel_x < width ) && ( voxel_pixel_y >= 0 && voxel_pixel_y < height) ) {
                    uint32_t voxel_image_index = voxel_pixel_y * width + voxel_pixel_x;

                    // Extract the depth to the surface at this point
                    uint16_t surface_depth = depth_map[ voxel_image_index ];

                    // If the depth is valid
                    if ( surface_depth > 0 ) {

                        // Get the voxel centre in cam coords
                        Vector3f voxel_centre_in_cam = camera.world_to_camera(centre_of_voxel);
                        float voxel_distance = voxel_centre_in_cam.norm();

                        Vector3f surface_vertex{ vertices( 0, voxel_image_index), vertices( 1, voxel_image_index), vertices(2, voxel_image_index) };
                        float surface_distance = surface_vertex.norm();

                        // SDF is the difference between them.
                        float sdf = surface_distance - voxel_distance;

                        // Truncate
                        float tsdf;
                        if ( sdf > 0 ) {
                            tsdf = std::fminf( 1.0, sdf / m_truncation_distance);
                        } else {
                            tsdf = std::fmaxf( -1.0, sdf / m_truncation_distance);
                        }

                        // Extract prior weight
                        float prior_weight = weight( vx, vy, vz );
                        float current_weight = 1.0f;

                        float prior_distance = distance( vx, vy, vz );

                        float new_weight = std::min( prior_weight + current_weight, m_max_weight );
                        float new_distance = ( (prior_distance * prior_weight) + (tsdf * current_weight) ) / new_weight;

                        set_weight(vx, vy, vz, new_weight);
                        set_distance(vx, vy, vz, new_distance);
                    }
                } // Voxel depth <= 0
            }
        }
    }
}

#pragma mark - Import/Export

/**
 * Save the TSDF to file
 * @param The filename
 * @return true if the file saved OK otherwise false.
 */
bool CPUTSDFVolume::save_to_file( const std::string & file_name) const {
    using namespace std;

    bool success = false;

    // Open file
    ofstream ofs;
    ofs.open( file_name, std::ios::out);
    if ( ofs.is_open() ) {
        ofs << fixed << setprecision(3);

        // Write Dimensions
        ofs << "voxel size = " << m_size.x() << " " << m_size.y() << " " << m_size.z() << std::endl;
        ofs << "space size = " << m_physical_size.x() << " " << m_physical_size.y() << " " << m_physical_size.z() << std::endl;

        // Write data
        for ( uint16_t y = 0; y < m_size.y() ; y++ ) {
            for ( uint16_t x = 0; x < m_size.x() ; x++ ) {
                ofs << std::endl << "# y " << y << ", x " << x << " tsdf" << std::endl;

                for ( uint16_t z = 0; z < m_size.z() ; z++ ) {
                    uint64_t idx = index( x, y, z ) ;

                    ofs << m_voxels[idx] << " ";
                }

                ofs << std::endl << "# y " << y << ", x " << x << " weights" << std::endl;
                for ( uint16_t z = 0; z < m_size.z() ; z++ ) {
                    uint64_t idx = index( x, y, z ) ;
                    ofs  << m_weights[ idx ] << " ";
                }
            }
        }

        // Close file
        ofs.close();
        success = true;
    } else {
        std::cout << "Couldn't save TSDF, error opening file for write " << file_name << std::endl;
    }

    return success;
}


/**
 * Load the given TSDF file
 * @param The filename
 * @return true if the file saved OK otherwise false.
 */
bool CPUTSDFVolume::load_from_file( const std::string & file_name) {
    TSDFLoader loader( this );
    return loader.load_from_file( file_name );
}

#pragma mark - Rendering
/**
 * @param width The width of the camera view
 * @param height The height of the camera view
 * @param camera The camera
 * @param vertices A Dynamic matrix of 3D vectors represnting the vertices of the object
 * @param normals  A Dynamic matrix of 3D vectors represnting the vertices of the object
 */
void CPUTSDFVolume::raycast( uint16_t width, uint16_t height,
                             const Camera& camera,
                             Eigen::Matrix<float, 3, Eigen::Dynamic>& vertices,
                             Eigen::Matrix<float, 3, Eigen::Dynamic>& normals ) const {
    CPURaycaster raycaster( width, height );
    raycaster.raycast(*this, camera, vertices, normals);
}

}