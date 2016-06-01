
//
//  TSDFVolume.cpp
//  TSDF
//
//  Created by Dave on 11/03/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include <pcl/io/ply_io.h>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <boost/weak_ptr.hpp>

#include "TSDFVolume.hpp"
#include "MarchingCubes.hpp"
#include "Cubic.hpp"

namespace phd {
    
    /**
     * Predefined bad vertex
     */
    static const Eigen::Vector3f BAD_VERTEX{ std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };
    
    TSDFVolume::~TSDFVolume() {
        if( m_voxels ) {
            delete [] m_voxels;
            m_voxels = 0;
        }
        if( m_weights ) {
            delete [] m_weights;
            m_weights = 0;
        }
    }
    
    /**
     * Constructor with specified number of voxels in each dimension
     * @param size
     * @param physical_size
     */
    TSDFVolume::TSDFVolume( const Eigen::Vector3i & size, const Eigen::Vector3f & physical_size ) : m_offset{ Eigen::Vector3f::Zero()}{
        if( ( size.x() > 0 ) && ( size.y() > 0 ) && ( size.z() > 0 ) && ( physical_size.x() > 0 ) && ( physical_size.y() > 0 ) && ( physical_size.z() > 0 ) ) {
            set_size( size.x(), size.y(), size.z() , physical_size.x(), physical_size.y(), physical_size.z() );
        } else {
            throw std::invalid_argument( "Attempt to construct TSDFVOlume with zero or negative size" );
        }
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
    void TSDFVolume::set_size( uint16_t volume_x, uint16_t volume_y, uint16_t volume_z, float psize_x, float psize_y, float psize_z) {
        using namespace Eigen;
        
        if( ( volume_x != 0 && volume_y != 0 && volume_z != 0 ) && ( psize_x != 0 && psize_y != 0 && psize_z != 0 ) ) {
            
            
            // Remove existing data
            if( m_voxels ) {
                delete [] m_voxels;
                m_voxels = 0;
            }
            if( m_weights ) {
                delete [] m_weights;
                m_weights = 0;
            }
            
            
            m_size = Vector3i{ volume_x, volume_y, volume_z };
            
            m_x_size = volume_x;
            m_xy_slice_size = volume_x * volume_y;
            
            m_physical_size = Vector3f{ psize_x, psize_y, psize_z };
            
            // Compute truncation distance - must be at least 2x max voxel size
            float cx = m_physical_size[0] / m_size[0];
            float cy = m_physical_size[1] / m_size[1];
            float cz = m_physical_size[2] / m_size[2];
            
            m_voxel_size = Eigen::Vector3f( cx, cy, cz );
            
            // Set t > diagonal of voxel
            m_truncation_distance = 1.1f * m_voxel_size.norm();
            
            // Create the volume storage
            m_voxels  = new float[volume_x * volume_y * volume_z];
            m_weights = new float[volume_x * volume_y * volume_z];
            
            assert( m_voxels );
            assert( m_weights );
            clear();
            
            // Max weight for integrating depth images
            m_max_weight = 20.0f;
            
        } else {
            throw std::invalid_argument( "Attempt to set TSDF size to zero" );
        }
        
    }
    
    
#pragma mark - Coordinate manipulation
    
    /**
     * @param voxel The voxel to consider
     * @param lower_bounds Coordinates (in global) of rear left lower corner of voxel
     * @param upper_bounds Coordinates (in global) of front right upper corner of voxel
     * @throw std::invalid_argument if voxel coords are out of range
     */
    void TSDFVolume::voxel_bounds( const Eigen::Vector3i & voxel, Eigen::Vector3f & lower_bounds, Eigen::Vector3f & upper_bounds ) const {
        if( contains_voxel( voxel ) ) {
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
    Eigen::Vector3f TSDFVolume::centre_of_voxel_at( int x, int y, int z ) const {
        using namespace Eigen;
        
        Vector3f centre{
            (x + 0.5f) * m_voxel_size[0] + m_offset[0],
            (y + 0.5f) * m_voxel_size[1] + m_offset[1],
            (z + 0.5f) * m_voxel_size[2] + m_offset[2]};
        return centre;
    }
    
    /**
     * @return The coordinate of the centre of the given voxel in world coords (mm)
     */
    Eigen::Vector3f TSDFVolume::centre_of_volume(  ) const {
        using namespace Eigen;
        
        return (m_physical_size / 2.0f) + m_offset;
    }
    
    
    /**
     * Convert a point in global coordinates into voxel coordinates
     * Logs an error if the point is outside of the grid by a sufficient margin
     * @param point The point to obtain as a voxel
     * @return voxel The voxel coordinate containing that point
     */
    Eigen::Vector3i TSDFVolume::point_to_voxel( const Eigen::Vector3f & point) const {
        
        // Convert from global to Volume coords
        Eigen::Vector3f grid_point = point - m_offset;
        
        // FRactional voxel
        Eigen::Vector3f fractional_voxel = grid_point.array() / m_voxel_size.array();
        
        Eigen::Vector3i voxel;
        voxel.x() = std::floor( fractional_voxel.x() );
        voxel.y() = std::floor( fractional_voxel.y() );
        voxel.z() = std::floor( fractional_voxel.z() );
        
        bool bad_point = false;
        for( int i=0; i<3; i++ ) {
            if( ( grid_point[i] < -0.01) || ( grid_point[i] > m_physical_size[i] + 0.01 ) ) {
                bad_point = true;
                break;
            }
        }
        
        if( ! bad_point ) {
            for( int i=0; i<3; i++ ) {
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
    Eigen::Vector3i TSDFVolume::size( ) const {
        return m_size;
    }
    
    
    
    /**
     * @return the dimensions of each voxel in mm
     */
    Eigen::Vector3f TSDFVolume::voxel_size( ) const {
        return m_voxel_size;
    }
    
    /**
     * @return the physical size of the volume in world coords (mm)
     */
    Eigen::Vector3f TSDFVolume::physical_size( ) const {
        return m_physical_size;
    }
    
    /**
     * @return the truncation distance (mm)
     */
    float TSDFVolume::truncation_distance( ) const {
        return m_truncation_distance;
    }
    
    /**
     * Offset the TSDF volume in space by the given offset. By default, the bottom, left, front corner of
     * voxel (0,0,0) is at world coordinate (0,0,0). This moves that point to the new world coordinate by a
     * @param ox X offset in mm
     * @param oy Y offset in mm
     * @param oz Z offset in mm
     */
    void TSDFVolume::offset( float ox, float oy, float oz ) {
        m_offset = Eigen::Vector3f(ox, oy, oz);
    }
    
    /**
     * @retrun true if the given point is contained in this TSDF volume
     */
    bool TSDFVolume::point_is_in_tsdf( const Eigen::Vector3f & point ) const {
        using namespace Eigen;
        
        Vector3f gridmax = m_physical_size + m_offset;
        bool is_in = (( point.x() >= m_offset.x()) && ( point.x() <= gridmax.x()) &&
                      ( point.y() >= m_offset.y()) && ( point.y() <= gridmax.y()) &&
                      ( point.z() >= m_offset.z()) && ( point.z() <= gridmax.z()));
        return is_in;
    }
    
    
#pragma mark - Data access
    /**
     * Clear the voxel and weight data
     */
    void TSDFVolume::clear( ) {
        size_t maxIdx = m_size[0] * m_size[1] * m_size[2];
        for( size_t idx = 0; idx < maxIdx; idx ++ ) {
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
    float TSDFVolume::distance( int x, int y, int z ) const {
        return m_voxels[ index(x, y, z) ];
    }
    
    /**
     * @param x The horizontal voxel coord
     * @param y The vertical voxel coord
     * @param z The depth voxel coord
     * @param distance The distance to set
     * @return The distance to the surface at that voxel
     */
    void TSDFVolume::set_distance( int x, int y, int z, float distance ) {
        size_t idx =index( x, y, z );
        m_voxels[ idx ] = distance;
    }
    
    
    /**
     * @param x The horizontal voxel coord
     * @param y The vertical voxel coord
     * @param z The depth voxel coord
     * @return The weight at that voxel
     */
    float TSDFVolume::weight( int x, int y, int z ) const {
        return m_weights[ index(x, y, z) ];
    }
    
    /**
     * @param x The horizontal voxel coord
     * @param y The vertical voxel coord
     * @param z The depth voxel coord
     * @param weight The weight to set
     * @return The weight at that voxel
     */
    void TSDFVolume::set_weight( int x, int y, int z, float weight ) {
        m_weights[ index(x, y, z) ] = weight;
    }
    
    
    /**
     * @return the length (number of elements) in this space. Used to index
     * raw data returned by other member functions
     */
    size_t TSDFVolume::length( ) const {
        return m_size[0] * m_size[1] * m_size[2];
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
    void TSDFVolume::integrate( const uint16_t * depth_map, uint32_t width, uint32_t height, const Camera & camera ) {
        using namespace Eigen;
        
        // First convert the depth_map into CAMERA space
        std::deque<Vector3f> vertices;
        std::deque<Vector3f> normals;
        camera.depth_image_to_vertices_and_normals(depth_map, width, height, vertices, normals);
        
        // For each voxel in the space
        for( int vy=0; vy<m_size.y(); vy++ ) {
            for( int vx=0; vx<m_size.x(); vx++ ) {
                for( int vz=0; vz<m_size.z(); vz++ ) {
                    
                    
                    // Work out where in the image, the centre of this voxel projects
                    // This gives us a pixel in the depth map
                    Vector3f centre_of_voxel = centre_of_voxel_at( vx, vy, vz );
                    Vector2i cov_in_pixels = camera.world_to_pixel( centre_of_voxel);
                    uint16_t voxel_pixel_x = cov_in_pixels.x();
                    uint16_t voxel_pixel_y = cov_in_pixels.y();
                    
                    // if this point is in the camera view frustum...
                    if( ( voxel_pixel_x >= 0 && voxel_pixel_x < width ) && ( voxel_pixel_y >= 0 && voxel_pixel_y < height) ) {
                        uint32_t voxel_image_index = voxel_pixel_y * width + voxel_pixel_x;
                        
                        // Extract the depth to the surface at this point
                        uint16_t surface_depth = depth_map[ voxel_image_index ];
                        
                        // If the depth is valid
                        if( surface_depth > 0 ) {
                            
                            // Get the voxel centre in cam coords
                            Vector3f voxel_centre_in_cam = camera.world_to_camera(centre_of_voxel);
                            float voxel_distance = std::fabs(voxel_centre_in_cam.z());
                            
                            // Compute conversion factor
                            Vector2f pixel_in_cam = camera.pixel_to_image_plane(voxel_pixel_x, voxel_pixel_y);
                            float lambda = std::sqrt( pixel_in_cam.x()*pixel_in_cam.x() + pixel_in_cam.y()*pixel_in_cam.y() + 1 );
                            
                            // Convert voxel distance to a depth
                            float voxel_depth = voxel_distance / lambda;
                            
                            // SDF is the difference between them.
                            float sdf = surface_depth - voxel_depth;
                            
                            // Truncate
                            float tsdf;
                            if( sdf > 0 ) {
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
    
    
    
    
    
    /**
     * Extract the ISO Surface corresponding to the 0 crossing
     * as a mesh
     */
    pcl::PolygonMesh TSDFVolume::extractSISOSurface( ) const {
        return do_marching_cubes(*this);
    }
    
#pragma mark - Raycasting
    
    /**
     * Compute the normal to the ISO surface at the given point
     * Based on http://www.cs.technion.ac.il/~veredc/openfusion/OpenFusionReport.pdf
     * @param point The point; should be inside the TSDF
     * @param normal The returned normal
     */
    void TSDFVolume::normal_at_point( const Eigen::Vector3f & point, Eigen::Vector3f & normal ) const {
        using namespace Eigen;
        
        Vector3i voxel = point_to_voxel( point );
        
            Vector3i lower_index;
            Vector3i upper_index;
            for( int i=0; i<3; i++ ) {
                lower_index[i] = std::max( voxel[i]-1, 0);
                upper_index[i] = std::min( voxel[i]+1, m_size[i] - 1);
            }
            
            Vector3f upper_values{ distance( upper_index.x(), voxel.y(), voxel.z() ),
                distance( voxel.x(), upper_index.y(), voxel.z() ),
                distance( voxel.x(), voxel.y(), upper_index.z() ) };
            Vector3f lower_values{ distance( lower_index.x(), voxel.y(), voxel.z() ),
                distance( voxel.x(), lower_index.y(), voxel.z() ),
                distance( voxel.x(), voxel.y(), lower_index.z() ) };
            
            normal = upper_values - lower_values;
            normal.normalize();
    }
    
    /**
     * Find the point where the given ray first intersects the TSDF space in global coordinates
     * @param origin The source of the ray
     * @param ray_direction A unit vector in the direction of the ray
     * @param entry_point The point at which the ray enters the TSDF which may be the origin
     * @param t The ray parameter for the intersection; entry_point = origin + (t * ray_direction)
     * @return true if the ray intersects the TSDF otherwise false
     */
    bool TSDFVolume::is_intersected_by_ray( const Eigen::Vector3f & origin, const Eigen::Vector3f & ray_direction, Eigen::Vector3f & entry_point, float & t ) const {
        float t_near = std::numeric_limits<float>::lowest( );
        float t_far = std::numeric_limits<float>::max();
        
        bool intersects = true;
        
        for ( int dim=0; dim<3; dim++ ) {
            if(  ray_direction[dim] == 0 ) {
                if( ( origin[dim] < m_offset[dim] ) || ( origin[dim] > (m_offset[dim] + m_physical_size[dim] ) ) ) {
                    intersects = false;
                    break;
                }
            } else {
                // compute intersection distance of the planes
                float t1 = ( m_offset[dim]                        - origin[dim] ) / ray_direction[dim];
                float t2 = ( m_offset[dim] + m_physical_size[dim] - origin[dim] ) / ray_direction[dim];
                
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
                    intersects = false;
                    break;
                }
                
                
                // If Tfar < 0 box is behind ray return false end
                if( t_far < 0 ) {
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
    
    
    
    /**
     * Get the upper and lower bounding voxels for a trilinear interpolation at the given point in
     * global space.
     * @param point The point in global coordinates
     * @param lower_bound The voxel forming the lower, left, near bound
     * @param upper_bound The voxel forming the upper, right, far bound
     */
    void TSDFVolume::get_interpolation_bounds( const Eigen::Vector3f & point, Eigen::Vector3i & lower_bounds, Eigen::Vector3i & upper_bounds ) const {
        using namespace Eigen;
        
        // Obtain current voxel
        Vector3i current_voxel = point_to_voxel(point);
        
        
            // And central point
            Vector3f voxel_centre = centre_of_voxel_at( current_voxel.x(), current_voxel.y(), current_voxel.z() );
            
            // For each coordinate axis, determine whether point is below or above and
            // select the appropriate bounds
            for( int i=0; i<3; i++ ) {
                if( point[i] < voxel_centre[i] ) {
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
    float TSDFVolume::trilinearly_interpolate_sdf_at( const Eigen::Vector3f & point ) const {
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
     * Walk ray from start to end seeking intersection with the ISO surface in this TSDF
     * If an intersection is found, return the coordnates in vertex and the surface normal
     * in normal
     
     
     for each pixel u ∈ output image in parallel do
     raystart ← back project [u, 0]; convert to grid pos
     raynext ← back project [u, 1]; convert to grid pos
     raydir ← normalize (raynext − raystart)
     raylen ← 0
     g ← first voxel along raydir
     m ← convert global mesh vertex to grid pos mdist ← ||raystart − m||
     
     while voxel g within volume bounds do
     raylen ← raylen + 1
     gprev ← g
     g ← traverse next voxel along raydir
     if zero crossing from g to gprev then
     p ← extract trilinear interpolated grid position
     v ← convert p from grid to global 3D position
     n ← extract surface gradient as ∇tsdf (p)
     shade pixel for oriented point (v, n) or
     follow secondary ray (shadows, reflections, etc)
     
     if raylen > mdist then
     shade pixel using inputed mesh maps or
     follow secondary ray (shadows, reflections, etc)
     */
    bool TSDFVolume::walk_ray( const Eigen::Vector3f & ray_start, const Eigen::Vector3f & ray_direction, Eigen::Vector3f & vertex, Eigen::Vector3f & normal) const {
        using namespace Eigen;
        
        bool values_returned = false;
        
        // Find the point at which the ray enters the grid - if it does
        float t;
        Vector3f current_point;
        if( is_intersected_by_ray( ray_start, ray_direction, current_point, t ) ) {
            
            Vector3i current_voxel = point_to_voxel( current_point);
            
            // Walk the ray until we hit a zero crossing or fall out of the grid
            Vector3i previous_voxel;
            Vector3f previous_position = current_point;
            float step_size = m_truncation_distance;
            
            // Only do this if the current distnce is positive
            float distance_to_surface =distance(current_voxel.x(), current_voxel.y(), current_voxel.z() );
            if( distance_to_surface >= 0 ) {
                bool done = false;
                
                
                
                // Ray walking loop
                do {
                    previous_voxel = current_voxel;
                    
                    
                    // Step along ray to get new global position
                    t = t + step_size;
                    Vector3f new_position = ray_start + (ray_direction * t );
                    
                    // If we go out of bounds, end. Otherwise...
                    if( point_is_in_tsdf(new_position) ) {
                        
                        // Get voxel containing this point as current and extract the distance
                        current_voxel = point_to_voxel( new_position );
                        distance_to_surface = distance(current_voxel.x(), current_voxel.y(), current_voxel.z() );
                        
                        // If the distance is negative, we've had a zero crossing
                        if( distance_to_surface < 0 ) {
                            
                            // Obtain more precise distances for last and current position using trilinear interp
                            float sdf_current = trilinearly_interpolate_sdf_at( new_position);
                            float sdf_previous = trilinearly_interpolate_sdf_at( previous_position );
                            
                            // Our assumption is that sdf_current is -ve and sdf_previous is positive
                            if( sdf_current >= 0 || sdf_previous <=0 ) {
                                //std::cerr << "Ray walking found sdfs with incorrect signs" << std::endl;
                            }
                            
                            // Now perform linear interpolation between these values
                            float step_forward = step_size * ( sdf_previous / ( sdf_previous - sdf_current ));
                            t -= step_size; // undo the last step we took
                            t += step_forward; // and step forward again slightly less
                            
                            // Compute vertex
                            vertex = ray_start + (t * ray_direction);
                            if( point_is_in_tsdf(vertex)) {
                                normal_at_point(vertex, normal );
                                values_returned = true;
                            }
                            
                            // And we're done
                            done = true;
                        } else { // distance to surface was positive or 0
                            // So long as it's not zero, set up the step size
                            if( distance_to_surface > 0 ) {
                                step_size = (distance_to_surface * m_truncation_distance);
                            }
                        }
                    } else { // Point no longer inside grid
                        done = true;
                    }

                    // Save the position
                    previous_position = new_position;
                } while( !done );
            } // Starting distance was valid
        } // Ray doesn't intersect Volume
        
        return values_returned;
    }
    
    
    /**
     * Generate a raycast surface
     * @param camera The camera doing the rendering
     * @param width The width of the output image
     * @param height The height of the output image
     * @param vertex_map A pointer to an array of width * height vertices in World Coordinate system
     * @param normal_map A pointer to an array of width * height normals in World Coordinate system
     */
    void TSDFVolume::raycast( const Camera & camera, uint16_t width, uint16_t height,
                             Eigen::Vector3f * vertex_map,
                             Eigen::Vector3f * normal_map) const {
        using namespace Eigen;
        
        // Ray origin is at camera position in world coords
        Vector3f ray_start = camera.position();
        
        // For each pixel u ∈ output image do
        for( uint16_t y=0; y<height; y++ ) {
            for( uint16_t x =0; x<width; x++ ) {
                
                // Obtain a unit vector in the direction of the ray
                // Backproject the pixel (x, y, 1mm) into global space - NB Z axis is negative in front of camera
                Vector2f camera_coord = camera.pixel_to_image_plane(x, y);
                Vector3f ray_next = camera.camera_to_world( Vector3f{ camera_coord.x(), camera_coord.y(), -1 } );
                Vector3f ray_direction = (ray_next - ray_start).normalized();
                
                // Walk the ray to obtain vertex and normal values
                Vector3f normal;
                Vector3f vertex;
                bool ray_insersects_surface = walk_ray( ray_start, ray_direction, vertex, normal);
                
                // If the ray doesn't intersect, create a BAD_VERTEX
                if( !ray_insersects_surface ) {
                    vertex = BAD_VERTEX;
                    normal = Vector3f::Zero();
                }
                
                
                // Transform normal and vertex back into pose space
                vertex_map[ y * width + x ] = vertex;
                normal_map[ y * width + x ] = normal;
            } // End for Y
        } // End For X
    } // End function
    
#pragma mark - Import/Export
    
    /**
     * Save the TSDF to file
     * @param The filename
     * @return true if the file saved OK otherwise false.
     */
    bool TSDFVolume::save_to_file( const std::string & file_name) const {
        using namespace std;
        
        // Open file
        ofstream ofs{ file_name };
        ofs << fixed << setprecision(3);
        
        // Write Dimensions
        ofs << "voxel size = " << m_size.x() << " " << m_size.y() << " " << m_size.z() << std::endl;
        ofs << "space size = " << m_physical_size.x() << " " << m_physical_size.y() << " " << m_physical_size.z() << std::endl;
        
        // Write data
        for( uint16_t y = 0; y< m_size.y() ; y++ ) {
            for( uint16_t x = 0; x< m_size.x() ; x++ ) {
                ofs << std::endl << "# y "<< y << ", x " << x << " tsdf" << std::endl;
                
                for( uint16_t z = 0; z< m_size.z() ; z++ ) {
                    size_t idx = index( x, y, z ) ;
                    
                    ofs << m_voxels[ idx ] << " ";
                }
                
                ofs << std::endl << "# y "<< y << ", x " << x << " weights" << std::endl;
                for( uint16_t z = 0; z< m_size.z() ; z++ ) {
                    size_t idx = index( x, y, z ) ;
                    ofs  << m_weights[ idx ] << " ";
                }
            }
        }
        
        // Close file
        ofs.close();
        return true;
    }
    
    /**
     * Load the given TSDF file
     * @param The filename
     * @return true if the file saved OK otherwise false.
     */
    bool TSDFVolume::load_from_file( const std::string & file_name) {
        
        // Open file
        std::ifstream ifs{ file_name, std::ifstream::in };
        
        if( ifs.good() ) {
            
            // Read Dimensions
            uint16_t size_x, size_y, size_z;
            std::string line;
            
            
            std::getline(ifs,line);
            std::stringstream iss(line);
            std::string rubbish;
            std::getline(iss, rubbish, '=');
            iss >> size_x >> size_y >> size_z;
            
            
            // Read physical size
            float psize_x, psize_y, psize_z;
            if( std::getline(ifs,line) ) {
                std::stringstream iss(line);
                std::getline(iss, rubbish, '=');
                iss >> psize_x >> psize_y >> psize_z;
            }
            
            
            // Allocate storage
            set_size( size_x, size_y, size_z, psize_x, psize_y, psize_z);
            
            std::cout << "         Size : " << size_x << ", " << size_y << ", " << size_z << std::endl;
            std::cout << "Physical Size : " << psize_x << ", " << psize_y << ", " << psize_z << std::endl;
            std::cout << std::endl;
            for( uint16_t y = 0; y<size_y ; y++ ) {
                std::cout <<  ".";
                std::cout.flush();
                for( uint16_t x = 0; x< size_x ; x++ ) {
                    
                    // Discard tsdf comment
                    std::getline( ifs, line );
                    
                    
                    // Get distances
                    for( uint16_t z=0; z<size_z; z++ ) {
                        float distance;
                        ifs >> distance;
                        set_distance( x, y, z, distance );
                    }
                    
                    // Discard weights comment
                    std::getline( ifs, line );
                    
                    // Get weights
                    for( uint16_t z=0; z<size_z; z++ ) {
                        float weight;
                        ifs >> weight;
                        set_weight( x, y, z, weight );
                    }
                }
            }
            std::cout << std::endl;
            
        } else {
            // File not found or wouldn't open
            std::cerr << "File error " << file_name << std::endl;
        }
        return true;
    }
}

