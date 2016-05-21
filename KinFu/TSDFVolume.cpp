
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
#include <boost/weak_ptr.hpp>

#include "TSDFVolume.hpp"
#include "MarchingCubes.hpp"
#include "Cubic.hpp"

namespace phd {
    
    TSDFVolume::~TSDFVolume() {
        if( m_voxels ) {
            delete m_voxels;
            m_voxels = 0;
        }
        if( m_weights ) {
            delete m_weights;
            m_weights = 0;
        }
    }
    
    /**
     * Constructor with specified number of voxels in each dimension
     * @param size
     * @param physical_size
     */
    TSDFVolume::TSDFVolume( const Eigen::Vector3i & size, const Eigen::Vector3f & physical_size ) : m_size{size}, m_physical_size{physical_size}, m_offset{0.0, 0.0, 0.0}
    {
        
        int volume_x = m_size[0];
        int volume_y = m_size[1];
        int volume_z = m_size[2];
        
        if( ( volume_x > 0 && volume_y > 0 && volume_z > 0 ) &&
           ( m_physical_size[0] > 0 && m_physical_size[1] > 0 && m_physical_size[2] > 0 ) ) {
            
            // Compute truncation distance
            float cx = m_physical_size[0] / m_size[0];
            float cy = m_physical_size[1] / m_size[1];
            float cz = m_physical_size[2] / m_size[2];
            
            // Set minimal truncation distance - must be at least 2x max voxel size
            m_truncation_distance = 2.1f * std::max (cx, std::max (cy, cz));
            
            m_voxel_size = Eigen::Vector3f( cx, cy, cz );
            
            // Create the volume storage
            m_voxels  = new float[volume_x * volume_y * volume_z];
            m_weights = new float[volume_x * volume_y * volume_z];
            
            assert( m_voxels );
            assert( m_weights );
            clear();
            
            // This K is from 3D with Kinect, Jan Smisek, Michal Jancosek and Tomas Pajdla
            m_K.setZero();
            m_K(0,0) = 585.6f;
            m_K(0,2) = 316.0f;
            m_K(1,1) = 585.6f;
            m_K(1,2) = 247.6f;
            m_K(2,2) = 1.0f;
            
            // Compute K_Inverse
            m_K_inv = m_K.inverse();
            
            // Max weight for integrating depth images
            m_max_weight = 10.0f;
            
        } else {
            //        std::cerr << "Attempt to construct TSDFvolume with zero or negative dimension" << std::endl;
            throw std::invalid_argument( "Attempt to construct TSDFvolume with zero or negative dimension" );
        }
    }
    
    
#pragma mark - Coordinate manipulation
    
    /**
     * @param voxel The voxel to consider
     * @param lower_bounds Coordinates (in global) of front left lower corner of voxel
     * @param upper_bounds Coordinates (in global) of back right upper corner of voxel
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
     * Convert a point in the TSDF volume into the corresponding voxel
     * @param point The point to obtain as a voxel
     * @param voxel The voxel coordinate containing that point
     */
    void TSDFVolume::point_to_voxel( const Eigen::Vector3f & point, Eigen::Vector3i & voxel ) const {
        Eigen::Vector3f adjusted_point = point - m_offset;
        adjusted_point = adjusted_point.array() / m_voxel_size.array();
        
        voxel.x() = std::floor( adjusted_point.x() );
        voxel.y() = std::floor( adjusted_point.y() );
        voxel.z() = std::floor( adjusted_point.z() );
        
        // Adjust into bounds for points which are on the far surface in any direction
        for (int i=0; i<3; i++ ) {
            if( voxel[i] == m_size[i]) {
                voxel[i] = m_size[i] - 1;
            } else if( voxel[i] == -1) {
                voxel[i] = 0;
            }
        }
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
            m_voxels[idx] = 0.0f;
            m_weights[idx] = 0.0f;
        }
    }
    
    /**
     * @param x The horizontal voxel coord
     * @param y The vertical voxel coord
     * @param z The depth voxel coord
     * @return The distance to the surface at that voxel
     */
    float & TSDFVolume::distance( int x, int y, int z ) const {
        return m_voxels[ index(x, y, z) ];
    }
    
    
    /**
     * @param x The horizontal voxel coord
     * @param y The vertical voxel coord
     * @param z The depth voxel coord
     * @return The weight at that voxel
     */
    float & TSDFVolume::weight( int x, int y, int z ) const {
        return m_weights[ index(x, y, z) ];
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
        
        // First cnvert the depth_map into global coordinates
        std::deque<Vector3f> vertices;
        std::deque<Vector3f> normals;
        camera.depth_image_to_vertices_and_normals(depth_map, width, height, vertices, normals);
    
        // For each voxel in the space
        for( int vy=0; vy<m_size[1]; vy++ ) {
            for( int vx=0; vx<m_size[0]; vx++ ) {
                for( int vz=0; vz<m_size[2]; vz++ ) {
                    
                    // Get voxel coordinates in global frame
                    Vector3f centre_of_voxel = centre_of_voxel_at( vx, vy, vz );
                    
                    // Convert to image coordinate sin camera
                    Vector2i cov_in_image;
                    camera.world_to_image( centre_of_voxel, cov_in_image);
                    
                    // if this point is in the camera view frustum...
                    if( cov_in_image.x() >= 0 && cov_in_image.x() < width && cov_in_image.y() >= 0 && cov_in_image.y() < height) {
                        
                        // Scanned depth
                        uint16_t scanned_depth = depth_map[ cov_in_image.x() + cov_in_image.y() * width ];
                        
                        // Surely we need to convert this scanned depth to an actal surface vertex and measure distance to that
                        Vector3f surface_vertex;
                        
                        

                        // If the depth is valid
                        if( scanned_depth > 0 ) {
                            // Extract camera origin in global space
                            Vector3f camera_origin{ camera.pose()(0,3), camera.pose()(1,3), camera.pose()(2,3) };

                            // Convert the cov to a 3D vertex in camera space
                            Vector2f cam_point;
                            camera.image_to_camera( cov_in_image.x(), cov_in_image.y(), cam_point );
                            Vector3f surface_vertex{ cam_point.x() * scanned_depth, cam_point.y() * scanned_depth, scanned_depth };
                            
                            // Measured distance is length of the surface_vertex vector
                            float measured_depth = surface_vertex.norm();

                            
                            // Computed distance is from cam origin to voxel centre in global space
                            Vector3f computed_depth_vector = centre_of_voxel - camera_origin;
                            float computed_depth = computed_depth_vector.norm();
                            
                            // SDF is the difference between them.
                            float sdf = measured_depth - computed_depth;
                            
                            // Truncate
                            float tsdf = tsdf = std::min( std::max( sdf, -m_truncation_distance ), m_truncation_distance );
                            
                            // Extract prior weight
                            float prior_weight = weight( vx, vy, vz );
                            float current_weight = 1;
                            
                            float prior_distance = distance( vx, vy, vz );
                            
                            float new_weight = std::min( prior_weight + current_weight, m_max_weight );
                            float new_distance = ( (prior_distance * prior_weight) + (tsdf * current_weight) ) / new_weight;
                            
                            distance( vx, vy, vz ) = new_distance;
                            weight( vx, vy, vz ) = new_weight;
                        }
                    }
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
     * Given an image plane coordinate (x,y) and a depth z, backproject to a point in 3D space
     */
    Eigen::Vector3f TSDFVolume::back_project( const Eigen::Matrix4f & pose, const Eigen::Vector3f & point ) const {
        Eigen::Matrix3f K_inv = m_K.inverse();
        Eigen::Vector3f inter = K_inv * point;
        
        // Now multiply by pose to convert from camera to worl coordinates
        Eigen::Vector4f back_h = pose * Eigen::Vector4f{ inter.x(), inter.y(), inter.z(), 1.0f };
        
        Eigen::Vector3f back{ back_h.x(), back_h.y(), back_h.z() };
        back = back / back_h.w();
        
        return back;
    }
    

    /**
     * Compute the bormal to the ISO surface at the given point
     * Based on http://www.cs.technion.ac.il/~veredc/openfusion/OpenFusionReport.pdf
     * @param point The point; should be inside the TSDF
     * @param normal The returned normal
     */
    void TSDFVolume::normal_at_point( const Eigen::Vector3f & point, Eigen::Vector3f & normal ) const {
        using namespace Eigen;
        
        Vector3i voxel;
        point_to_voxel( point, voxel );
        
        // Get voxel values at voxel (x +/1 1, y +/- 1 z +/-1 )

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
     * @param origin The r=source of the ray
     * @param ray_direction A unit vector in the direction of the ray
     * @param entry_point The point at which the ray enters the TSDF which may be the origin
     * @param t The ray parameter for the intersection; entry_point = origin + (t * ray_direction)
     * @return true if the ray intersects the TSDF otherwise false
     */
    bool TSDFVolume::is_intersected_by_ray_2( const Eigen::Vector3f & origin, const Eigen::Vector3f & ray_direction, Eigen::Vector3f & entry_point, float & t ) const {
        float t_near = std::numeric_limits<float>::min( );
        float t_far = std::numeric_limits<float>::max( );
        
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
            } // end of for loop
            
            // If Box survived all above tests, return true with intersection point Tnear and exit point Tfar.
            t = t_near;
            entry_point = origin + ( t * ray_direction);
            
        }
        
        return intersects;
    }
    
    /**
     * Find the point where the given ray first intersects the TSDF space in global coordinates
     * @param origin The r=source of the ray
     * @param ray_direction A unit vector in the direction of the ray
     * @param entry_point The point at which the ray enters the TSDF which may be the origin
     * @param t The ray parameter for the intersection; entry_point = origin + (t * ray_direction)
     * @return true if the ray intersects the TSDF otherwise false
     */
    bool TSDFVolume::is_intersected_by_ray( const Eigen::Vector3f & origin, const Eigen::Vector3f & ray_direction, Eigen::Vector3f & entry_point, float & t ) const {
        using namespace Eigen;
        
        bool is_intersected = false;
        
        // Handle the case where origin is in grid
        if( point_is_in_tsdf( origin )  ) {
            entry_point = origin;
            is_intersected = true;
        } else {
            // Work out ray intersection with boundary of grid if it exists
            std::vector<float> minT;
            std::vector<Vector3f> intersection;
            
            for( int i=0; i<3; i++ ) {
                if( ray_direction[i] != 0 ) {
                    float t_near = ( m_offset[i]                      - origin[i] ) / ray_direction[i];
                    float t_far  = ( m_offset[i] + m_physical_size[i] - origin[i] ) / ray_direction[i];
                    
                    Vector3f potential_entry_point = origin + (t_near * ray_direction);
                    if(  point_is_in_tsdf( potential_entry_point ) ) {
                        minT.push_back( t_near );
                        intersection.push_back( potential_entry_point );
                    }
                    
                    potential_entry_point = origin + (t_far * ray_direction);
                    if(  point_is_in_tsdf( potential_entry_point ) ) {
                        minT.push_back( t_far );
                        intersection.push_back( potential_entry_point );
                    }
                }
            }
            
            // Now find the lowest minT and adopt that intersection
            if( minT.size() > 0 ) {
                t = minT[0];
                entry_point = intersection[0];
                int i=1;
                while( i < minT.size() ) {
                    if( minT[i] < t ) {
                        entry_point = intersection[i];
                        t = minT[i];
                    }
                    i++;
                }
                
                is_intersected = true;
            }
        }
        return is_intersected;
    }
    
    /**
     * Walk ray from start to end seeking intersection with the ISO surface in this TSDF
     * If an intersection is found, return the coordnates in vertex and the surface normal
     * in normal
     */
    bool TSDFVolume::walk_ray( const Eigen::Vector3f & ray_start, const Eigen::Vector3f & ray_direction, Eigen::Vector3f & vertex, Eigen::Vector3f & normal) const {
        using namespace Eigen;
        
        bool values_returned = false;
        
        // Find the point at which the ray enters the grid
        float t;
        Vector3f current_point;
        if( is_intersected_by_ray_2( ray_start, ray_direction, current_point, t ) ) {
            
            Vector3i voxel;
            float previous_distance_to_surface;
            float previous_step = 0.0f;
            float distance_to_surface = std::numeric_limits<float>::max();

            // Check that this isn't behind the surface already
            point_to_voxel( current_point, voxel );

            bool done = (distance(voxel.x(), voxel.y(), voxel.z() ) < 0 );
            while( ! done ) {
                
                // Read off distance to surface in this voxel
                previous_distance_to_surface = distance_to_surface;
                distance_to_surface = distance(voxel.x(), voxel.y(), voxel.z() );
                
                // If this is > truncation distance we can skip by truncation distance (minus a bit)
                if( distance_to_surface < m_truncation_distance ) {
                    
                    // If distance to surface is positive, skip
                    if( distance_to_surface > 0 ) {
                        previous_step = distance_to_surface;
                        t = t + previous_step;
                        
                    } else { // We crossed the surface
                        // Interpolate between this value and previous value
                        float interpolated_ratio = previous_distance_to_surface / ( previous_distance_to_surface - distance_to_surface );
                        float interpolated_value = interpolated_ratio * previous_step;
                        
                        t = t + interpolated_value;
                        
                        vertex = ray_start + (t * ray_direction);
                        
                        if( point_is_in_tsdf(vertex)) {
                            normal_at_point(vertex, normal );
                            
                            values_returned = true;
                        }
                        
                        // Somehow skipped past iso surface
                        done = true;
                    }
                    
                } else { // We are at least truncation distance away from the surface so we can skip by 80 % of truncation distance
                    previous_step = m_truncation_distance * 0.8f;
                    t = t + previous_step;
                }
                
                if( !done ) {
                    current_point = ray_start + (t * ray_direction);
                    
                    // Check we're still reasonable; point is in TSDF
                    if( point_is_in_tsdf(current_point)) {
                        point_to_voxel( current_point, voxel );
                    } else {
                        done = true;
                    }
                }
            }
        }
        
        return values_returned;
    }
    
    
    /**
     * Generate a raycast surface
     * @param pose The point in world coordinates from which to render
     * @param width The width of the output image
     * @param height The height of the output image
     * @param vertex_map A pointer to an array of width * height vertices in frame of reference of camera
     * @param normal_map A pointer to an array of width * height normals in frame of reference of camera
     */
    void TSDFVolume::raycast( const Eigen::Matrix4f & pose, uint16_t width, uint16_t height,
                             Eigen::Vector3f * vertex_map,
                             Eigen::Vector3f * normal_map) const {
        using namespace Eigen;
        
        Matrix4f pose_inverse = pose.inverse();
        Matrix3f rot = pose_inverse.block<3,3>(0,0);
        
        // For each pixel u ∈ output image do
        for( uint16_t y=0; y<height; y++ ) {
            for( uint16_t x =0; x<width; x++ ) {

                
                
                // Ray origin is at camera position
                Vector3f ray_start{ pose(0,3), pose(1,3), pose(2,3) };
                
                // Backproject the pixel (x, y, 1mm) into global space
                Vector3f ray_next = back_project( pose, Vector3f{ x, y, 1 } );
                
                // Obtain a unit vector in the direction of the ray
                Vector3f ray_direction = (ray_next - ray_start).normalized();
                
                // Walk the ray to obtain vertex and normal values
                // Default normal value is 0
                Vector3f normal{ 0.0, 0.0, 0.0 };
                
                // Default vertex value is maximum range
                Vector3f vertex = ray_start + 8000 * ray_direction;
                
                bool ray_insersects_surface = walk_ray( ray_start, ray_direction, vertex, normal);
                
                Vector3f normal_posed;
                if( ray_insersects_surface ) {
                    normal_posed = rot * normal;
                } else {
                    normal_posed = Vector3f::Zero();
                }
                
                Vector4f vertex_h{ vertex.x(), vertex.y(), vertex.z(), 1.0f };
                Vector4f vertex_posed_h = pose_inverse * vertex_h;
                Vector3f vertex_posed = vertex_posed_h.block<3,1>( 0,0);
                vertex_posed = vertex_posed / vertex_posed_h.w();
                    
                // Transform normal and vertex back into pose space
                vertex_map[ y * width + x ] = vertex_posed;
                normal_map[ y * width + x ] = normal_posed;
            } // End for Y
        } // End For X
    } // End function
}
