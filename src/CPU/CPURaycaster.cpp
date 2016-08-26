//
//  CPURaycaster.cpp
//  KinFu
//
//  Created by Dave on 16/06/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include "CPURaycaster.hpp"
#include "../Utilities/Definitions.hpp"

namespace phd {

const float TRUNC_DIST_PROPORTION_PER_STEP = 0.8f;

#pragma mark - Raycasting

/**
 * Compute the normal to the ISO surface at the given point
 * Based on http://www.cs.technion.ac.il/~veredc/openfusion/OpenFusionReport.pdf
 * @param point The point; should be inside the TSDF
 * @param normal The returned normal
 */
void CPURaycaster::normal_at_point( const CPUTSDFVolume & volume, const Eigen::Vector3f & point, Eigen::Vector3f & normal ) const {
    using namespace Eigen;

    Vector3i voxel = volume.point_to_voxel( point );

    Vector3i lower_index;
    Vector3i upper_index;
    for( int i=0; i<3; i++ ) {
        lower_index[i] = std::max( voxel[i]-1, 0);
        upper_index[i] = std::min( voxel[i]+1, volume.size()[i] - 1);
    }

    Vector3f upper_values {
        volume.distance( upper_index.x(), voxel.y(), voxel.z() ),
        volume.distance( voxel.x(), upper_index.y(), voxel.z() ),
        volume.distance( voxel.x(), voxel.y(), upper_index.z() )
    };

    Vector3f lower_values {
        volume.distance( lower_index.x(), voxel.y(), voxel.z() ),
        volume.distance( voxel.x(), lower_index.y(), voxel.z() ),
        volume.distance( voxel.x(), voxel.y(), lower_index.z() )
    };

    normal = upper_values - lower_values;
    if( normal.norm() != 0 ) {
        normal.normalize();
    }
}


/**
 * Walk ray from start to end seeking intersection with the ISO surface in this TSDF
 * If an intersection is found, return the coordnates in vertex and the surface normal
 * in normal
 * @param volume The volume to be rendered
 * @param ray_start The origin of the ray to be traced
 * @param ray_directioon The direction of the ray to be traced
 * @param vertex The returned vertex
 * @param normal The returned normal
 * @return true if the ray intersects the ISOSurface in which case vertex and normal are populated or else false if not
 */
bool CPURaycaster::walk_ray( const CPUTSDFVolume & volume, const Eigen::Vector3f & ray_start, const Eigen::Vector3f & ray_direction, Eigen::Vector3f & vertex, Eigen::Vector3f & normal) const {
    using namespace Eigen;

    // Set if we intersect ISO surface along this ray
    bool found_intersection = false;

    // Maximum (default) step size is less than truncation distance
    float max_step_size = volume.truncation_distance() * TRUNC_DIST_PROPORTION_PER_STEP;

    // Find the point at which the ray enters the grid - if it does
    float t;
    Vector3f current_point;
    if( volume.is_intersected_by_ray( ray_start, ray_direction, current_point, t ) ) {

        // Assume default step
        float step_size = max_step_size;

        // Walk the ray until we hit a zero crossing or fall out of the grid
        float current_sdf = volume.trilinearly_interpolate_sdf_at( current_point );

        // Store previous point
        float    previous_sdf   = current_sdf;



        // Only do this if the current distnce is positive
        if( current_sdf >= 0 ) {
            bool done = false;

            // Ray walking loop
            do {
                previous_sdf = current_sdf;

                // Step along ray to get new global position
                t = t + step_size;
                current_point = ray_start + (ray_direction * t );

                // If we go out of bounds, end. Otherwise...
                if( volume.point_is_in_tsdf( current_point) ) {

                    // Get voxel containing this point as current and extract the distance
                    current_sdf = volume.trilinearly_interpolate_sdf_at( current_point );

                    // If the distance is negative, we've had a zero crossing
                    if( current_sdf <= 0 ) {

                        // Our assumption is that sdf_current is -ve and sdf_previous is positive
                        if( previous_sdf < 0 ) {
                            std::cerr << "Ray walking found sdfs with incorrect signs" << std::endl;
                            done = true;
                        }

                        // Now perform linear interpolation between these values
                        float step_forward = step_size * ( previous_sdf / ( previous_sdf - current_sdf ));
                        t -= step_size; // undo the last step we took
                        t += step_forward; // and step forward again slightly less

                        // Compute vertex
                        vertex = ray_start + (t * ray_direction);
                        if( volume.point_is_in_tsdf(vertex)) {
                            normal_at_point(volume, vertex, normal );
                            found_intersection = true;
                        }

                        // And we're done
                        done = true;
                    } else { // distance to surface was positive or 0
                        // So long as it's not zero, set up the step size
                        if( current_sdf > 0 ) {
                            step_size = std::max( 1.0f, (current_sdf * max_step_size) );
                        }
                    }
                } else { // Point no longer inside grid
                    done = true;
                }
            } while( !done );
        } // Starting distance was valid
    } // Ray doesn't intersect Volume

    return found_intersection;
}


/**
 * Raycast the TSDF and store discovered vertices and normals in the ubput arrays
 * @param volume The volume to cast
 * @param camera The camera
 * @param vertices The vertices discovered
 * @param normals The normals
 */
void CPURaycaster::raycast( const CPUTSDFVolume & volume, const Camera & camera,
                            Eigen::Matrix<float, 3, Eigen::Dynamic> & vertices,
                            Eigen::Matrix<float, 3, Eigen::Dynamic> & normals ) const {
    using namespace Eigen;

    vertices.resize( 3, m_width * m_height );
    normals.resize( 3, m_width * m_height );

    // Ray origin is at camera position in world coords
    Vector3f ray_start = camera.position();

    // For each pixel u ∈ output image do
    size_t pixel_index = 0;
    for( int y=0; y<m_height; y++ ) {
        for( int x =0; x<m_width; x++ ) {

            // Obtain a unit vector in the direction of the ray
            // Backproject the pixel (x, y, 1mm) into global space - NB Z axis is negative in front of camera
            Vector2f camera_coord = camera.pixel_to_image_plane( x, y );
            Vector3f ray_next = camera.camera_to_world( Vector3f { camera_coord.x(), camera_coord.y(), -1 } );
            Vector3f ray_direction = (ray_next - ray_start).normalized();

            // Walk the ray to obtain vertex and normal values
            Vector3f normal;
            Vector3f vertex;
            bool ray_insersects_surface = walk_ray( volume, ray_start, ray_direction, vertex, normal);

            // If the ray doesn't intersect, create a BAD_VERTEX
            if( !ray_insersects_surface ) {
                vertex = BAD_VERTEX;
                normal = Vector3f::Zero();
            }

            // Transform normal and vertex back into pose space
            for( int i=0; i<3; i++ ) {
                vertices(i, pixel_index ) = vertex[i];
                normals( i, pixel_index) = normal[i];
            }
            pixel_index++;
        } // End for Y
    } // End For X
} // End function

}
