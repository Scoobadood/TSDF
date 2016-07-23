#include "TSDF_kernel.hpp"
#include "TSDF_utilities.hpp"


#include "math_constants.h"


#include <cfloat>


/**
 * Convert a depth to a 3D vertex in camera space
 */
__device__
float3 depth_to_vertex( uint16_t depth, uint16_t x, uint16_t y, const Mat33& kinv ) {
    float3 vertex{ CUDART_NAN_F , CUDART_NAN_F, CUDART_NAN_F };
    if( depth != 0 ) {
        // Back project the point into camera 3D space using D(x,y) * Kinv * (x,y,1)T
        float3 cam_point{
            kinv.m11 * x + kinv.m12 * y + kinv.m13,
            kinv.m21 * x + kinv.m22 * y + kinv.m23,
            kinv.m31 * x + kinv.m32 * y + kinv.m33
        };

        vertex.x = cam_point.x * -depth;
        vertex.y = cam_point.y * -depth;
        vertex.z = cam_point.z * -depth;
    }

    return vertex;

}
/**
 * Convert a world coordinate into camera space in the camera image
 * by multiplying by pose matrix inverse, then by K then dividing through by
 * z coordinate to project down.
 * @param world_coordinate The world coordinate to convert
 * @param inv_pose The 4x4 inverse pose matrix
 * @param K The camera intrinsic matrix
 * @return a 2D integral pixel value (u,v)
 */
__device__
int3 camera_to_pixel( const float3& camera_coordinate, const Mat33& k ) {
    // Project
    float3 image_coordinate {
        camera_coordinate.x / camera_coordinate.z,
        camera_coordinate.y / camera_coordinate.z,
        1.0f
    };

    // Adjust by cam intrinsics
    int3 pixel_coordinate {
        static_cast<int>(floor( ( k.m11 * image_coordinate.x) + (k.m12 * image_coordinate.y) + k.m13 )),
        static_cast<int>(floor( ( k.m21 * image_coordinate.x) + (k.m22 * image_coordinate.y) + k.m23 )),
        1
    };

    return pixel_coordinate;
}

/**
 * Convert a world coordinate into camera space by
 * by multiplying by pose matrix inverse
 * @param world_coordinate The world coordinate to convert
 * @param inv_pose The 4x4 inverse pose matrix
 * @return a 3D coordinate in camera space
 */
__device__
float3 world_to_camera( const float3& world_coordinate, const Mat44& inv_pose ) {
    float3 cam_coordinate {
        (inv_pose.m11 * world_coordinate.x ) + (inv_pose.m12 * world_coordinate.y) + (inv_pose.m13 * world_coordinate.z) + inv_pose.m14,
        (inv_pose.m21 * world_coordinate.x ) + (inv_pose.m22 * world_coordinate.y) + (inv_pose.m23 * world_coordinate.z) + inv_pose.m24,
        (inv_pose.m31 * world_coordinate.x ) + (inv_pose.m32 * world_coordinate.y) + (inv_pose.m33 * world_coordinate.z) + inv_pose.m34
    };
    float w = (inv_pose.m41 * world_coordinate.x ) + (inv_pose.m42 * world_coordinate.y) + (inv_pose.m43 * world_coordinate.z) + inv_pose.m44;

    cam_coordinate.x /= w;
    cam_coordinate.y /= w;
    cam_coordinate.z /= w;

    return cam_coordinate;
}

/**
 * @param m_voxels The voxel values (in devcie memory) 
 * @param m_weights The weight values (in device memory) 
 * @param m_size The voxel size of the space
 * @param m_physical_size The physical size of the space
 * @param m_offset The offset of the front, bottom, left corner
 * @param m_truncation_distance A distance, greater than the voxel diagonal, at which we truncate distance measures in the TSDF
 * @param inv_pose Inverse of the camera pose matrix (maps world to camera coords) (4x4)
 * @param k The caera's intrinsic parameters (3x3)
 * @param kinv Invers eof k (3x3)
 * @param width Width of the depth image
 * @param height Height of the depth image
 * @param d_depth_map Pointer to array of width*height uint16 types in devcie memory
 */
__global__
void integrate_kernel(  float * m_voxels, float * m_weights,
                        dim3 voxel_grid_size, float3 voxel_space_size, float3 offset, const float trunc_distance,
                        Mat44 inv_pose, Mat33 k, Mat33 kinv,
                        uint32_t width, uint32_t height, const uint16_t * depth_map) {

    // TODO: Move this into parameter bock and pass in from Volume
    const float m_max_weight = 100;

    // Extract the voxel Y and Z coordinates we then iterate over X
    int vy = threadIdx.y + blockIdx.y * blockDim.y;
    int vz = threadIdx.z + blockIdx.z * blockDim.z;

    // If this thread is in range
    if( vy < voxel_grid_size.y && vz < voxel_grid_size.z ) {


        // The next (x_size) elements from here are the x coords
        size_t base_voxel_index =  ((voxel_grid_size.x*voxel_grid_size.y) * vz ) + (voxel_grid_size.x * vy);

        // Compute the voxel size
        float3 voxel_size { voxel_space_size.x / voxel_grid_size.x, 
                            voxel_space_size.y / voxel_grid_size.y, 
                            voxel_space_size.z / voxel_grid_size.z  };

        // We want to iterate over the entire voxel space
        // Each thread should be a Y,Z coordinate with the thread iterating over x
        size_t voxel_index = base_voxel_index;
        for( int vx=0; vx<voxel_grid_size.x; vx++ ) {

            // Work out where in the image, the centre of this voxel projects
            // This gives us a pixel in the depth map

            // Convert voxel to world coords of centre
            float3 centre_of_voxel        = centre_of_voxel_at( vx, vy, vz, voxel_size, offset );

            // Convert world to camera coords
            float3 centre_of_voxel_in_cam = world_to_camera( centre_of_voxel, inv_pose );

            // Project into image plane
            int3   centre_of_voxel_in_pix = camera_to_pixel( centre_of_voxel_in_cam, k );

            // if this point is in the camera view frustum...
            if( ( centre_of_voxel_in_pix.x >=0 && centre_of_voxel_in_pix.x < width ) && ( centre_of_voxel_in_pix.y >= 0 && centre_of_voxel_in_pix.y < height) ) {

                uint16_t voxel_pixel_x = static_cast<uint16_t>( centre_of_voxel_in_pix.x);
                uint16_t voxel_pixel_y = static_cast<uint16_t>( centre_of_voxel_in_pix.y);

                // Extract the depth to the surface at this point
                uint32_t voxel_image_index = voxel_pixel_y * width + voxel_pixel_x;
                uint16_t surface_depth = depth_map[ voxel_image_index ];

                // If the depth is valid
                if( surface_depth > 0 ) {

                    // Project depth entry to a vertex
                    float3 surface_vertex = depth_to_vertex( surface_depth, voxel_pixel_x, voxel_pixel_y, kinv);


                    // First approach ot getting SDF is based on absolute difference in distance
                    // float voxel_distance = sqrt( (centre_of_voxel_in_cam.x * centre_of_voxel_in_cam.x ) + (centre_of_voxel_in_cam.y * centre_of_voxel_in_cam.y ) + (centre_of_voxel_in_cam.z * centre_of_voxel_in_cam.z )  );
                    //float surface_distance = sqrt( surface_vertex.x*surface_vertex.x + surface_vertex.y*surface_vertex.y + surface_vertex.z*surface_vertex.z);
                    // float sdf = surface_distance - voxel_distance;


                    // Alternatve approach is based on difference in Z coords only
                    // Note that for RHS camera, we expect Z coords here to be -ve
                    // So SDF is (-surface.z) - (-voxel.z)
                    // which is voxel.z - surface.z hence inversion
                    // We'd also expect that surface_vertex.z is the same as -ve depth
                    float sdf = centre_of_voxel_in_cam.z - surface_vertex.z;

                    // Truncate
                    float tsdf;
                    if( sdf > 0 ) {
                        tsdf = min( 1.0, sdf / trunc_distance);
                    } else {
                        tsdf = max( -1.0, sdf / trunc_distance);
                    }

                    // Extract prior weight
                    float prior_weight = m_weights[voxel_index];
                    float current_weight = 1.0f;

                    float prior_distance = m_voxels[voxel_index];

                    float new_weight = min( prior_weight + current_weight, m_max_weight );
                    float new_distance = ( (prior_distance * prior_weight) + (tsdf * current_weight) ) / new_weight;

                    m_weights[voxel_index] = new_weight;
                    m_voxels[voxel_index] = new_distance; 
                } // End of point in frustrum
            } // Voxel depth <= 0

            voxel_index++;
        } // For all Xss
    }
}