//
//  TSDFVolume.cpp
//  TSDF
//
//  Created by Dave on 11/03/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//
#include "cu_common.hpp"

#include "GPUTSDFVolume.hpp"
#include "GPURaycaster.hpp"
#include "TSDF_utilities.hpp"
#include "../Utilities/TSDFLoader.hpp"

#include <fstream>
#include <iomanip>
#include <cfloat>
#include <cstdint>

#include "math_constants.h"




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

                    float new_weight = prior_weight + current_weight;
//                    float new_weight = min( prior_weight + current_weight, m_max_weight );
                    float new_distance = ( (prior_distance * prior_weight) + (tsdf * current_weight) ) / new_weight;

                    m_weights[voxel_index] = new_weight;
                    m_voxels[voxel_index] = new_distance; 
                } // End of point in frustrum
            } // Voxel depth <= 0

            voxel_index++;
        } // For all Xss
    }
}



namespace phd {

GPUTSDFVolume::~GPUTSDFVolume() {
            // Remove existing data
        if( m_voxels ) {
            cudaFree( m_voxels ) ;
            m_voxels = 0;
        }
        if( m_weights ) {
            cudaFree( m_weights );
            m_weights = 0;
        }
        if( m_voxel_rotations ) {
            cudaFree( m_voxel_rotations );
            m_voxel_rotations = 0;
        }
        if( m_voxel_translations ) {
            cudaFree( m_voxel_translations );
            m_voxel_translations = 0;
        }


}

/**
 * Constructor with specified number of voxels in each dimension
 * @param size
 * @param physical_size
 */
GPUTSDFVolume::GPUTSDFVolume( const Eigen::Vector3i & size, const Eigen::Vector3f & physical_size ) : m_offset { 0.0, 0.0, 0.0 }, m_voxels {NULL}, m_weights {NULL} {
    std::cout << "TSDFVolume::ctor called." << std::endl;

    if( ( size.x() > 0 ) && ( size.y() > 0 ) && ( size.z() > 0 ) && ( physical_size.x() > 0 ) && ( physical_size.y() > 0 ) && ( physical_size.z() > 0 ) ) {
        set_size( size.x(), size.y(), size.z() , physical_size.x(), physical_size.y(), physical_size.z() );
    } else {
        throw std::invalid_argument( "Attempt to construct CPUTSDFVolume with zero or negative size" );
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
__host__
void GPUTSDFVolume::set_size( uint16_t volume_x, uint16_t volume_y, uint16_t volume_z, float psize_x, float psize_y, float psize_z) {
    using namespace Eigen;

    if( ( volume_x != 0 && volume_y != 0 && volume_z != 0 ) && ( psize_x != 0 && psize_y != 0 && psize_z != 0 ) ) {


        // Remove existing data
        if( m_voxels ) {
            cudaFree( m_voxels ) ;
            m_voxels = 0;
        }
        if( m_weights ) {
            cudaFree( m_weights );
            m_weights = 0;
        }

        m_size = dim3 { volume_x, volume_y, volume_z };
        m_physical_size = float3 { psize_x, psize_y, psize_z };

        // Compute truncation distance - must be at least 2x max voxel size
        float cx = m_physical_size.x / m_size.x;
        float cy = m_physical_size.y / m_size.y;
        float cz = m_physical_size.z / m_size.z;

        m_voxel_size = float3 { cx, cy, cz };

        // Set t > diagonal of voxel
        float vs_norm = sqrt( cx*cx+cy*cy+cz*cz );
        m_truncation_distance = 1.1f * vs_norm;

        // Allocate device storage
        cudaError_t err;
        err = cudaMalloc( &m_voxels, volume_x * volume_y * volume_z * sizeof( float ) );
        if( err != cudaSuccess ) {
            throw std::bad_alloc( );
        }


        err = cudaMalloc( &m_weights, volume_x * volume_y * volume_z * sizeof( float ) );
        if( err != cudaSuccess ) {
            cudaFree( m_voxels );
            throw std::bad_alloc( );
        }

        err = cudaMalloc( &m_voxel_translations, volume_x * volume_y * volume_z * sizeof( float3 ) );
        if( err != cudaSuccess ) {
            cudaFree( m_voxels );
            cudaFree( m_weights );
            throw std::bad_alloc( );
        }

        err = cudaMalloc( &m_voxel_rotations, volume_x * volume_y * volume_z * sizeof( float3 ) );
        if( err != cudaSuccess ) {
            cudaFree( m_voxels );
            cudaFree( m_weights );
            cudaFree( m_voxel_translations );
            throw std::bad_alloc( );
        }

        clear();

        // Max weight for integrating depth images
        m_max_weight = 20.0f;

    } else {
        throw std::invalid_argument( "Attempt to set TSDF size to zero" );
    }
}



/**
 * @return the size of this space.
 */
Eigen::Vector3i GPUTSDFVolume::size( ) const {
    return Eigen::Vector3i { static_cast<int>(m_size.x), static_cast<int>(m_size.y), static_cast<int>(m_size.z)};
}



/**
 * @return the dimensions of each voxel in mm
 */
Eigen::Vector3f GPUTSDFVolume::voxel_size( ) const {
    return Eigen::Vector3f { m_voxel_size.x, m_voxel_size.y, m_voxel_size.z};
}

/**
 * @return the physical size of the volume in world coords (mm)
 */
Eigen::Vector3f GPUTSDFVolume::physical_size( ) const {
    return Eigen::Vector3f {m_physical_size.x, m_physical_size.y, m_physical_size.z};
}

/**
 * @return the truncation distance (mm)
 */
float GPUTSDFVolume::truncation_distance( ) const {
    return m_truncation_distance;
}

/**
 * Offset the TSDF volume in space by the given offset. By default, the bottom, left, front corner of
 * voxel (0,0,0) is at world coordinate (0,0,0). This moves that point to the new world coordinate by a
 * @param ox X offset in mm
 * @param oy Y offset in mm
 * @param oz Z offset in mm
 */
void GPUTSDFVolume::offset( float ox, float oy, float oz ) {
    m_offset = float3 {ox, oy, oz};
}

/**
 * @return the offset f the TSDF volume in space
 */
Eigen::Vector3f GPUTSDFVolume::offset( ) const {
    return Eigen::Vector3f { m_offset.x, m_offset.y, m_offset.z };
}



#pragma mark - Data access
/**
 * @param x The horizontal voxel coord
 * @param y The vertical voxel coord
 * @param z The depth voxel coord
 * @return The distance to the surface at that voxel
 */
float GPUTSDFVolume::distance( int x, int y, int z ) const {
    return m_voxels[ index(x, y, z) ];
}

/**
 * @param x The horizontal voxel coord
 * @param y The vertical voxel coord
 * @param z The depth voxel coord
 * @param distance The distance to set
 * @return The distance to the surface at that voxel
 */
void GPUTSDFVolume::set_distance( int x, int y, int z, float distance ) {
    size_t idx =index( x, y, z );
    m_voxels[ idx ] = distance;
}


/**
 * @param x The horizontal voxel coord
 * @param y The vertical voxel coord
 * @param z The depth voxel coord
 * @return The weight at that voxel
 */
float GPUTSDFVolume::weight( int x, int y, int z ) const {
    return m_weights[ index(x, y, z) ];
}

/**
 * @param x The horizontal voxel coord
 * @param y The vertical voxel coord
 * @param z The depth voxel coord
 * @param weight The weight to set
 * @return The weight at that voxel
 */
void GPUTSDFVolume::set_weight( int x, int y, int z, float weight ) {
    m_weights[ index(x, y, z) ] = weight;
}

void GPUTSDFVolume::set_distance_data( const float * distance_data ) {
        size_t data_size = m_size.x * m_size.y * m_size.z * sizeof( float);
        cudaMemcpy( m_voxels, distance_data, data_size, cudaMemcpyHostToDevice );
}
void GPUTSDFVolume::set_weight_data( const float * weight_data ) {
        size_t data_size = m_size.x * m_size.y * m_size.z * sizeof( float);
        cudaMemcpy( m_weights, weight_data, data_size, cudaMemcpyHostToDevice );
}


/**
 * Reset the 
 * @param translations X x Y x Z array of float3s
 * @param grid_size The size of the voxel grid
 * @param voxel_size The size of an individual voxel
 * @param grid_offset The offset of the grid
 */
__global__
void initialise_translations( float3 * translations, dim3 grid_size, float3 voxel_size, float3 grid_offset ) {

    // Extract the voxel Y and Z coordinates we then iterate over X
    int vy = threadIdx.y + blockIdx.y * blockDim.y;
    int vz = threadIdx.z + blockIdx.z * blockDim.z;

    // If this thread is in range
    if( vy < grid_size.y && vz < grid_size.z ) {


        // The next (x_size) elements from here are the x coords
        size_t base_voxel_index =  ((grid_size.x * grid_size.y) * vz ) + (grid_size.x * vy);

        // We want to iterate over the entire voxel space
        // Each thread should be a Y,Z coordinate with the thread iterating over x
        size_t voxel_index = base_voxel_index;
        for( int vx=0; vx<grid_size.x; vx++ ) {
            translations[voxel_index].x = (( vx + 0.5f ) * voxel_size.x) + grid_offset.x;
            translations[voxel_index].y = (( vy + 0.5f ) * voxel_size.y) + grid_offset.y;
            translations[voxel_index].z = (( vz + 0.5f ) * voxel_size.z) + grid_offset.z;

            voxel_index++;
        }
    }
}

/**
 * Initialise the deformation field with a twist
 * @param translations X x Y x Z array of float3s
 * @param grid_size The size of the voxel grid
 * @param voxel_size The size of an individual voxel
 * @param grid_offset The offset of the grid
 */
__global__
void initialise_translations_with_twist( float3 * translations, dim3 grid_size, float3 voxel_size, float3 grid_offset  ) {

    // Extract the voxel Y and Z coordinates we then iterate over X
    int vy = threadIdx.y + blockIdx.y * blockDim.y;
    int vz = threadIdx.z + blockIdx.z * blockDim.z;

    // If this thread is in range
    if( vy < grid_size.y && vz < grid_size.z ) {

        // The next (x_size) elements from here are the x coords
        size_t base_voxel_index =  ((grid_size.x * grid_size.y) * vz ) + (grid_size.x * vy);

        // Compute centre of space
        float3 centre_of_space {
            grid_offset.x + (0.5f * voxel_size.x * grid_size.x),
            grid_offset.y + (0.5f * voxel_size.y * grid_size.y),
            grid_offset.z + (0.5f * voxel_size.z * grid_size.z)
        };

        // Compute the centre of rotation
        float3 centre_of_rotation {
            grid_offset.x + ( 1.5f * grid_size.x * voxel_size.x),
            grid_offset.y + ( 0.5f * grid_size.y * voxel_size.y),
            grid_offset.z + ( 0.5f * grid_size.z * voxel_size.z),
        };


        // We want to iterate over the entire voxel space
        // Each thread should be a Y,Z coordinate with the thread iterating over x
        size_t voxel_index = base_voxel_index;
        for( int vx=0; vx<grid_size.x; vx++ ) {

            // Starting point
            float3 tran{
                (( vx + 0.5f ) * voxel_size.x) + grid_offset.x,
                (( vy + 0.5f ) * voxel_size.y) + grid_offset.y,
                (( vz + 0.5f ) * voxel_size.z) + grid_offset.z
            };

            // Compute current angle with cor and hor axis
            float dx = tran.x - centre_of_rotation.x;
            float dy = tran.y - centre_of_rotation.y;

            float theta = atan2( dy, dx ) * 2;

            float sin_theta = sin( theta );
            float cos_theta = cos( theta );

            // Compute relative X and Y
            float rel_x = ( tran.x - centre_of_rotation.x );
            float rel_y = ( tran.y - centre_of_rotation.y );

            translations[voxel_index].x = ( ( cos_theta * rel_x ) - ( sin_theta * rel_y ) ) + centre_of_rotation.x;
            translations[voxel_index].y = ( ( sin_theta * rel_x ) + ( cos_theta * rel_y ) ) + centre_of_rotation.y;
            translations[voxel_index].z = tran.z;

            voxel_index++;
        }
    }
}


/**
 * Clear the TSDF memory on the device
 */
__host__
void GPUTSDFVolume::clear( ) {
    size_t data_size = m_size.x * m_size.y * m_size.z * sizeof( float );

    cudaMemset( m_weights, 0, data_size );
    cudaMemset( m_voxels, 0, data_size );
    cudaMemset( m_voxel_rotations, 0, m_size.x * m_size.y * m_size.z * sizeof( float3) );

    // Now initialise the translations
    dim3 block( 1, 32, 32 );
    dim3 grid ( 1, divUp( m_size.y, block.y ), divUp( m_size.z, block.z ) );
//    initialise_translations<<<grid, block>>>( m_voxel_translations, m_size, m_voxel_size, m_offset );
    initialise_translations_with_twist<<<grid, block>>>( m_voxel_translations, m_size, m_voxel_size, m_offset);
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
__host__
void GPUTSDFVolume::integrate( const uint16_t * depth_map, uint32_t width, uint32_t height, const Camera & camera ) {
    using namespace Eigen;

    std::cout << "Integrate" << std::endl;

    // Construct device type parameters for integration
    Mat44 inv_pose;
    memcpy( &inv_pose, camera.inverse_pose().data(), 16 * sizeof( float ) );

    Mat33 k;
    memcpy( &k, camera.k().data(), 9 * sizeof( float ) );

    Mat33 kinv;
    memcpy( &kinv, camera.kinv().data(), 9 * sizeof( float ) );

    // Copy depth map data to device
    uint16_t * d_depth_map;
    size_t data_size = width * height * sizeof( uint16_t);
    cudaError_t err = cudaMalloc( &d_depth_map, data_size );
    if( err == cudaSuccess) {


        err = cudaMemcpy( d_depth_map, depth_map, data_size, cudaMemcpyHostToDevice );
        assert( err == cudaSuccess );

        // Call the kernel
        dim3 block( 1, 32, 32 );
        dim3 grid ( 1, divUp( m_size.y, block.y ), divUp( m_size.z, block.z ) );
        integrate_kernel<<<grid, block>>>( m_voxels, m_weights, m_size, m_physical_size, m_offset, m_truncation_distance, inv_pose, k, kinv, width, height, d_depth_map);
        // Wait for kernel to finish
        cudaDeviceSynchronize( );

        // Now delete data
        cudaFree( d_depth_map );
    } else {
        std::cout << "Couldn't allocate storage for depth map" << std::endl;
    }
}

#pragma mark - Import/Export

//TODO: Put load and save into base class. Block transfer data from memory. Write as binary file at least an option.
/**
 * Save the TSDF to file
 * @param The filename
 * @return true if the file saved OK otherwise false.
 */
bool GPUTSDFVolume::save_to_file( const std::string & file_name) const {
    using namespace std;

    bool success = false;

    // Copy to local memory
    float * host_voxels;
    float * host_weights;

    size_t num_voxels = m_size.x * m_size.y * m_size.z;

    host_voxels = new float[ num_voxels ];
    if ( host_voxels ) {
        host_weights = new float[ num_voxels ];
        if ( host_weights) {

            // Copy data from card into local memeory
            cudaMemcpy( host_voxels, m_voxels, num_voxels * sizeof( float), cudaMemcpyDeviceToHost);
            cudaMemcpy( host_weights, m_weights, num_voxels * sizeof( float), cudaMemcpyDeviceToHost);



            // Open file
            ofstream ofs { file_name };
            ofs << fixed << setprecision(3);

            // Write Dimensions
            ofs << "voxel size = " << m_size.x << " " << m_size.y << " " << m_size.z << std::endl;
            ofs << "space size = " << m_physical_size.x << " " << m_physical_size.y << " " << m_physical_size.z << std::endl;

            // Write data
            for( uint16_t y = 0; y< m_size.y ; y++ ) {
                for( uint16_t x = 0; x< m_size.x ; x++ ) {
                    ofs << std::endl << "# y "<< y << ", x " << x << " tsdf" << std::endl;

                    for( uint16_t z = 0; z< m_size.z ; z++ ) {
                        size_t idx = index( x, y, z ) ;

                        ofs << host_voxels[ idx ] << " ";
                    }

                    ofs << std::endl << "# y "<< y << ", x " << x << " weights" << std::endl;
                    for( uint16_t z = 0; z< m_size.z ; z++ ) {
                        size_t idx = index( x, y, z ) ;
                        ofs  << host_weights[ idx ] << " ";
                    }
                }
            }

            // Close file
            ofs.close();

            delete [] host_voxels;
            delete [] host_weights;
            success = true;
        } else {
            delete [] host_voxels;
            std::cout << "Couldn't allocate host_weights memory to save TSDF" << std::endl;
        }
    } else {
        std::cout << "Couldn't allocate host_voxels memory to save TSDF" << std::endl;
    }


    return success;
}


/**
 * Load the given TSDF file
 * @param The filename
 * @return true if the file saved OK otherwise false.
 */
bool GPUTSDFVolume::load_from_file( const std::string & file_name) {
    std::cout << "INvalid method call: load_from_file" << std::endl;
    return false;
}


#pragma mark - Rendering
void GPUTSDFVolume::raycast( uint16_t width, uint16_t height, const Camera& camera, Eigen::Matrix<float, 3, Eigen::Dynamic>& vertices, Eigen::Matrix<float, 3, Eigen::Dynamic>& normals ) const {
    GPURaycaster raycaster( width, height );

    raycaster.raycast( *this, camera, vertices, normals );
}
}

