//
//  TSDFVolume.cpp
//  TSDF
//
//  Created by Dave on 11/03/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//
#include "../include/cuda_utilities.hpp"
#include "../include/cuda_coordinate_transforms.hpp"

#include "../include/TSDFVolume.hpp"
#include "../include/GPURaycaster.hpp"
#include "../include/TSDF_utilities.hpp"

#include <fstream>
#include <iomanip>
#include <cfloat>
#include <cstdint>

#include <thrust/device_vector.h>

#include "math_constants.h"



/**
 * Compute the index into the voxel space for a given x,y,z coordinate
 * @param size The size (in voxels) of the volume
 * @param x The x coord
 * @param y The y coord
 * @param z The z coord
 * @return The index
 */
__device__ __forceinline__
size_t index( const dim3& size, int x, int y, int z ) {
    return x + (y * size.x) + (z * size.x * size.y);
};

/**
 * @param size The size (in voxels) of the volume
 * @param distance_data The depth data for the volume
 * @param x The horizontal voxel coord
 * @param y The vertical voxel coord
 * @param z The depth voxel coord
 * @return The distance to the surface at that voxel
 */
__device__ __forceinline__
float distance( const dim3& size, float *distance_data, int x, int y, int z ) {
    return distance_data[ index( size, x, y, z) ];
}

/**
 * @param size The size (in voxels) of the volume
 * @param distance_data The depth data for the volume
 * @param x The horizontal voxel coord
 * @param y The vertical voxel coord
 * @param z The depth voxel coord
 * @param distance The distance to set
 * @return The distance to the surface at that voxel
 */
__device__ __forceinline__
void set_distance(const  dim3& size, float * distance_data, int x, int y, int z, float distance ) {
    size_t idx = index( size, x, y, z );
    distance_data[ idx ] = distance;
}

/**
 * @param size The size (in voxels) of the volume
 * @param weights The weight data for the volume
 * @param x The horizontal voxel coord
 * @param y The vertical voxel coord
 * @param z The depth voxel coord
 * @return The weight at that voxel
 */
__device__ __forceinline__
float weight( const dim3& size, float * weights, int x, int y, int z ) {
    return weights[ index(size, x, y, z) ];
}

/**
 * @param size The size (in voxels) of the volume
 * @param weights The weight data for the volume
 * @param x The horizontal voxel coord
 * @param y The vertical voxel coord
 * @param z The depth voxel coord
 * @param weight The weight to set
 * @return The weight at that voxel
 */
__device__ __forceinline__
void set_weight( const dim3& size, float * weights, int x, int y, int z, float weight ) {
    weights[ index(size, x, y, z) ] = weight;
}


/* **************************************************************************************************************
 * *
 * *
 * *    Integrate
 * *
 * *
 * **************************************************************************************************************/
__global__
/**
 * Predicate:
 * Tag indices of voxels that project into frustum with true
 */
void voxel_in_frustrum( const int               num_voxels,
                        const float3 * const    voxel_centres,
                        const uint32_t          width,
                        const uint32_t          height,
                        const Mat33             k,
                        const Mat44             inv_pose,
                        bool                    * in_frustum ) {

    int voxel_index = blockDim.x * blockIdx.x + threadIdx.x;

    if ( voxel_index < num_voxels ) {
        float3 voxel_centre = voxel_centres[ voxel_index ];

        int3   centre_of_voxel_in_pix = world_to_pixel( voxel_centre, inv_pose, k );

        if ( centre_of_voxel_in_pix.x >= 0 && centre_of_voxel_in_pix.x <  width && centre_of_voxel_in_pix.y >= 0 && centre_of_voxel_in_pix.y < height ) {
            in_frustum[voxel_index] = true;
        } else {
            in_frustum[voxel_index] = false;
        }
    }
}

/**
 * Exlusive sum scan on in_frustum
 * Input the list of predicate outputs
 * Output an array of offsets into a storage array for affected indices
 *
 */
__host__
void scan_storage_for_voxel_indices( const bool * in_frustum,
                                     const int  num_voxels,
                                     int        * scatter_address,
                                     int&       num_included ) {
    num_included = 0;

    for ( int i = 0; i < num_voxels; i++ ) {
        scatter_address[i] = num_included;

        if ( in_frustum[i]) {
            num_included++;
        }
    }
}

/**
 * Scatter
 * Scatter the voxel index array
 * For every 'in frustum' index, write the index to 'scatter_address' in outputs
 * Compact index list is preallocated host memory
 */
__host__
void scatter_voxel_indices( const bool      * in_frustum,
                            const int       num_voxels,
                            const int       * scatter_address,
                            int             * compact_index_list ) {
    for ( int i = 0; i < num_voxels; i++ ) {
        if ( in_frustum[ i ] ) {
            compact_index_list[ scatter_address[ i ] ] = i;
        }
    }
}

__host__
void get_voxels_projecting_to_frustum(  const int               num_voxels,
                                        const float3 * const    voxel_centres,
                                        const uint32_t          width,
                                        const uint32_t          height,
                                        const Mat33             k,
                                        const Mat44             inv_pose,
                                        int&                    num_included,
                                        int*&                   included_indices ) {
    bool * d_in_frustum;

    cudaError_t err = cudaMalloc( &d_in_frustum, sizeof( bool ) * num_voxels );
    check_cuda_error( "Failed to allocate storage for voxel indices", err );
    bool * h_in_frustum = new bool[ num_voxels];
    if ( !h_in_frustum) exit( -1 );
	dim3 block( 1024 );
	dim3 grid( num_voxels/block.x );
    voxel_in_frustrum <<< grid, block >>> ( num_voxels, voxel_centres, width, height, k, inv_pose, d_in_frustum );
	cudaDeviceSynchronize();
	err= cudaGetLastError( );
	check_cuda_error("voxel in frustum kernel failed" , err );
 
    err = cudaMemcpy( h_in_frustum, d_in_frustum, num_voxels * sizeof( bool ), cudaMemcpyDeviceToHost);
    check_cuda_error ( "Failed to copy results back from host" , err );

    err = cudaFree( d_in_frustum );
	check_cuda_error( "Failed to free voxel in frustum device data", err );

    int * scatter_address = new int[num_voxels];
    scan_storage_for_voxel_indices( h_in_frustum, num_voxels, scatter_address, num_included );

    included_indices = new int[ num_included ];
    scatter_voxel_indices( h_in_frustum, num_voxels, scatter_address, included_indices );

    delete [] h_in_frustum;
    delete [] scatter_address;
}


/* **************************************************************************************************************
 * *
 * *
 * *    Old Integrate
 * *
 * *
 * **************************************************************************************************************/

/**
 * @param distance_data The voxel values (in devcie memory)
 * @param weight_data The weight values (in device memory)
 * @param trunc_distance A distance, greater than the voxel diagonal, at which we truncate distance measures in the TSDF
 * @param pose The camera pose matrix (maps cam to world, 4x4 )
 * @param inv_pose Inverse of the camera pose matrix (maps world to camera coords) (4x4)
 * @param k The camera's intrinsic parameters (3x3)
 * @param kinv Invers eof k (3x3)
 * @param width Width of the depth image
 * @param height Height of the depth image
 * @param depth_map Pointer to array of width*height uint16 types in devcie memory
 */
__global__
void integrate_kernel(  const int 				num_data_items, 
						const int * const		data_items,
						float      		 		* distance_data,
                        float         			* weight_data,
                        float3      			* voxel_centres,
                        const float   			trunc_distance,
                        const float     		max_weight,
                        Mat44         	 		pose,
                        Mat44           		inv_pose,
                        Mat33           		k,
                        Mat33           		kinv,
                        uint32_t       		 	width,
                        uint32_t        		height,
                        const uint16_t * const 	depth_map) {


    int data_index = blockDim.x * blockIdx.x + threadIdx.x;

    if ( data_index < num_data_items ) {
        int voxel_index = data_items[ data_index ];

        // Work out where in the image, the centre of this voxel projects
        // This gives us a pixel in the depth map

        // Convert voxel to world coords of centre
        float3 centre_of_voxel        = voxel_centres[ voxel_index ];

        // Convert world to pixel coords
        int3   centre_of_voxel_in_pix = world_to_pixel( centre_of_voxel, inv_pose, k );

        // Extract the depth to the surface at this point
        uint32_t voxel_pixel_index = centre_of_voxel_in_pix.y * width + centre_of_voxel_in_pix.x;
        uint16_t surface_depth = depth_map[ voxel_pixel_index ];

        // If the depth is valid
        if ( surface_depth > 0 ) {

            // Project depth entry to a vertex ( in camera space)
            float3 surface_vertex = pixel_to_camera( centre_of_voxel_in_pix, kinv, surface_depth );

            // Compute the SDF is the distance between the camera origin and surface_vertex in world coordinates
            float3 voxel_cam = world_to_camera( centre_of_voxel, inv_pose );
            float sdf = surface_vertex.z - voxel_cam.z;

            if ( sdf >= -trunc_distance ) {
                // Truncate the sdf to the range -trunc_distance -> trunc_distance
                float tsdf;
                if ( sdf > 0 ) {
                    tsdf = min( sdf, trunc_distance);
                } else {
                    tsdf = sdf;
                }

                // Extract prior weight
                float prior_weight = weight_data[voxel_index];
                float current_weight = 1.0f;
                float new_weight = prior_weight + current_weight;
                new_weight = min(new_weight, max_weight );

                float prior_distance = distance_data[voxel_index];
                float new_distance = ( (prior_distance * prior_weight) + (tsdf * current_weight) ) / new_weight;

                weight_data[voxel_index] = new_weight;
                distance_data[voxel_index] = new_distance;
            } // End of sdf > -trunc
        } // End of depth > 0

    }
}



TSDFVolume::~TSDFVolume() {
    std::cout << "Destroying TSDFVolume" << std::endl;

    // Remove existing data
    if ( m_voxels ) {
        cudaFree( m_voxels ) ;
        m_voxels = 0;
    }
    if ( m_weights ) {
        cudaFree( m_weights );
        m_weights = 0;
    }
    if ( m_voxel_translations ) {
        cudaFree( m_voxel_translations );
        m_voxel_translations = 0;
    }
}

/**
 * Constructor with specified number of voxels in each dimension
 * @param size
 * @param physical_size
 */
TSDFVolume::TSDFVolume( const UInt3& size, const UInt3& physical_size ) : m_offset { 0.0, 0.0, 0.0 }, m_voxels {NULL}, m_weights {NULL}, m_voxel_translations{NULL} {
    if ( ( size.x > 0 ) && ( size.y > 0 ) && ( size.z > 0 ) &&
            ( physical_size.x > 0 ) && ( physical_size.y > 0 ) && ( physical_size.z > 0 ) ) {
        set_size( size.x, size.y, size.z , physical_size.x, physical_size.y, physical_size.z );
    } else {
        throw std::invalid_argument( "Attempt to construct TSDFVolume with zero or negative size" );
    }
}


/**
 * Make a TSDFVolume with the given dimensins and physical dimensions
 * @param volume_x X dimension in voxels
 * @param volume_y Y dimension in voxels
 * @param volume_z Z dimension in voxels
 * @param psize_x Physical size in X dimension in mm
 * @param psize_y Physical size in Y dimension in mm
 * @param psize_z Physical size in Z dimension in mm
 */
TSDFVolume::TSDFVolume( uint16_t volume_x, uint16_t volume_y, uint16_t volume_z, float psize_x, float psize_y, float psize_z )  : m_offset { 0.0, 0.0, 0.0 }, m_voxels {NULL}, m_weights {NULL}, m_voxel_translations{NULL} {
    if ( ( volume_x > 0 ) && ( volume_y > 0 ) && ( volume_z > 0 ) &&
            ( psize_x > 0 ) && ( psize_y > 0 ) && ( psize_z > 0 ) ) {

        set_size( volume_x, volume_y, volume_z , psize_x, psize_y, psize_z );
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
void TSDFVolume::set_size( uint16_t volume_x, uint16_t volume_y, uint16_t volume_z, float psize_x, float psize_y, float psize_z) {
    using namespace Eigen;

    if ( ( volume_x != 0 && volume_y != 0 && volume_z != 0 ) && ( psize_x != 0 && psize_y != 0 && psize_z != 0 ) ) {


        // Remove existing data
        if ( m_voxels ) {
            cudaFree( m_voxels ) ;
            m_voxels = 0;
        }
        if ( m_weights ) {
            cudaFree( m_weights );
            m_weights = 0;
        }
        if ( m_voxel_translations ) {
            cudaFree( m_voxel_translations );
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
        float vs_norm = sqrt( cx * cx + cy * cy + cz * cz );
        m_truncation_distance = 1.1f * vs_norm;

        // Allocate device storage
        cudaError_t err;
        err = cudaMalloc( &m_voxels, volume_x * volume_y * volume_z * sizeof( float ) );
        if ( err != cudaSuccess ) {
            throw std::bad_alloc( );
        }


        err = cudaMalloc( &m_weights, volume_x * volume_y * volume_z * sizeof( float ) );
        if ( err != cudaSuccess ) {
            cudaFree( m_voxels );
            throw std::bad_alloc( );
        }

        err = cudaMalloc( &m_voxel_translations, volume_x * volume_y * volume_z * sizeof( float3 ) );
        if ( err != cudaSuccess ) {
            cudaFree( m_voxels );
            cudaFree( m_weights );
            throw std::bad_alloc( );
        }

        clear();

        // Max weight for integrating depth images
        m_max_weight = 60.0f;

    } else {
        throw std::invalid_argument( "Attempt to set TSDF size to zero" );
    }
}


#pragma mark - Data access

/**
 * Set the distance data for the TSDF in one call
 * @param distance_data Pointer to enough floats to populate the TSFD
 */
void TSDFVolume::set_distance_data( const float * distance_data ) {
    size_t data_size = m_size.x * m_size.y * m_size.z * sizeof( float);
    cudaError_t err = cudaMemcpy( m_voxels, distance_data, data_size, cudaMemcpyHostToDevice );
    check_cuda_error( "Couldn't set distance data", err );
}


/**
 * Set the weight data for the TSDF in one call
 * @param weight_data Pointer to enough floats to populate the TSFD
 */
void TSDFVolume::set_weight_data( const float * weight_data ) {
    size_t data_size = m_size.x * m_size.y * m_size.z * sizeof( float);
    cudaError_t err = cudaMemcpy( m_weights, weight_data, data_size, cudaMemcpyHostToDevice );
    check_cuda_error( "Couldn't set weight data", err );
}


/**
 * Set the translation dat for this space
 * @param data Data on host memory space; Assumed to be vx*vy*vz float3
 */
void TSDFVolume::set_translation_data( Float3 *data) {
    size_t data_size = m_size.x * m_size.y * m_size.z * sizeof( Float3 );
    cudaError_t err = cudaMemcpy( m_voxel_translations, data, data_size, cudaMemcpyHostToDevice );
    check_cuda_error( "Couldn't set voxel translations", err );
}


/**
 * Reset the defomation grid by setting each transaltion point to the effectve, reglar position
 * in space of that voxel centre.
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
    if ( vy < grid_size.y && vz < grid_size.z ) {


        // The next (x_size) elements from here are the x coords
        size_t base_voxel_index =  ((grid_size.x * grid_size.y) * vz ) + (grid_size.x * vy);

        size_t voxel_index = base_voxel_index;
        for ( int vx = 0; vx < grid_size.x; vx++ ) {
            translations[voxel_index].x = (( vx + 0.5f ) * voxel_size.x) + grid_offset.x;
            translations[voxel_index].y = (( vy + 0.5f ) * voxel_size.y) + grid_offset.y;
            translations[voxel_index].z = (( vz + 0.5f ) * voxel_size.z) + grid_offset.z;

            voxel_index++;
        }
    }
}


__global__
void set_memory_to_value( float * pointer, int size, float value ) {
    int idx = threadIdx.x + (blockIdx.x * blockDim.x );
    if ( idx < size ) {
        pointer[idx] = value;
    }
}



/**
 * Clear the TSDF memory on the device
 */
__host__
void TSDFVolume::clear( ) {
    int data_size = m_size.x * m_size.y * m_size.z;


    dim3 block( 1024, 1, 1 );
    dim3 grid ( divUp( data_size, block.x ), 1, 1 );

    cudaError_t err;

    set_memory_to_value <<< grid, block >>>( m_weights, data_size, 0.0f );
    cudaDeviceSynchronize( );
    err = cudaGetLastError();
    check_cuda_error( "couldn't clear weights memory", err );


    set_memory_to_value <<< grid, block >>>( m_voxels, data_size, m_truncation_distance );
    cudaDeviceSynchronize( );
    err = cudaGetLastError();
    check_cuda_error( "couldn't clear depth memory", err );

    // Now initialise the translations
    dim3 block2( 1, 32, 32 );
    dim3 grid2 ( 1, divUp( m_size.y, block2.y ), divUp( m_size.z, block2.z ) );
    initialise_translations <<< grid2, block2>>>( m_voxel_translations, m_size, m_voxel_size, m_offset );
    cudaDeviceSynchronize( );
    err = cudaGetLastError();
    check_cuda_error( "couldn't initialise deformation memory", err );
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
void TSDFVolume::integrate( const uint16_t * depth_map, uint32_t width, uint32_t height, const Camera & camera ) {
    assert( depth_map );

    std::cout << "Integrating depth map size " << width << "x" << height << std::endl;

    // Get the list of voxels which fall in the frustum
    std::cout << " -- Get affected voxels" << std::endl;
    Mat44 inv_pose;
    memcpy( &inv_pose, camera.inverse_pose().data(), 16 * sizeof( float ) );

    Mat33 k;
    memcpy( &k, camera.k().data(), 9 * sizeof( float ) );

    int num_voxels = m_size.x * m_size.y * m_size.z;

    int num_included = 0;
    int * included_indices = nullptr;
    get_voxels_projecting_to_frustum( num_voxels, m_voxel_translations,
                                      width, height,
                                      k, inv_pose,
                                      num_included, included_indices );

    // For each of these, update the TSDF
    Mat44 pose;
    memcpy( &pose, camera.pose().data(), 16 * sizeof( float ) );

    Mat33 kinv;
    memcpy( &kinv, camera.kinv().data(), 9 * sizeof( float ) );

    // Copy depth map data to device
    uint16_t * d_depth_map;
    size_t data_size = width * height * sizeof( uint16_t);
    cudaError_t err = cudaMalloc( &d_depth_map, data_size );
    check_cuda_error( "Couldn't allocate storage for depth map", err);

    err = cudaMemcpy( d_depth_map, depth_map, data_size, cudaMemcpyHostToDevice );
    check_cuda_error( "Failed to copy depth map to GPU", err);


	// Copy the data items back to device
	int * d_included_indices;
	err = cudaMalloc( &d_included_indices, num_included * sizeof( int ) );
    check_cuda_error( "Couldn't allocate storage for included indices on device", err);
	err = cudaMemcpy( d_included_indices, included_indices, num_included*sizeof(int), cudaMemcpyHostToDevice );
    check_cuda_error( "Failed to copy included indices to GPU", err);



    // Call the kernel
    dim3 block( 512  );
    dim3 grid ( divUp( num_included, block.x ) );

    integrate_kernel <<< grid, block>>>( num_included, d_included_indices, m_voxels, m_weights,  m_voxel_translations, m_truncation_distance, m_max_weight, pose, inv_pose, k, kinv, width, height, d_depth_map);
    cudaDeviceSynchronize( );
    err = cudaGetLastError();
    check_cuda_error( "Integrate kernel failed", err);

    // Now delete depth map data from device
    err = cudaFree( d_depth_map );
    check_cuda_error( "Failed to deallocate cuda depth map", err);

    err = cudaFree( d_included_indices );
    check_cuda_error( "Failed to deallocate included indices map", err);

     std::cout << "Integration finished" << std::endl;
}

#pragma mark - Import/Export

/**
 * Save the TSDF to a binary file
 * @param The filename
 * @return true if the file saved OK otherwise false.
 */
bool TSDFVolume::save_to_file( const std::string & file_name) const {
    using namespace std;

    bool success = true;

    // Copy to local memory
    float * host_voxels = nullptr;
    float * host_weights = nullptr;
    float3 * host_deformation = nullptr;

    size_t num_voxels = m_size.x * m_size.y * m_size.z;
    cudaError_t err;

    // Copy distance data from device to host
    host_voxels = new float[ num_voxels ];
    size_t depth_data_size = num_voxels * sizeof( float);
    if ( host_voxels ) {
        err = cudaMemcpy( host_voxels, m_voxels, depth_data_size, cudaMemcpyDeviceToHost);
        if ( err != cudaSuccess ) {
            success = false;
            std::cout << "Failed to copy voxel data from device memory [" << err << "] " << std::endl;
        }
    } else {
        std::cout << "Couldn't allocate host_voxels memory to save TSDF" << std::endl;
        success = false;
    }


    // Copy distance data from device to host
    size_t weight_data_size = num_voxels * sizeof( float);
    if ( success ) {
        host_weights = new float[ num_voxels ];
        if ( host_weights) {
            err = cudaMemcpy( host_weights, m_weights, weight_data_size, cudaMemcpyDeviceToHost);
            if ( err != cudaSuccess ) {
                success = false;
                std::cout << "Failed to copy weight data from device memory [" << err << "] " << std::endl;
            }
        } else {
            success = false;
            std::cout << "Couldn't allocate host_weights memory to save TSDF" << std::endl;
        }
    }


    // Copy distance data from device to host
    size_t deformation_data_size = num_voxels * sizeof( float3);
    if ( success ) {
        host_deformation = new float3[ num_voxels ];
        if ( host_deformation ) {
            err = cudaMemcpy( host_deformation, m_voxel_translations, deformation_data_size, cudaMemcpyDeviceToHost);
            if ( err != cudaSuccess ) {
                success = false;
                std::cout << "Failed to copy deformation data from device memory [" << err << "] " << std::endl;
            }
        } else {
            success = false;
            std::cout << "Couldn't allocate host_weights memory to save TSDF" << std::endl;
        }
    }

    if ( success ) {
        ofstream ofs { file_name, ios::out | ios::binary };

        // Write dimesnions
        std::cout << "  writing " << sizeof( m_size ) + sizeof( m_physical_size ) << " bytes of header data" << std::endl;
        ofs.write( (char *) &m_size, sizeof( m_size ) );
        ofs.write( (char *)&m_physical_size, sizeof( m_physical_size));

        std::cout << "  writing " << depth_data_size << " bytes of depth data" << std::endl;
        ofs.write( (char *)host_voxels, depth_data_size );

        std::cout << "  writing " << weight_data_size << " bytes of weight data" << std::endl;
        ofs.write( (char *)host_weights, weight_data_size );

        std::cout << "  writing " << deformation_data_size << " bytes of deformation data" << std::endl;
        ofs.write( (char *)host_deformation, deformation_data_size );

        ofs.close();
    } else {
        std::cout << "Not saving file due to previous errors" << std::endl;
    }

    // Free up memory
    if ( host_voxels != nullptr ) { delete[] host_voxels; }
    if ( host_weights != nullptr ) { delete[] host_weights; }
    if ( host_deformation != nullptr ) { delete[] host_deformation; }

    return success;
}


/**
 * Load the given TSDF file
 * @param The filename
 * @return true if the file saved OK otherwise false.
 */
bool TSDFVolume::load_from_file( const std::string & file_name) {
    using namespace std;

    ifstream ifs{ file_name, ios::in | ios::binary };

    // Load dimensions
    // Load data
    // Move to device


    std::cout << "Invalid method call: load_from_file" << std::endl;
    return false;
}


#pragma mark - Rendering
void TSDFVolume::raycast( uint16_t width, uint16_t height, const Camera& camera, Eigen::Matrix<float, 3, Eigen::Dynamic>& vertices, Eigen::Matrix<float, 3, Eigen::Dynamic>& normals ) const {
    GPURaycaster raycaster( width, height );

    raycaster.raycast( *this, camera, vertices, normals );
}
