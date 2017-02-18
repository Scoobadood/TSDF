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

#include "math_constants.h"

const float POINT_EPSILON=0.001f;

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
 * Set the distance to the surface at a grid point
 * @param size The size (in voxels) of the volume
 * @param distance_data The depth data for the volume
 * @param x The horizontal voxel coord
 * @param y The vertical voxel coord
 * @param z The depth voxel coord
 * @param distance The distance to set
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

/**
 * Obtain indices and trilinear coefficients for for the gridpoints which surround the given point in space
 * @param point The point in TSDF coordinate space
 * @param voxel_grid_size Dimensions of the TSDF
 * @param voxel_size The physical size of a single voxel
 * @param indices An array of indices of the voxels surrounding the given point
 * ordered as (minx, miny, minz), (maxx, miny, minz), (maxx, miny, maxz), (maxx, miny, minz) and then the maxz values
 * @return true If the values in indices are valid (ie point is in TSDF space)
 */
__device__ 
bool get_trilinear_elements( const float3   point, 
                             const dim3     voxel_grid_size,
                             const float3   voxel_size,
                             int * const    indices,
                             float * const  coefficients ) {
    bool is_valid = false;

    // Manage boundary points
    float3 max_values {
        voxel_grid_size.x * voxel_size.x,
        voxel_grid_size.y * voxel_size.y,
        voxel_grid_size.z * voxel_size.z
    };

    float3 adjusted_point = point;
    if( (point.x > max_values.x) && ( point.x - max_values.x < POINT_EPSILON ) ) adjusted_point.x = max_values.x - POINT_EPSILON;
    if( (point.y > max_values.y) && ( point.y - max_values.y < POINT_EPSILON ) ) adjusted_point.y = max_values.y - POINT_EPSILON;
    if( (point.z > max_values.z) && ( point.z - max_values.z < POINT_EPSILON ) ) adjusted_point.z = max_values.z - POINT_EPSILON;
    if( point.x < -POINT_EPSILON ) adjusted_point.x = 0.0f;
    if( point.y < -POINT_EPSILON ) adjusted_point.y = 0.0f;
    if( point.z < -POINT_EPSILON ) adjusted_point.z = 0.0f;

    // Get the voxel containing this point
    int3 voxel = voxel_for_point( adjusted_point, voxel_size );

    // Handle voxel out of bounds
    if ( voxel.x >= 0 && voxel.y >= 0 && voxel.z >= 0  && voxel.x < voxel_grid_size.x && voxel.y < voxel_grid_size.y && voxel.z < voxel_grid_size.z) {

        // Get the centre of the voxel
        float3 v_centre = centre_of_voxel_at( voxel.x, voxel.y, voxel.z, voxel_size );

        // Set up the lower bound for trilinear interpolation
        int3 lower;
        lower.x = (adjusted_point.x < v_centre.x) ? voxel.x - 1 : voxel.x;
        lower.y = (adjusted_point.y < v_centre.y) ? voxel.y - 1 : voxel.y;
        lower.z = (adjusted_point.z < v_centre.z) ? voxel.z - 1 : voxel.z;

        // Handle lower out of bounds
        lower.x = max( lower.x, 0 );
        lower.y = max( lower.y, 0 );
        lower.z = max( lower.z, 0 );

        // Compute u,v,w
        float3 lower_centre = centre_of_voxel_at( lower.x, lower.y, lower.z, voxel_size );
        float3 uvw = f3_sub( adjusted_point, lower_centre );
        uvw = f3_div_elem( uvw, voxel_size );
        float u = uvw.x;
        float v = uvw.y;
        float w = uvw.z;

        // Populate indices
        int delta_x = 1;
        int delta_y = voxel_grid_size.x;
        int delta_z = voxel_grid_size.x * voxel_grid_size.y;
        indices[0] = lower.x + ( lower.y * voxel_grid_size.x ) + ( lower.z * voxel_grid_size.x * voxel_grid_size.y );
        indices[1] = indices[0] + delta_x;
        indices[2] = indices[1] + delta_z;
        indices[3] = indices[0] + delta_z;
        indices[4] = indices[0] + delta_y;
        indices[5] = indices[1] + delta_y;
        indices[6] = indices[2] + delta_y;
        indices[7] = indices[3] + delta_y;

        // And coefficients
        coefficients[0] = (1 - u) * (1 - v) * (1 - w);
        coefficients[1] =    u  * (1 - v) * (1 - w);
        coefficients[2] =    u  * (1 - v) *    w ;
        coefficients[3] = (1 - u) * (1 - v) *    w;
        coefficients[4] = (1 - u) *    v  * (1 - w);
        coefficients[5] =    u  *    v  * (1 - w);
        coefficients[6] = (1 - u) *    v  *    w;
        coefficients[7] =    u  *    v  *    w;
    }

    // Voxel is out of bounds and so can't be used.
    else {
        printf( "Point outside of voxel space %f, %f, %f\n", point.x, point.y, point.z );
    }
    return is_valid;
}

/**
 * Apply rotation  to point
 * @param rotation The rotation expressed as 3 Euler angles
 * @param point The point to rotate
 * @return The rotated point
 */
__device__
float3 rotate( const float3 point, const float3 rotation ) {
    float c1 = cos( rotation.x );
    float c2 = cos( rotation.y );
    float c3 = cos( rotation.z );
    float s1 = sin( rotation.x );
    float s2 = sin( rotation.y );
    float s3 = sin( rotation.z );

    float rx = (c2 * c3)          * point.x - (c2 * s3 )        * point.y + s2          * point.z;
    float ry = (c1*s3 + s1*s2*c3) * point.x + (c1*c3-s1*s2*s3)  * point.y - (s1 * c2 )  * point.z;
    float rz = (s1*s3 - c1*s2*c3) * point.x + (s1*c3 + c1*s2*s3)* point.y + (c1 * c2 )  * point.z;

    return make_float3( rx, ry, rz );
}

/**
 * Apply the TSDF deformation field to a collection of points, deforming them in place
 * @param global_rotation The global rotation of the space
 * @param global_translation The global translation of the space
 * @param deformation_nodes An array of DeformationNodes
 * @param voxel_grid_size The voxel size of the space
 * @param voxel_space_size The physical size of the space
 * @param points The points to be transformed in world coordinates
 * @param num_points The number of points to be transformed
 */
__global__
void deformation_kernel( const float3                               global_rotation,
                         const float3                               global_translation,
                         const TSDFVolume::DeformationNode * const  deformation_nodes,
                         const dim3                                 voxel_grid_size,
                         const float3                               voxel_size,
                         const float3                               voxel_space_size,
                         const float3                               offset,
                         const int                                  num_points,
                         float3 *                                   points) {

    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    if( idx < num_points ) {
        float3 point = points[idx];

        // Get indices of neighbours
        point = f3_sub( point, offset );

        int neighbours[8];
        float coefficients[8];
        get_trilinear_elements( point, voxel_grid_size, voxel_size, neighbours, coefficients );

        // Compute the deformation at this point
        float3 deformed_point{ 0.0f, 0.0f, 0.0f };
        deformed_point = f3_add( deformed_point, f3_mul_scalar( coefficients[0], deformation_nodes[ neighbours[0] ].translation ) );
        deformed_point = f3_add( deformed_point, f3_mul_scalar( coefficients[1], deformation_nodes[ neighbours[1] ].translation ) );
        deformed_point = f3_add( deformed_point, f3_mul_scalar( coefficients[2], deformation_nodes[ neighbours[2] ].translation ) );
        deformed_point = f3_add( deformed_point, f3_mul_scalar( coefficients[3], deformation_nodes[ neighbours[3] ].translation ) );
        deformed_point = f3_add( deformed_point, f3_mul_scalar( coefficients[4], deformation_nodes[ neighbours[4] ].translation ) );
        deformed_point = f3_add( deformed_point, f3_mul_scalar( coefficients[5], deformation_nodes[ neighbours[5] ].translation ) );
        deformed_point = f3_add( deformed_point, f3_mul_scalar( coefficients[6], deformation_nodes[ neighbours[6] ].translation ) );
        deformed_point = f3_add( deformed_point, f3_mul_scalar( coefficients[7], deformation_nodes[ neighbours[7] ].translation ) );

        // Apply global rotation
        deformed_point = rotate( deformed_point, global_rotation );

        // Apply global translation
        deformed_point = f3_add( deformed_point, global_translation );

        // Set this to output point
        points[idx] = deformed_point;
    }
}

/**
 *
 */
void TSDFVolume::deform_mesh( const int num_points, float3 * points ) const {

    // Copy the point array to the device
	float3 * d_points;
	cudaSafeAlloc( (void **) &d_points, num_points * sizeof( float3 ), "d_points" );
	cudaError_t err = cudaMemcpy( d_points, points, num_points * sizeof( float3 ), cudaMemcpyHostToDevice );
    check_cuda_error( "Failed to copy points to device for deformation", err);


    dim3 block( 512, 1, 1 );
    dim3 grid ( divUp( num_points, block.x ), 1, 1 );

    deformation_kernel<<<grid, block >>>(m_global_rotation, 
                                         m_global_translation,
                                         m_deformation_nodes,
                                         m_size,
                                         m_voxel_size,
                                         m_physical_size,
                                         m_offset,
                                         num_points,
                                         d_points );
    cudaDeviceSynchronize( );
    err = cudaGetLastError();
    check_cuda_error( "Deformation kernel failed", err);

	err = cudaMemcpy( points, d_points, num_points * sizeof( float3 ), cudaMemcpyDeviceToHost );
    check_cuda_error( "Failed to copy points from device after deformation", err);
	cudaSafeFree( d_points, "d_points");
}

/**
 * @param distance_data The voxel values (in device memory)
 * @param weight_data The weight values (in device memory)
 * @param voxel_grid_size The voxel size of the space
 * @param voxel_space_size The physical size of the space
 * @param offset The offset of the front, bottom, left corner
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
void integrate_kernel(  float         * distance_data, 
                        float         * weight_data,
                        dim3            voxel_grid_size, 
                        float3          voxel_space_size,
                        TSDFVolume::DeformationNode * deformation_nodes,
                        float3          offset, 
                        const float     trunc_distance,
                        const float     max_weight,
                        Mat44           pose, 
                        Mat44           inv_pose,
                        Mat33           k, 
                        Mat33           kinv,
                        uint32_t        width, 
                        uint32_t        height, 
                        const uint16_t  * depth_map) {

    // Extract the voxel Y and Z coordinates we then iterate over X
    int vy = threadIdx.y + blockIdx.y * blockDim.y;
    int vz = threadIdx.z + blockIdx.z * blockDim.z;

    // If this thread is in range
    if ( vy < voxel_grid_size.y && vz < voxel_grid_size.z ) {


        // The next (x_size) elements from here are the x coords
        int voxel_index =  ((voxel_grid_size.x * voxel_grid_size.y) * vz ) + (voxel_grid_size.x * vy);

        // For each voxel in this column
        for ( int vx = 0; vx < voxel_grid_size.x; vx++ ) {

            // Work out where in the image, the centre of this voxel projects
            // This gives us a pixel in the depth map

            // Convert voxel to world coords of deformed centre
            float3 centre_of_voxel        = deformation_nodes[ voxel_index ].translation;

            // Convert world to pixel coords
            int3   centre_of_voxel_in_pix = world_to_pixel( centre_of_voxel, inv_pose, k );

            // if this point is in the camera view frustum...
            if ( ( centre_of_voxel_in_pix.x >= 0 ) && ( centre_of_voxel_in_pix.x < width ) && ( centre_of_voxel_in_pix.y >= 0 ) && ( centre_of_voxel_in_pix.y < height) ) {

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

					if( sdf >= -trunc_distance ) {
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
                	 //   new_weight = min(new_weight, max_weight );

            	        float prior_distance = distance_data[voxel_index];
        	            float new_distance = ( (prior_distance * prior_weight) + (tsdf * current_weight) ) / new_weight;

    	                weight_data[voxel_index] = new_weight;
	                    distance_data[voxel_index] = new_distance;
					} // End of sdf > -trunc
                } // End of depth > 0
            } // End of point in frustrum

            voxel_index++;
        } // End each voxel in this column
    }
}



TSDFVolume::~TSDFVolume() {
    std::cout << "Destroying TSDFVolume" << std::endl;
    deallocate( );
}


/**
 * Deallocate storage for this TSDF
 */
void TSDFVolume::deallocate( ) {
    // Remove existing data
    if ( m_distances ) {
        cudaFree( m_distances ) ;
        m_distances = 0;
    }
    if ( m_weights ) {
        cudaFree( m_weights );
        m_weights = 0;
    }
    if ( m_colours ) {
        cudaFree( m_colours );
        m_colours = 0;
    }
    if ( m_deformation_nodes ) {
        cudaFree( m_deformation_nodes );
        m_deformation_nodes = 0;
    }
}

/**
 * Constructor with specified number of voxels in each dimension
 * @param size
 * @param physical_size
 */
TSDFVolume::TSDFVolume( const UInt3& size, const Float3& physical_size ) : m_offset { 0.0, 0.0, 0.0 }, m_distances {NULL}, m_weights {NULL}, m_deformation_nodes{NULL}, m_colours{NULL} {
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
TSDFVolume::TSDFVolume( uint16_t volume_x, uint16_t volume_y, uint16_t volume_z, float psize_x, float psize_y, float psize_z )  : m_offset { 0.0, 0.0, 0.0 }, m_distances {NULL}, m_weights {NULL}, m_deformation_nodes{NULL}, m_colours{NULL} {
    if ( ( volume_x > 0 ) && ( volume_y > 0 ) && ( volume_z > 0 ) &&
            ( psize_x > 0 ) && ( psize_y > 0 ) && ( psize_z > 0 ) ) {

        set_size( volume_x, volume_y, volume_z , psize_x, psize_y, psize_z );
    } else {
        throw std::invalid_argument( "Attempt to construct CPUTSDFVolume with zero or negative size" );
    }
}


/**
 * Load a TSDFVolume from the specified file. The volume must previously have been saved
 */
TSDFVolume::TSDFVolume( const std::string& file_name ) {
    using namespace std;
    ifstream ifs{ file_name, ios::in | ios::binary };

    bool success = true;
    std::string specific_error_message = "";

    success = ifs.read( (char *) &m_size, sizeof( m_size ) );
    if( !success ) {
        specific_error_message = "Couldn't load file data";
    } else {
        std::cout << "Loading TSDF with size " << m_size.x << "x" << m_size.y << "x" << m_size.z << std::endl;
    }


    if( success ) {
        success = ifs.read( (char *) &m_physical_size, sizeof( m_physical_size));
        if( success ) {
            std::cout << "  physical size is " << m_physical_size.x << "x" << m_physical_size.y << "x" << m_physical_size.z << "mm" << std::endl;

            // Compute voxel size
            m_voxel_size = f3_div_elem( m_physical_size, m_size );
        } else {
            specific_error_message = "Couldn't load physical size";
        }
    }

    // Load other header stats
    if( success ) {
        success = ifs.read( (char *)&m_offset, sizeof( m_offset));
        success = success && ifs.read( (char *)&m_truncation_distance, sizeof( m_truncation_distance));
        success = success &&ifs.read( (char *)&m_max_weight, sizeof( m_max_weight));
        success = success &&ifs.read( (char *)&m_global_translation, sizeof( m_global_translation));
        success = success &&ifs.read( (char *)&m_global_rotation, sizeof( m_global_rotation));
        if( success ) {
            std::cout << "  read header data" << std::endl;
        } else {
            specific_error_message = "Couldn't load header data";
        }
    }


    // Compute some sizes
    size_t num_voxels = m_size.x * m_size.y * m_size.z;

    // Load distance data
    if( success ) {
        float * host_distances = new float[ num_voxels ];
        if( host_distances ) {
            size_t distance_data_size = num_voxels * sizeof( float);
            cudaError_t err = cudaMalloc( &m_distances, distance_data_size );
            if( err == cudaSuccess ) {
                // Read data into host memory, copy to device and free host memory
                success = ifs.read( ( char * ) host_distances, distance_data_size );
                if( success ) {
                    err = cudaMemcpy( m_distances, host_distances, distance_data_size, cudaMemcpyHostToDevice);
                    if( err == cudaSuccess ) {
                        std::cout << "  loaded distance data" << std::endl;
                    } else {
                        specific_error_message = "Failed to copy distance data to device";
                        success = false;
                        cudaFree( m_distances );
                    }
                    delete[] host_distances;
                } else {
                    specific_error_message = "Failed to read distance data";
                    success = false;
                    delete[] host_distances;
                    cudaFree( m_distances );
                }
            } else {
                specific_error_message = "Failed to allocate device memory for distance data";
                success = false;
                delete[] host_distances;
            }
        } else {
            specific_error_message = "Failed to allocate host memory for distance data";
            success = false;
        }
    }

    // Load weight data
    if( success ) {
        float * host_weights = new float[ num_voxels ];
        if( host_weights ) {
            size_t weight_data_size = num_voxels * sizeof( float);
            cudaError_t err = cudaMalloc( &m_weights, weight_data_size );
            if( err == cudaSuccess ) {
                // Read data into host memory, copy to device and free host memory
                success = ifs.read( ( char * ) host_weights, weight_data_size );
                if( success ) {
                    err = cudaMemcpy( m_weights, host_weights, weight_data_size, cudaMemcpyHostToDevice);
                    if( err == cudaSuccess ) {
                        std::cout << "  loaded weight data" << std::endl;
                    } else {
                        specific_error_message = "Failed to copy weight data to device";
                        success = false;
                        cudaFree( m_weights );
                    }
                    delete[] host_weights;
                } else {
                    specific_error_message = "Failed to read weight data";
                    success = false;
                    delete[] host_weights;
                    cudaFree( m_weights );
                }
            } else {
                specific_error_message = "Failed to allocate device memory for weight data";
                success = false;
                delete[] host_weights;
            }
        } else {
            specific_error_message = "Failed to allocate host memory for weight data";
            success = false;
        }
    }

    // Load colour data
    if( success ) {
        uchar3 * host_colours = new uchar3[ num_voxels ];
        if( host_colours ) {
            size_t colour_data_size = num_voxels * sizeof( uchar3 );
            cudaError_t err = cudaMalloc( &m_colours, colour_data_size );
            if( err == cudaSuccess ) {
                // Read data into host memory, copy to device and free host memory
                success = ifs.read( ( char * ) host_colours, colour_data_size );
                if( success ) {
                    err = cudaMemcpy( m_colours, host_colours, colour_data_size, cudaMemcpyHostToDevice);
                    if( err == cudaSuccess ) {
                        std::cout << "  loaded colour data" << std::endl;
                    } else {
                        specific_error_message = "Failed to copy colour data to device";
                        success = false;
                        cudaFree( m_colours );
                    }
                    delete[] host_colours;
                } else {
                    specific_error_message = "Failed to read colour data";
                    success = false;
                    delete[] host_colours;
                    cudaFree( m_colours );
                }
            } else {
                specific_error_message = "Failed to allocate device memory for colour data";
                success = false;
                delete[] host_colours;
            }
        } else {
            specific_error_message = "Failed to allocate host memory for colour data";
            success = false;
        }
    }


    // Load deformation data
    if( success ) {

        DeformationNode * host_deformations = new DeformationNode[ num_voxels ];
        if( host_deformations ) {
            size_t deformation_data_size = num_voxels * sizeof( DeformationNode );
            cudaError_t err = cudaMalloc( &m_deformation_nodes, deformation_data_size );
            if( err == cudaSuccess ) {
                // Read data into host memory, copy to device and free host memory
                success = ifs.read( ( char * ) host_deformations, deformation_data_size );
                if( success ) {
                    err = cudaMemcpy( m_deformation_nodes, host_deformations, deformation_data_size, cudaMemcpyHostToDevice);
                    if( err == cudaSuccess ) {
                        std::cout << "  loaded deformation data" << std::endl;
                    } else {
                        specific_error_message = "Failed to copy deformation data to device";
                        success = false;
                        cudaFree( m_deformation_nodes );
                    }
                    delete[] host_deformations;
                } else {
                    specific_error_message = "Failed to read deformation data";
                    success = false;
                    delete[] host_deformations;
                    cudaFree( m_deformation_nodes );
                }
            } else {
                specific_error_message = "Failed to allocate device memory for deformation data";
                success = false;
                delete[] host_deformations;
            }
        } else {
            specific_error_message = "Failed to allocate host memory for deformation data";
            success = false;
        }
    }



    ifs.close();

    if( !success ) {
        std::string msg = "Failed to load TSDF ";
        msg += file_name;
        msg += " " + specific_error_message;
        throw std::invalid_argument( msg );
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

    if ( ( volume_x != 0 && volume_y != 0 && volume_z != 0 ) && ( psize_x != 0 && psize_y != 0 && psize_z != 0 ) ) {

        // Remove existing data
        deallocate() ;

        m_size          = dim3 { volume_x, volume_y, volume_z };
        m_physical_size = float3 { psize_x, psize_y, psize_z };

        // Compute truncation distance - must be at least 2x max voxel size
        m_voxel_size = f3_div_elem( m_physical_size, m_size );

        // Set t > diagonal of voxel
        m_truncation_distance = 1.1f * f3_norm( m_voxel_size );

        // Allocate device storage
        cudaError_t err;
		size_t data_size = volume_x * volume_y * volume_z * sizeof( float );

        err = cudaMalloc( &m_distances, data_size );
		check_cuda_error( "Couldn't allocate space for distance data for TSDF", err );

        err = cudaMalloc( &m_weights, data_size );
        check_cuda_error( "Couldn't allocate space for weight data for TSDF", err );

        err = cudaMalloc( &m_colours,  volume_x * volume_y * volume_z * sizeof( uchar3 ) );
        check_cuda_error( "Couldn't allocate space for colour data for TSDF", err );

        err = cudaMalloc( &m_deformation_nodes, volume_x * volume_y * volume_z * sizeof( DeformationNode ) );
        check_cuda_error( "Couldn't allocate space for deformation nodes for TSDF", err );

        m_global_rotation    = make_float3( 0.0f, 0.0f, 0.0f );
        m_global_translation = make_float3( 0.0f, 0.0f, 0.0f );

        clear();

        // Max weight for integrating depth images
        m_max_weight = 15.0f;

    } else {
        throw std::invalid_argument( "Attempt to set TSDF size or physical size to zero" );
    }
}


#pragma mark - Data access

/**
 * Set the distance data for the TSDF in one call
 * @param distance_data Pointer to enough floats to populate the TSFD
 */
void TSDFVolume::set_distance_data( const float * distance_data ) {
    size_t data_size = m_size.x * m_size.y * m_size.z * sizeof( float);
    cudaError_t err = cudaMemcpy( m_distances, distance_data, data_size, cudaMemcpyHostToDevice );
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
 * Set the deformation data for this space
 * @param data Data in host memory space; Assumed to be vx*vy*vz DeformationNode
 */
void TSDFVolume::set_deformation( DeformationNode *deformation) {
    size_t data_size = m_size.x * m_size.y * m_size.z * sizeof( DeformationNode );
    cudaError_t err = cudaMemcpy( m_deformation_nodes, deformation, data_size, cudaMemcpyHostToDevice );
    check_cuda_error( "Couldn't set deformation", err );
}


/**
 * Reset the defomation grid by setting each translation point to the effectve, reglar position
 * in space of that voxel centre and the related rotation to {0,0,0}
 * @param deformation_nodes X x Y x Z array of DeformationNodes
 * @param grid_size The size of the voxel grid
 * @param voxel_size The size of an individual voxel
 * @param grid_offset The offset of the grid
 */
__global__
void initialise_deformation( TSDFVolume::DeformationNode * deformation, dim3 grid_size, float3 voxel_size, float3 grid_offset ) {

    // Extract the voxel Y and Z coordinates we then iterate over X
    int vy = threadIdx.y + blockIdx.y * blockDim.y;
    int vz = threadIdx.z + blockIdx.z * blockDim.z;

    // If this thread is in range
    if ( vy < grid_size.y && vz < grid_size.z ) {

        // The next (x_size) elements from here are the x coords
        size_t base_voxel_index =  ((grid_size.x * grid_size.y) * vz ) + (grid_size.x * vy);

        size_t voxel_index = base_voxel_index;
        for ( int vx = 0; vx < grid_size.x; vx++ ) {
            deformation[voxel_index].translation.x = (( vx + 0.5f ) * voxel_size.x) + grid_offset.x;
            deformation[voxel_index].translation.y = (( vy + 0.5f ) * voxel_size.y) + grid_offset.y;
            deformation[voxel_index].translation.z = (( vz + 0.5f ) * voxel_size.z) + grid_offset.z;

            deformation[voxel_index].rotation.x = 0.0f;
            deformation[voxel_index].rotation.y = 0.0f;
            deformation[voxel_index].rotation.z = 0.0f;

            voxel_index++;
        }
    }
}


__global__
void set_memory_to_value( float * pointer, int size, float value ) {
    int idx = threadIdx.x + (blockIdx.x * blockDim.x );
    if( idx < size ) {
        pointer[idx] = value;
    }
}



/**
 * Clear the TSDF memory on the device
 * zeros colour and weight data, sets distance to truncation_distance
 */
__host__
void TSDFVolume::clear( ) {
    int data_size = m_size.x * m_size.y * m_size.z;


    dim3 block( 1024, 1, 1 );
    dim3 grid ( divUp( data_size, block.x ), 1, 1 );

    cudaError_t err;

    // Clear weights to 0
    set_memory_to_value<<< grid, block >>>( m_weights, data_size, 0.0f );
    cudaDeviceSynchronize( );
    err = cudaGetLastError();
    check_cuda_error( "Couldn't clear weight data", err );


    // Set distance data to truncation distance
    set_memory_to_value<<< grid, block >>>( m_distances, data_size, m_truncation_distance );
    cudaDeviceSynchronize( );
    err = cudaGetLastError();
    check_cuda_error( "Couldn't clear depth data", err );

    // Clear RGB data to black
    err = cudaMemset( m_colours, data_size * 3, 0  );
    check_cuda_error( "Couldn't clear colour data", err );

    // Now initialise the deformations
    dim3 block2( 1, 32, 32 );
    dim3 grid2 ( 1, divUp( m_size.y, block2.y ), divUp( m_size.z, block2.z ) );
    initialise_deformation <<<grid2, block2>>>( m_deformation_nodes, m_size, m_voxel_size, m_offset );
    cudaDeviceSynchronize( );
    err = cudaGetLastError();
    check_cuda_error( "Couldn't initialise deformation nodes", err );
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

    // Convert the input parameters to device (CUDA) types
    Mat44 pose;
    memcpy( &pose, camera.pose().data(), 16 * sizeof( float ) );

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
    check_cuda_error( "Couldn't allocate storage for depth map", err);

    err = cudaMemcpy( d_depth_map, depth_map, data_size, cudaMemcpyHostToDevice );
    check_cuda_error( "Failed to copy depth map to GPU", err);

    // Call the kernel
    dim3 block( 1, 20, 20  );
    dim3 grid ( 1, divUp( m_size.y, block.y ), divUp( m_size.z, block.z ) );

    integrate_kernel <<< grid, block>>>( m_distances, m_weights, m_size, m_physical_size, m_deformation_nodes, m_offset, m_truncation_distance, m_max_weight, pose, inv_pose, k, kinv, width, height, d_depth_map);
    cudaDeviceSynchronize( );
    err = cudaGetLastError();
    check_cuda_error( "Integrate kernel failed", err);

    // Now delete depth map data from device
    err = cudaFree( d_depth_map );
    check_cuda_error( "Failed to deallocate cuda depth map", err);

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

    // We need to extract the data from the GPU device into host memory
    float * host_distances = nullptr;
    uchar3 * host_colours = nullptr;
    float * host_weights = nullptr;
    DeformationNode * host_deformation = nullptr;
    size_t num_voxels = m_size.x * m_size.y * m_size.z;


    cudaError_t err;

    // Copy distance data from device to host
    size_t distance_data_size = num_voxels * sizeof( float);
    host_distances = new float[ num_voxels ];
    if ( host_distances ) {
        err = cudaMemcpy( host_distances, m_distances, distance_data_size, cudaMemcpyDeviceToHost);
        if ( err != cudaSuccess ) {
            success = false;
            std::cout << "Failed to copy voxel data from device memory [" << err << "] " << std::endl;
        }
    } else {
        std::cout << "Couldn't allocate host_voxels memory to save TSDF" << std::endl;
        success = false;
    }


    // Copy weight data from device to host
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

    // Copy colour data from device to host
    size_t colour_data_size = num_voxels * sizeof( uchar3 );
    if ( success ) {
        host_colours = new uchar3[ num_voxels ];
        if ( host_colours) {
            err = cudaMemcpy( host_colours, m_colours, colour_data_size, cudaMemcpyDeviceToHost);
            if ( err != cudaSuccess ) {
                success = false;
                std::cout << "Failed to copy colour data from device memory [" << err << "] " << std::endl;
            }
        } else {
            success = false;
            std::cout << "Couldn't allocate host_colours memory to save TSDF" << std::endl;
        }
    }

    // Copy deformation data from device to host
    size_t deformation_data_size = num_voxels * sizeof( DeformationNode );
    if ( success ) {
        host_deformation = new DeformationNode[ num_voxels ];
        if ( host_deformation ) {
            err = cudaMemcpy( host_deformation, m_deformation_nodes, deformation_data_size, cudaMemcpyDeviceToHost);
            if ( err != cudaSuccess ) {
                success = false;
                std::cout << "Failed to copy deformation data from device memory [" << err << "] " << std::endl;
            }
        } else {
            success = false;
            std::cout << "Couldn't allocate host_weights memory to save TSDF" << std::endl;
        }
    }

    // Now it's all local, write to file
    if( success ) {
        ofstream ofs { file_name, ios::out | ios::binary };

        // Write dimesnions
        std::cout << "  writing "<< sizeof( m_size ) + sizeof( m_physical_size ) <<" bytes of header data" << std::endl;
        ofs.write( (char *) &m_size, sizeof( m_size ) );
        ofs.write( (char *)&m_physical_size, sizeof( m_physical_size));
        ofs.write( (char *)&m_offset, sizeof( m_offset));
        ofs.write( (char *)&m_truncation_distance, sizeof( m_truncation_distance));
        ofs.write( (char *)&m_max_weight, sizeof( m_max_weight));
        ofs.write( (char *)&m_global_translation, sizeof( m_global_translation));
        ofs.write( (char *)&m_global_rotation, sizeof( m_global_rotation));

        std::cout << "  writing "<< distance_data_size <<" bytes of depth data" << std::endl;
        ofs.write( (char *)host_distances, distance_data_size );

        std::cout << "  writing "<< weight_data_size <<" bytes of weight data" << std::endl;
        ofs.write( (char *)host_weights, weight_data_size );

        std::cout << "  writing "<< colour_data_size <<" bytes of colour data" << std::endl;
        ofs.write( (char *)host_colours, colour_data_size );

        std::cout << "  writing "<< deformation_data_size <<" bytes of deformation data" << std::endl;
        ofs.write( (char *)host_deformation, deformation_data_size );

        ofs.close();
    } else {
        std::cout << "Not saving file due to previous errors" << std::endl;
    }

    // Free up memory
    if ( host_distances != nullptr ) { delete[] host_distances; }
    if ( host_colours != nullptr ) { delete[] host_colours; }
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


    std::cout << "Not yet implemented: load_from_file" << std::endl;
    return false;
}


#pragma mark - Rendering
/**
 * Render this TSDF to a raycast image
 */
void TSDFVolume::raycast( uint16_t width, uint16_t height, const Camera& camera, Eigen::Matrix<float, 3, Eigen::Dynamic>& vertices, Eigen::Matrix<float, 3, Eigen::Dynamic>& normals ) const {
    GPURaycaster raycaster( width, height );

    raycaster.raycast( *this, camera, vertices, normals );
}
