#include "../include/SceneFusion.hpp"
#include "../include/MarkAndSweepMC.hpp"
#include "../include/TSDFVolume.hpp"
#include "../include/Camera.hpp"
#include "../include/cuda_utilities.hpp"
#include "../include/cuda_coordinate_transforms.hpp"

#include "vector_types.h"

#include <iostream>
#include <vector>


// Based on voxel size of 50
const float THRESHOLD = 500.0f;


/*
	This is the Plan B variant

	At time t=1
	Compute sf_1 ( D_0, D_1, RGB_0. RGB_1)


	For each (deformed) voxel in grid
		project voxel (VX, VY, VZ) into depth image giving dx, dy
		let d_candidate = D(dx,dy) * Kinv * [dx;dy;1]
		if( |d_candidate - VZ| < threshold ) we can apply scene flow (dx,dy) to
			(VX, VY, VZ) but weighted by distance from d_candidate
		end

*/


__global__
void update_deformation_field(  int    			num_cubes,
								int				* cube_indices,
								float3		 	* deformation_field,
                                const dim3		volume_size,
                                const uint16_t 	* d_depth_data,
                                const float3 	* d_scene_flow,
                                uint16_t		width,
                                uint16_t		height,
                                const Mat33	 	k,
                                const Mat33		kinv,
                                const Mat44		pose,
                                const Mat44		inv_pose,
                                const float     threshold ) {

	int data_index = blockDim.x * blockIdx.x + threadIdx.x;
	if( data_index < num_cubes ) {
		int cube_index = cube_indices[ data_index];

		// Obtain the 8 voxels for this cube
		int voxel_indices[8];
		voxel_indices_for_cube_index( cube_index, volume_size.x, volume_size.y, voxel_indices );


		// For each one of these voxels
		for( int i=0; i<8; i++ ) {
			int voxel_index = voxel_indices[i];

			// project voxel (VX, VY, VZ) into depth image giving pixel coordinates
			float3 voxel_centre = deformation_field[ voxel_index ];
			int3 pixel = world_to_pixel( voxel_centre, inv_pose, k );

			// let d_candidate = D(dx,dy) * Kinv * [dx;dy;1]
			if ( pixel.x < width && pixel.x >= 0 && pixel.y < height && pixel.y >= 0 ) {

				// Look up the depth corresponding to this pixel
				uint pixel_index = pixel.y * width + pixel.x;
				float depth = d_depth_data[ pixel_index ];

				// If it's valid, reproject the point back int space
				if ( depth > 0 ) {

					float3 surface_vertex = pixel_to_world( pixel, pose, kinv, depth );

					// if( |surface_vertex.z - voxel_centre.z| < threshold ) we can apply scene flow (dx,dy) to
					// 	(VX, VY, VZ) but weighted by distance from d_candidate
					float dist = fabs( voxel_centre.z - surface_vertex.z ); 
					if ( dist < threshold ) {
						// Lookup the scene flow
						float3 scene_flow = d_scene_flow[pixel_index];

						// Scale the effect by distance
//						float scale_factor = 1.0f - (dist / threshold);
//						scene_flow = f3_mul_scalar( scale_factor, scene_flow );

						// Apply to the deformation field
						float3 current_deformation = deformation_field[ voxel_index];
						float3 new_deformation = f3_add( current_deformation, scene_flow);
						deformation_field[voxel_index] = new_deformation;
					} // voxel centre is too far from surface, do nothing
				} // Depth is 0, do nothing
			} // Pixel outside frustum
		} // next voxel in cube
	} // data index outside legitimate range
}


__host__
/**
 * 
 */
void process_frames(
    TSDFVolume 		* volume,
    const Camera 	* camera,
    uint16_t 		width,
    uint16_t 		height,
    const uint16_t 	* depth_data,
    const float		* scene_flow_data ) {


	cudaError_t err;
	int num_pixels = width * height;


	// Extract the mesh - but only t get hold of the impacted voxels
	int num_cubes;
	int num_vertices;
	int	*cube_indices;
	float3	*vertices;
	extract_surface_ms( volume, vertices, num_vertices, cube_indices, num_cubes );

	// Discard the vertices
	delete [] vertices;
	

	// Push cube indices back to device
	std::cout << "Copying " << num_cubes << " cube indices to device" << std::endl;
	int * d_cube_indices;
	err = cudaMalloc( &d_cube_indices, num_cubes*sizeof(int) );
	check_cuda_error( "Couldn't allocate storage for cube indices on device", err );
	err = cudaMemcpy( d_cube_indices, cube_indices, num_cubes * sizeof( int ), cudaMemcpyHostToDevice );
	check_cuda_error( "Failed to copy cube indices to device memory" , err );

	// Push scene flow data to device
	// Note: SF data is row major, ie Y,X
	std::cout << "-- Stashing scene flow to device" << std::endl;
	float3 * d_scene_flow;
	err = cudaMalloc( &d_scene_flow, num_pixels * sizeof( float3 ) );
	check_cuda_error( "Couldn't allocate memory for scene flow data", err );
	err = cudaMemcpy( d_scene_flow, scene_flow_data, num_pixels * sizeof( float3 ), cudaMemcpyHostToDevice );
	check_cuda_error( "Failed to copy scene flow data to device" , err );


	// .. and last depth data - which is also row major
	uint16_t * d_depth_data;
	err = cudaMalloc( &d_depth_data, num_pixels * sizeof( uint16_t ) );
	check_cuda_error( "Couldn't allocate memory for depth image", err );
	err = cudaMemcpy( d_depth_data, depth_data, num_pixels * sizeof( uint16_t ), cudaMemcpyHostToDevice );
	check_cuda_error( "Failed to copy depth data to device" , err );


	// prep the matrices
	std::cout << "-- Prepping matrices" << std::endl;
	Mat33 k, kinv;
	memcpy( &k, camera->k().data(), sizeof( float ) * 9 );
	memcpy( &kinv, camera->kinv().data(), sizeof( float ) * 9 );
	Mat44 pose, inv_pose;
	memcpy( &pose, camera->pose( ).data(), sizeof( float ) * 16 );
	memcpy( &inv_pose, camera->inverse_pose().data(), sizeof( float ) * 16 );


	// Invoke kernel to update deformation field
	std::cout << "-- Invoking kernel" << std::endl;
	dim3 block( 512 ); 
	dim3 grid( divUp( num_cubes, block.x ) );
	update_deformation_field <<< grid, block >>>(
		num_cubes,
		d_cube_indices,
	    reinterpret_cast<float3*>( volume->translation_data()),
	    volume->size(),
	    d_depth_data,
	    d_scene_flow,
	    width, height,
	    k, kinv,
	    pose, inv_pose ,
	    THRESHOLD);

	cudaDeviceSynchronize( );
	err = cudaGetLastError( );
	check_cuda_error( "update_deformation_field kernel failed" , err );


	// Clear up after kernel
	std::cout << "-- Cleaning up" << std::endl;
	err = cudaFree( d_scene_flow );
	check_cuda_error( "Failed to free scene flow data" , err );

	err = cudaFree( d_cube_indices );
	check_cuda_error( "Failed to free cube indices memory", err );

	err = cudaFree( d_depth_data );
	check_cuda_error( "Failed to free depth data" , err );

	delete []  cube_indices;
}
