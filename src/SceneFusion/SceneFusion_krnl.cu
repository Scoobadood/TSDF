#include "../include/SceneFusion.hpp"
#include "../include/GPUMarchingCubes.hpp"
#include "../include/TSDFVolume.hpp"
#include "../include/Camera.hpp"
#include "../include/cuda_utilities.hpp"

#include "vector_types.h"

#include <iostream>
#include <vector>



const float THRESHOLD = 2.0f;


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




__device__
/**
 * Convert global coordinates into pixel coordinates
 * Multiply by pose.inverse(), then K
 * @param world_coordinate The 3D point in world space
 * @return pixel_coordinate The 2D point in pixel space
 */
int3 world_to_pixel( const float3 & world_coordinate, const Mat44 & inv_pose, const Mat33 & k ) {
	float3 cam_coordinate;
	cam_coordinate.x = inv_pose.m11 * world_coordinate.x + inv_pose.m12 * world_coordinate.y + inv_pose.m13 * world_coordinate.z + inv_pose.m14;
	cam_coordinate.y = inv_pose.m21 * world_coordinate.x + inv_pose.m22 * world_coordinate.y + inv_pose.m23 * world_coordinate.z + inv_pose.m24;
	cam_coordinate.z = inv_pose.m31 * world_coordinate.x + inv_pose.m32 * world_coordinate.y + inv_pose.m33 * world_coordinate.z + inv_pose.m34;


	// Push into camera image
	float3 image_coordinate;
	image_coordinate.x = k.m11 * cam_coordinate.x + k.m12 * cam_coordinate.y + k.m13 * cam_coordinate.z;
	image_coordinate.y = k.m21 * cam_coordinate.x + k.m22 * cam_coordinate.y + k.m23 * cam_coordinate.z;
	image_coordinate.z = k.m31 * cam_coordinate.x + k.m32 * cam_coordinate.y + k.m33 * cam_coordinate.z;

	// Round and store
	int3 pixel_coordinate;
	pixel_coordinate.x = round( image_coordinate.x / image_coordinate.z);
	pixel_coordinate.y = round( image_coordinate.y / image_coordinate.z);

	return pixel_coordinate;
}

/**
 * Convert pixel coordinates into world coordinates via a depth
 * Multiply by k.inverse(), project by depth, then pose
 * @param pixel The 2d pixel coordinate
 * @param depth The depth value of
 * @return world coordinate in 3D space
 */
__device__
float3 pixel_to_world( const int3 pixel, const Mat44 & pose, const Mat33 & inv_k, float depth ) {
	// From image to camera plane
	float3 image_plane_coords {
		inv_k.m11 * pixel.x + inv_k.m12 * pixel.y + inv_k.m13,
		inv_k.m21 * pixel.x + inv_k.m22 * pixel.y + inv_k.m23,
		inv_k.m31 * pixel.x + inv_k.m32 * pixel.y + inv_k.m33
	};

	// TODO
	// The actual calc here should be to scale the cam coords so tat Z == depth
	// There's an implicit assumption here that the Z coord is 1 which may not be true
	float3 cam_coords = f3_mul_scalar( depth, image_plane_coords);


	// Back to worl dcoords
	float3 world_coords;
	world_coords.x = pose.m11 * cam_coords.x + pose.m12 * cam_coords.y + pose.m13 * cam_coords.z + pose.m14;
	world_coords.y = pose.m21 * cam_coords.x + pose.m22 * cam_coords.y + pose.m23 * cam_coords.z + pose.m24;
	world_coords.z = pose.m31 * cam_coords.x + pose.m32 * cam_coords.y + pose.m33 * cam_coords.z + pose.m34;

	float w = pose.m41 * cam_coords.x + pose.m42 * cam_coords.y + pose.m43 * cam_coords.z + pose.m44;

	world_coords.x /= w;
	world_coords.y /= w;
	world_coords.z /= w;

	return world_coords;
}


__global__
void update_deformation_field(  float3		 	* deformation_field,
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

	uint voxel_y = (blockDim.y * blockIdx.y) + threadIdx.y;
	uint voxel_z = (blockDim.z * blockIdx.z) + threadIdx.z;


	if ( voxel_y < volume_size.y && voxel_z < volume_size.z ) {

		uint voxel_index = voxel_z * ( volume_size.x * volume_size.y) + (voxel_y * volume_size.x );

		for ( uint voxel_x = 0 ; voxel_x < volume_size.x; voxel_x ++ ) {

			// project voxel (VX, VY, VZ) into depth image giving dx, dy
			float3 voxel_centre = deformation_field[ voxel_index ];

			int3 pixel = world_to_pixel( voxel_centre, inv_pose, k );


			// let d_candidate = D(dx,dy) * Kinv * [dx;dy;1]
			if ( pixel.x < width && pixel.x >= 0 && pixel.y < height && pixel.y >= 0 ) {
				uint pixel_index = pixel.y * width + pixel.x;

				float depth = d_depth_data[ pixel_index ];

				if ( depth > 0 ) {

					float3 surface_vertex = pixel_to_world( pixel, pose, kinv, depth );

					// if( |d_candidate - VZ| < threshold ) we can apply scene flow (dx,dy) to
					// 	(VX, VY, VZ) but weighted by distance from d_candidate
					float3 delta_vector = f3_sub(surface_vertex, voxel_centre);
					float dist = f3_norm(delta_vector);
					if ( dist < threshold ) {
						// Lookup the scene flow
						float3 scene_flow = d_scene_flow[pixel_index];

						// Scale the effect by distance
						float scale_factor = 1.0f - (dist / threshold);
						scene_flow = f3_mul_scalar( scale_factor, scene_flow );

						float3 current_deformation = deformation_field[ voxel_index];
						float3 new_deformation = f3_add( current_deformation, scene_flow);
						deformation_field[voxel_index] = new_deformation;
					} // voxel centre is too far from surface
				} // Depth is 0, do nothing
			}
			voxel_index++;
		}

	}
}


__host__


void process_frames(
    TSDFVolume 		* volume,
    const Camera 	* camera,
    uint16_t 		width,
    uint16_t 		height,
    const uint16_t 	* depth_data,
    const float		* scene_flow_data ) {


	cudaError_t err;
	int num_pixels = width * height;


	// Push scene flow data to device
	std::cout << "-- Stashing scene flow to dvice" << std::endl;
	float3 * d_scene_flow;
	err = cudaMalloc( &d_scene_flow, num_pixels * sizeof( float3 ) );
	check_cuda_error( "Couldn't allocate memory for scene flow data", err );
	err = cudaMemcpy( d_scene_flow, scene_flow_data, num_pixels * sizeof( float3 ), cudaMemcpyHostToDevice );
	check_cuda_error( "Failed to copy scene flow data to device" , err );


	// .. and last depth data
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


	// Invoke kernel to update defomrmation field
	std::cout << "-- Invoking kernel" << std::endl;
	dim3 block( 1, 16, 16 );
	dim3 grid( 1, divUp( volume->size().y, block.y ), divUp( volume->size().z, block.z) );
	update_deformation_field <<< grid, block >>>(
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
	cudaFree( d_scene_flow );
	check_cuda_error( "Failed to free scene flow data" , err );

	cudaFree( d_depth_data );
	check_cuda_error( "Failed to free depth data" , err );
}