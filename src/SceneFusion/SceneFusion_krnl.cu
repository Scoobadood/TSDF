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
const float THRESHOLD = 10.0f;
const int BLOCK_SIZE = 512;

/* *
 * * Scene fusion plan
 * * Given 
 * *    an input mesh (float3 coordinates, int count of nodes)
 * *    And a list of pairs of voxel indices which bracket them
 * *    And scene flow and depth data
 * *    And Camera data
 * *
 * * Update
 * *    voxel deformation for each pari 
 * *    with scene flow corresponding to mesh vertex
 * * 
 * * Steps
 * *     Find corresponding mesh vertex and pixel coordinates
 * *     Update correspondong voxel deformation with scene flow
 * * 
 * *    For all mesh vertices in parallel
 * *		Project mesh vertex into depth image and back to find correspondence
 * *	    	If in frustum AND depth <> 0, and reprojected close enough
 * *	Requiring
 * *    	Mesh verts (float3), num_verts( int ), depth map, inv_pose, k, pose, inv_k
 * *	Returning
 * *		Binary mesh vert flag per mesh vert, mesh pixel coords and count of true flags
 * *
 * *	Compute scatter addresses for mesh pixe and index data
 * *		For all mesh verts 
 * * 			Work out scatter addresses for pixel coords and mesh vert index
 * *	Requiring
 * *		Mesh vert flags, num mesh verts
 * *	Returning
 * *		Scatter array for mesh data
 * *
 * *	Compact mesh data
 * * 		Requiring
 * *			Mesh pixel projected coords, num eligible mesh verts, mesh vertex voxel indices
 * *		Returning 
 * *			compacted arrays of each
 * * 			
 * *	Update scene flow
 * *		Requiring
 * *			Compacted Mesh pixel projected coords, num eligible mesh verts, mesh vertex voxel indices, scene flow
 * *		
 * */


/* ************************************************************************************************************************
 * * Compute mesh vertex correspondences from depth data
 * * Project each mesh vertex into depth image and back into 3D space
 * * Where the reprojected point is 'close enough' to the origina mesh vertex, a correspondence is deemed
 * * Set a flag in the out put array to indicate this and also store the pixel coordinates
 * * 
 * * Requires:
 * *    	Mesh vertices (float3), num_mesh_verts( int), depth_map (uint16_t *), inv_pose, k, pose, inv_k
 * * Returns:
 * *		bool flag per mesh vert (true if corresponds else false), mesh pixel coords
 * ************************************************************************************************************************/
__global__
void find_mesh_vertex_correspondences( 
	const float3 * const 	d_mesh_vertices,	// in device
	const int 				num_mesh_vertices,	// in
	const uint16_t * const 	depth_map,			// in
	const uint32_t			width,				// in
	const uint32_t			height,				// in
	const Mat33	 			k,					// in
    const Mat33				inv_k,				// in
    const Mat44				pose,				// in
    const Mat44				inv_pose,			// in
    const float     		threshold,			// in
    bool * 					d_corr_flag,		// out (preallocated)
    int * 					d_corr_pixel_index	// out (preallocated)
   	) {
	int mesh_vertex_index = blockDim.x * blockIdx.x + threadIdx.x;
	if( mesh_vertex_index < num_mesh_vertices ) {

		float3 vertex_coord = d_mesh_vertices[ mesh_vertex_index ];
		int3 pixel_coord = world_to_pixel( vertex_coord, inv_pose, k );

		if( pixel_coord.x >= 0 && pixel_coord.x < width && pixel_coord.y >= 0 && pixel_coord.y < height ) {
			int pixel_index = pixel_coord.y * width + pixel_coord.x;
			uint16_t depth = depth_map[ pixel_index ];

			if( depth > 0 ) {
				float3 reprojected_coords = pixel_to_world( pixel_coord, pose, inv_k, depth );

				// Compute the dist on depth only - previous method used lateral distance too
				// float3 delta = f3_sub( reprojected_coords, vertex_coord );
				//	float dist = f3_norm( delta );
				float dist = fabs(reprojected_coords.z - vertex_coord.z );

				if( dist < threshold ) {
					d_corr_flag[ mesh_vertex_index] = true;
					d_corr_pixel_index[ mesh_vertex_index ] = pixel_index;
				}
			}
		}
	}
}


/* ************************************************************************************************************************
 * * Compute scatter addresses for mesh index, pixel index and voxel index data
 * * For all mesh verts
 * * Work out scatter addresses for pixel coords and mesh vert index
 * * Requires:
 * *	Mesh vert flags, num mesh verts
 * * Returns:
 * * 	Scatter array for mesh data plus count of entries
 * ************************************************************************************************************************/
__host__
void compute_mesh_vertex_scatter_array( const bool * const 	d_correspondence_flag, //	Device correspondence flags
										const int 			num_mesh_vertices,
										int *& 				d_scatter_addresses,
										int& 				num_correspondences ) {
	// First get the data off the device
	bool * h_correspondence_flag = new bool[ num_mesh_vertices ];
	if( !h_correspondence_flag) {
		printf( "Couldn't allocate memory to copy correspondence flags to host");
		exit( -1 );
	}
	cudaError_t err = cudaMemcpy( h_correspondence_flag, d_correspondence_flag, sizeof( bool ) * num_mesh_vertices, cudaMemcpyDeviceToHost );
	check_cuda_error( "Failed to copy correspondence flags from device to host", err );

	// Allocate scatter address memory - one address per item
	int * h_scatter_addresses = new int[ num_mesh_vertices ];
	if( !h_scatter_addresses) {
		printf( "Couldn't allocate memory for scatter addresses on host");
		exit( -1 );
	}

	// Do compaction
	int output_index = 0;
	num_correspondences = 0;
	for( int i = 0; i< num_mesh_vertices; i++ ) {
		h_scatter_addresses[i] = output_index;
		if( h_correspondence_flag[i] ) {
			output_index++;
			num_correspondences++;
		}
	}

	// Copy scattered addresses to device
	err = cudaMalloc( &d_scatter_addresses, num_mesh_vertices * sizeof( int ) );
	check_cuda_error( "Failed to allocate scatter address memory on device", err );

	cudaMemcpy( d_scatter_addresses, h_scatter_addresses, num_mesh_vertices * sizeof( int ), cudaMemcpyHostToDevice);
	check_cuda_error( "Failed to copy scatter addresses to  device", err );

	delete [] h_scatter_addresses;
	delete [] h_correspondence_flag;
}

/* ************************************************************************************************************************
 * * Compact all mesh data based on scatter addresses. This includes
 * * Mesh indices, pixel indices, mesh voxel vertices
 * * Requires:
 * *	Mesh pixel indices, mesh vertex indices (implicit in, explicit out), num eligible mesh verts, mesh vertex voxel indices
 * * Returns:
 * *	Compacted arrays of each
 * ************************************************************************************************************************/
__global__
void compact_mesh_data(	const bool * const 	d_correspondence_flags, //	Device correspondence flags
						const int * const 	d_corr_pixel_index,
						const int * const 	d_mesh_vertex_voxel_indices,
						const int * const 	d_scatter_addresses,
						const int 			num_mesh_vertices,
						int * 				d_compact_pixel_indices,				// out
						int * 				d_compact_mesh_vertex_voxel_indices		// out
						) {

	int mesh_vertex_index = blockDim.x * blockIdx.x + threadIdx.x;

	if( mesh_vertex_index < num_mesh_vertices ) {
		if( d_correspondence_flags[ mesh_vertex_index ] ) {

			int compact_index = d_scatter_addresses[ mesh_vertex_index ];

			d_compact_pixel_indices[ compact_index ] = d_corr_pixel_index[ mesh_vertex_index ];
			d_compact_mesh_vertex_voxel_indices[ compact_index * 2 ] = d_mesh_vertex_voxel_indices[ mesh_vertex_index * 2 ];
			d_compact_mesh_vertex_voxel_indices[ compact_index * 2 + 1] = d_mesh_vertex_voxel_indices[ mesh_vertex_index * 2 +1];
		}
	}
}

/* ************************************************************************************************************************
 * * Update scene flow for voxel points related to mesh vertex
 * * Requires:
 * * 	Compacted Mesh pixel projected coords
 * * 	Num compacted entries
 * * 	Compacted mesh vertex voxel indices
 * * 	Scene flow
 * * Returns:
 * * 	Nothing.
 * ************************************************************************************************************************/
__global__
void update_deformation_field(	const int * const 		d_compact_pixel_indices,
							 	const int * const 		d_compact_mesh_vertex_voxel_indices,
							 	const uint8_t * const 	d_mesh_vertex_voxel_count,
								const int 				num_compacted_entries,
								const float3 * const 	d_scene_flow,
								TSDFVolume::DeformationNode * const d_deformation_field)
{
 	int compact_index = blockIdx.x * blockDim.x + threadIdx.x;

 	if( compact_index < num_compacted_entries ) {
 		float3 scene_flow = d_scene_flow[ d_compact_pixel_indices[ compact_index ] ];

 		int voxel_index_1 = d_compact_mesh_vertex_voxel_indices[ compact_index * 2];
 		float scale_factor_1 = ( 1.0f / d_mesh_vertex_voxel_count[ voxel_index_1 ] );
 		d_deformation_field[ voxel_index_1 ].translation = f3_add( d_deformation_field[ voxel_index_1 ].translation, f3_mul_scalar( scale_factor_1, scene_flow ) );

 		int voxel_index_2 = d_compact_mesh_vertex_voxel_indices[ compact_index * 2 + 1];
 		float scale_factor_2 = ( 1.0f / d_mesh_vertex_voxel_count[ voxel_index_2 ] );
 		d_deformation_field[ voxel_index_2 ].translation = f3_add( d_deformation_field[ voxel_index_2 ].translation, f3_mul_scalar( scale_factor_2, scene_flow ) );
 	}
}


__host__
void process_frames(	TSDFVolume *			volume,
						const Camera * const 	camera,
						const uint16_t			width,
						const uint16_t 			height,
						const uint16_t * const 	h_depth_data,
						const float3 * const 	h_scene_flow )
{
	//
	// Extract the mesh from the TSDF
	//
	std::cout << "Processing Frames" << std::endl;

	// Lookup CUDA memory to make sure I'm not leaking it
	size_t mem_tot = 0;
	size_t mem_free = 0;
	cudaMemGetInfo  (&mem_free, & mem_tot);
	std::cout<<"  Free memory : "<<mem_free<<std::endl;


	float3 * 	d_mesh_vertices;
	int 		num_mesh_vertices;
	int * 		d_mesh_vertex_voxel_indices;
	uint8_t *   d_mesh_vertex_voxel_count;
	extract_surface_ms( volume, 						// in
						num_mesh_vertices, 				// out
						d_mesh_vertices,				// out 
						d_mesh_vertex_voxel_indices,	// out
						d_mesh_vertex_voxel_count );	// out
	std::cout << "-- got " << num_mesh_vertices << " mesh vertices" << std::endl;


	//
	// Find mesh correspondences
	//
	Mat33 k, inv_k;
	Mat44 pose, inv_pose;
	memcpy( &k, camera->k( ).data(), sizeof( float ) * 9 );
	memcpy( &inv_k, camera->kinv( ).data(), sizeof( float ) * 9 );
	memcpy( &pose, camera->pose( ).data(), sizeof( float ) * 16 );
	memcpy( &inv_pose, camera->inverse_pose( ).data(), sizeof( float ) * 16 );

	cudaError_t err;

	// Push depth map
	std::cout << "-- pushing depth map" << std::endl;
	int num_pixels = width * height;
	uint16_t * d_depth_data;
	cudaSafeAlloc( (void **) &d_depth_data, num_pixels * sizeof( uint16_t ), "d_depth_data" );
	err = cudaMemcpy( d_depth_data, h_depth_data, num_pixels * sizeof( uint16_t ), cudaMemcpyHostToDevice );
	check_cuda_error( "Failed to copy depth data to device" , err );


	std::cout << "-- prepping for correspondence calculation" << std::endl;
	bool * d_corr_flags;
	cudaSafeAlloc( (void **) &d_corr_flags, num_mesh_vertices * sizeof( bool ), "d_corr_flags" );
	err = cudaMemset( d_corr_flags, 0, num_mesh_vertices * sizeof( bool ) );
	check_cuda_error( "Failed to zero correspondence flags on device", err );

	int  * d_corr_pixel_indices;
	cudaSafeAlloc( (void **) &d_corr_pixel_indices, num_mesh_vertices * sizeof( int ), "d_corr_pixel_indices" );

	dim3 block( BLOCK_SIZE  );
	dim3 grid( divUp( num_mesh_vertices, block.x ) );
	find_mesh_vertex_correspondences<<< grid, block>>> (
			d_mesh_vertices,					// in
			num_mesh_vertices,					// in
			d_depth_data,						// in
			width, height,						// in
			k, inv_k, pose,	inv_pose, THRESHOLD,// in
			d_corr_flags, 						// out
			d_corr_pixel_indices );				// out
	cudaDeviceSynchronize( );
	err = cudaGetLastError( );
	check_cuda_error( "Kernel find_mesh_vertex_correspondences failed", err );

	//
	// Dispose of depth map now as it's not needed any more
	//
	cudaSafeFree( d_depth_data, "d_depth_data" );

	// And no need for d_mesh_vertices  either now we have compacted
	cudaSafeFree( d_mesh_vertices, "d_depth_data" );


	//
	// Compute scatter addresses for compaction
	//
	int num_compacted_entries = 0;
	int * d_scatter_addresses = nullptr;
	compute_mesh_vertex_scatter_array(	d_corr_flags, 
										num_mesh_vertices, 		// in
										d_scatter_addresses, 
										num_compacted_entries );	//	out
	std::cout << "   -- got " << num_compacted_entries << " correspondences " << std::endl;

	//
	// Perform compaction of pixel and voxel verts and 
	//
	std::cout << "-- compacting vectors" << std::endl;
	int * d_compact_pixel_indices;
	cudaSafeAlloc( (void **) &d_compact_pixel_indices, num_compacted_entries * sizeof( int ) , "d_compact_pixel_indices" );

	int * d_compact_mesh_vertex_voxel_indices;
	cudaSafeAlloc( (void **) &d_compact_mesh_vertex_voxel_indices, num_compacted_entries * 2 * sizeof( int ), "d_compact_mesh_vertex_voxel_indices" );
	
	compact_mesh_data<<< grid, block>>> (	d_corr_flags, 
											d_corr_pixel_indices, 
											d_mesh_vertex_voxel_indices,  
											d_scatter_addresses, 
											num_mesh_vertices,	// in
											d_compact_pixel_indices, 
											d_compact_mesh_vertex_voxel_indices ); // out
	cudaDeviceSynchronize( );
	err = cudaGetLastError( );
	check_cuda_error( "Kernel compact_mesh_data failed", err );

	cudaSafeFree( d_scatter_addresses, "d_scatter_addresses");
	cudaSafeFree( d_corr_flags, "d_corr_flags" );
	cudaSafeFree( d_corr_pixel_indices, "d_corr_pixel_indices" );
	cudaSafeFree( d_mesh_vertex_voxel_indices, "d_mesh_vertex_voxel_indices" );
	std::cout << "  done" << std::endl;


	//
	//	Update the deformation field
	//

	// Push scene flow data to device
	// Note: SF data is row major, ie Y,X
	std::cout << "-- Stashing scene flow to device" << std::endl;
	float3 * d_scene_flow;
	err = cudaMalloc( &d_scene_flow, num_pixels * sizeof( float3 ) );
	check_cuda_error( "Couldn't allocate memory for scene flow data", err );
	err = cudaMemcpy( d_scene_flow, h_scene_flow, num_pixels * sizeof( float3 ), cudaMemcpyHostToDevice );
	check_cuda_error( "Failed to copy scene flow data to device" , err );

	std::cout << "-- Updating deformation field" << std::endl;
	dim3 grid2( divUp( num_compacted_entries, block.x ) );
	update_deformation_field<<< grid2, block  >>>(	
			d_compact_pixel_indices, 
			d_compact_mesh_vertex_voxel_indices, 
			d_mesh_vertex_voxel_count, 
			num_compacted_entries,
			d_scene_flow, 
			volume->deformation( ) );

	cudaDeviceSynchronize( );
	err = cudaGetLastError( );
	check_cuda_error( "Kernel update_deformation_field failed", err );
	std::cout << "  done" << std::endl;

	//
	// Tidy Up
	//
	err = cudaFree( d_scene_flow );
	check_cuda_error( "Couldn't free scene flow memory from device", err );

	err = cudaFree( d_compact_pixel_indices );
	check_cuda_error( "Couldn't free compact pixel indices memory from device", err );

	err = cudaFree( d_compact_mesh_vertex_voxel_indices );
	check_cuda_error( "Couldn't free compact mesh vertex voxel indices  memory from device", err );

	err = cudaFree( d_mesh_vertex_voxel_count);
	check_cuda_error( "Couldn't free compact mesh vertex voxel count  memory from device", err );
}
