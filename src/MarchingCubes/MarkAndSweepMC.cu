#include "../include/TSDFVolume.hpp"
#include "../include/cuda_utilities.hpp"
#include "../include/TSDF_utilities.hpp"
#include "vector_types.h"

#include "MC_edge_table.cu"
#include "MC_triangle_table.cu"

/*
 *
 *            4--------(4)---------5
 *           /|                   /|
 *          / |                  / |
 *         /  |                 /  |
 *       (7)  |               (5)  |
 *       /    |               /    |
 *      /    (8)             /    (9)
 *     /      |             /      |
 *    7---------(6)--------6       |
      |       |            |       |
 *    |       0------(0)---|-------1
 *    |      /             |      /
 *   (11)   /             (10)   /
 *    |    /               |    /
 *    |  (3)               |  (1)
 *    |  /                 |  /
 *    | /                  | /
 *    |/                   |/
 *    3---------(2)--------2
 *
 * where X axis is horizontal, +ve to right
 *       Y axis is vertical, +ve upwards
 *       Z axis is into page, +ve towards back
 */


/*
* We first find all grid cells that contain a zero crossing based on a data-parallel prefix sum. One thread per valid grid cell is
* used to extract the final list of triangles. Since each valid grid cell potentially generates multiple triangles, we use atomic
* counters to manage the list. The resulting vertices are immediately deformed according to the current deformation field,
* resulting in a polygonal approximation of P.
*/



/* ******************************************************************************************
 * **                                                                                      **
 * **  UTILITIES                                                                           **
 * **                                                                                      **
 * **                                                                                      **
 * ******************************************************************************************/


__device__
float3 interpolate( float3 v0, float3 v1, float w0, float w1 ) {
	if ( ( w0 > 0 ) && ( w1 < 0 ) ) {
		float tw = w0;  w0 = w1; w1 = tw;
		float3 tv = v0; v0 = v1; v1 = tv;
	}


	// w0 < 0, w1 > 0 (or else they're the same sign which doesn't phase me)
	float3 delta = f3_sub( v1, v0 );

	// w1 and w0 should not be the same, by definition, they should have opposite signd
	// If they do have the same sign and value, then the result is not going to be used
	// so we don't police it here.
	float ratio = -(w0) / (w1 - w0);

	return f3_add( v0, f3_mul_scalar( ratio, delta ) );
}




/**
 * A TSDF contains a number of voxels. Each cluster of 8 adjacent voxels form a cube. Cubes are numbered from 0
 * to a maximum cube index of ((vx-1) * (vy-1) * (vz-1)) where vx, vy and vz are voxel indices.
 * This method returns the voxel indices of the 8 voxels whose centres form the cube given by the cube index
 * @param cube_index 	The index of the cube under consideration
 * @param grid_size 	The size of the TSDF in voxels
 * @param voxel_indices An array of 8 ints which is preallocated an populated by this function with the indices of the
 *						8 voxels forming this cube. The order of the voxels is as shown in the image at the top of the file
 * @param voxel_coords	An array of 8 dim3s which is preallocated and populated by this function with the x,y,z coordinates of the
 *						8 voxels forming this cube. The order of the voxels is as shown in the image at the top of the file. 
 *						If null, this is ignored
 */
__device__
void get_indices_of_voxels_for_cube(const int 	cube_index, 
									const dim3 	grid_size, 
									int 		voxel_indices[8], 
									dim3 	 	voxel_coords[8] ) {

	// Compute the x, y and z indices of the cube.
	int cube_xy_slab_size = ( ( grid_size.x - 1) * (grid_size.y - 1));
	unsigned int cube_z = cube_index / cube_xy_slab_size;
	unsigned int cube_y = ( cube_index - (cube_z * cube_xy_slab_size) ) / (grid_size.x - 1);
	unsigned int cube_x = ( cube_index - (cube_z * cube_xy_slab_size) ) % (grid_size.x - 1);

	// Get the voxel index of the bottom front left cube
	int front_left_bottom_voxel_index = ( cube_z * grid_size.x * grid_size.y) + (cube_y * grid_size.x) + cube_x;

	// Xomputethe voxl offsets for increments in x, y and z directions
	int dx = 1;
	int dy = grid_size.x;
	int dz = grid_size.x * grid_size.y;

	// Compute voxel indices for other voxels
	voxel_indices[0] = front_left_bottom_voxel_index + 		dz;		// back left bottom
	voxel_indices[1] = front_left_bottom_voxel_index + dx + dz;		// back rght bottom
	voxel_indices[2] = front_left_bottom_voxel_index + dx;			// frnt rght bottom
	voxel_indices[3] = front_left_bottom_voxel_index;				// frnt left bottom
	voxel_indices[4] = voxel_indices[0] + dy;						// back left top
	voxel_indices[5] = voxel_indices[1] + dy;						// back rght top
	voxel_indices[6] = voxel_indices[2] + dy;						// frnt rght top
	voxel_indices[7] = voxel_indices[3] + dy;						// frnt left top

	if( voxel_coords != nullptr ) {
		voxel_coords[0] = { cube_x    , cube_y    , cube_z + 1};
		voxel_coords[1] = { cube_x + 1, cube_y    , cube_z + 1};
		voxel_coords[2] = { cube_x + 1, cube_y    , cube_z    };
		voxel_coords[3] = { cube_x    , cube_y    , cube_z    };
		voxel_coords[4] = { cube_x    , cube_y + 1, cube_z + 1};
		voxel_coords[5] = { cube_x + 1, cube_y + 1, cube_z + 1};
		voxel_coords[6] = { cube_x + 1, cube_y + 1, cube_z    };
		voxel_coords[7] = { cube_x    , cube_y + 1, cube_z    };
	}
}

/**
 * For a given cube in the TSDF specified by the indices of the voxels forming it's vertices,
 * compute the cube type
 * @param voxel_indices The indices of the voxels forming the vertices of the cube. Order as per diagram at top of page
 * @param distance_data The array of distances from the TSDF which define the isosurface
 * @return The cube index, 0-255
 */
__device__
uint8_t calculate_cube_type( const int 				voxel_indices[8],
                             const float * const 	distance_data ) {
	// Compute the cube_type
	uint8_t cube_type = 0;
	cube_type |= ( ( distance_data[ voxel_indices[0] ] < 0 ) << 0 );
	cube_type |= ( ( distance_data[ voxel_indices[1] ] < 0 ) << 1 );
	cube_type |= ( ( distance_data[ voxel_indices[2] ] < 0 ) << 2 );
	cube_type |= ( ( distance_data[ voxel_indices[3] ] < 0 ) << 3 );
	cube_type |= ( ( distance_data[ voxel_indices[4] ] < 0 ) << 4 );
	cube_type |= ( ( distance_data[ voxel_indices[5] ] < 0 ) << 5 );
	cube_type |= ( ( distance_data[ voxel_indices[6] ] < 0 ) << 6 );
	cube_type |= ( ( distance_data[ voxel_indices[7] ] < 0 ) << 7 );

	return cube_type;
}



/* ******************************************************************************************
 * **                                                                                      **
 * **  KERNELS                                                                             **
 * **                                                                                      **
 * **                                                                                      **
 * ******************************************************************************************/


/**
 * Execute for each potential cube in the TSDF (ie a cube rooted at a specific voxel).
 * This kernel computes the number of vertcies that this cube will contribute to the surface - which may be 0
 * 
 * @param grid_size 			The size of the TSDF in voxels. Passed in.
 * @param num_cubes				The number of cubes being checked  (and dimension of the output array)
 * @param distance_data 		The distance data from the TSDF indexed [z][y][x].
 * @param d_vertices_per_cube	A Pre-allocated array with size equal to the number of cubes in the 
 * 								TSDF (vx-1 * vy-1 * vz-1) On exit, this is populated with the count of
 *								vertices that this cube contributes to the mesh.
 */
__global__
void krnl_get_num_mesh_vertices_for_cube(	const dim3 			grid_size,
                            				const float * const distance_data,
                            				const int 			num_cubes,
                            				uint8_t * const 	d_vertices_per_cube)
{
	int cube_index = blockIdx.x * blockDim.x  + threadIdx.x;
	if ( cube_index < num_cubes) {

		// Get the list of indices of the voxels which form this cube
		int voxel_indices[8];
		get_indices_of_voxels_for_cube( cube_index, grid_size, voxel_indices, nullptr);

		// Determine the type of the cube from the voxel indices
		int cube_type = calculate_cube_type( voxel_indices, distance_data );

		// Lookup the number of vertices contributed by this type of cube 
		uint8_t num_verts_in_cube = VERTICES_FOR_CUBE_TYPE[ cube_type ];

		// Store this in the output array
		d_vertices_per_cube[ cube_index ] = num_verts_in_cube;
	}
}


/**
 * Execute for every cube which is intersected by the isosurface
 * Generates a list of vertex coordinates -- in the world coordinate frame -- for the intersections
 * @param grid_size 		The size of the grid. Used to translate a cube index into a voxel index
 * @param distance_data 	The Isosurface data
 * @param offset 			The offset of the TSDF in world coordinates

 * @param num_cubes 		Number of cube entries (data items) to process
 * @param cube_indices 		Array of indices of the cubes which encapsulate the surface (size is num_cubes)

 * @param write_indices 	Offsets in the output array at which a cube should write its output
 * @param d_mesh_vertices 	The device array (preallocated) to which the mesh vertcies should be written
 * @param d_m_dash 			Preallocated byte array of flags which will be set if the correspodning voxel is adjacent to a vertex, otherwise cleared
 */
__global__
void krnl_generate_vertices( const dim3 			grid_size, 						// in
	                         const float * const 	distance_data,					// in (device)
    	                     const float3 			offset,							// in
        	                 const float3 			voxel_size,
                	         const int 				num_cubes,						// in
            	             const int * const		cube_indices,					// in (device)
                    	     const int * const 		write_indices,					// in
                        	 float3 * const 		d_mesh_vertices,				// out (preallocated device)
	                         uint8_t * const 		d_m_dash )						// out preallocated
{
	// Get the index of the data item we're to deal with. Each item is a cube index (plus related offset in outputs)
	int data_index = blockIdx.x * blockDim.x + threadIdx.x;
	if ( data_index < num_cubes ) {

		// Pick the cube index to address
		int cube_index = cube_indices[ data_index ];

		// Now get voxel indices (and coords) for the vertices of the cube
		int voxel_indices[8];
		dim3 voxel_coords[8];
		get_indices_of_voxels_for_cube( cube_index, grid_size, voxel_indices, voxel_coords );

		// Get weights of each vertex
		float w0 = distance_data[ voxel_indices[0] ];
		float w1 = distance_data[ voxel_indices[1] ];
		float w2 = distance_data[ voxel_indices[2] ];
		float w3 = distance_data[ voxel_indices[3] ];
		float w4 = distance_data[ voxel_indices[4] ];
		float w5 = distance_data[ voxel_indices[5] ];
		float w6 = distance_data[ voxel_indices[6] ];
		float w7 = distance_data[ voxel_indices[7] ];

		// Get the vertex coordinates in world frame
		float3 v0 = centre_of_voxel_at( voxel_coords[0].x, voxel_coords[0].y, voxel_coords[0].z, voxel_size, offset );
		float3 v1 = centre_of_voxel_at( voxel_coords[1].x, voxel_coords[1].y, voxel_coords[1].z, voxel_size, offset );
		float3 v2 = centre_of_voxel_at( voxel_coords[2].x, voxel_coords[2].y, voxel_coords[2].z, voxel_size, offset );
		float3 v3 = centre_of_voxel_at( voxel_coords[3].x, voxel_coords[3].y, voxel_coords[3].z, voxel_size, offset );
		float3 v4 = centre_of_voxel_at( voxel_coords[4].x, voxel_coords[4].y, voxel_coords[4].z, voxel_size, offset );
		float3 v5 = centre_of_voxel_at( voxel_coords[5].x, voxel_coords[5].y, voxel_coords[5].z, voxel_size, offset );
		float3 v6 = centre_of_voxel_at( voxel_coords[6].x, voxel_coords[6].y, voxel_coords[6].z, voxel_size, offset );
		float3 v7 = centre_of_voxel_at( voxel_coords[7].x, voxel_coords[7].y, voxel_coords[7].z, voxel_size, offset );

		// Compute the edge intersections (some may not be valid)
		float3 vertex[12];
		vertex[0] = interpolate(  v0, v1, w0, w1 );
		vertex[1] = interpolate(  v2, v1, w2, w1 );
		vertex[2] = interpolate(  v3, v2, w3, w2 );
		vertex[3] = interpolate(  v3, v0, w3, w0 );

		vertex[4] = interpolate(  v4, v5, w4, w5 );
		vertex[5] = interpolate(  v6, v5, w6, w5 );
		vertex[6] = interpolate(  v7, v6, w7, w6 );
		vertex[7] = interpolate(  v7, v4, w7, w4 );

		vertex[8] = interpolate(  v0, v4, w0, w4 );
		vertex[9] = interpolate(  v1, v5, w1, w5 );
		vertex[10] = interpolate( v2, v6, w2, w6 );
		vertex[11] = interpolate( v3, v7, w3, w7 );

		// Extract the vertex offset;
		int edge_index;		
		int i = 0;
		int output_index = write_indices[ data_index ];
		int cube_type = calculate_cube_type( voxel_indices, distance_data );
		while ( (edge_index = TRIANGLE_TABLE[cube_type][i]) != -1 ) {

			// Stash the vertex data
			d_mesh_vertices[ output_index++ ] = vertex[edge_index];

			//	Get the indices of the vertex end points
			int voxel_index_1 = voxel_indices[ EDGE_VERTICES[edge_index][0] ];
			int voxel_index_2 = voxel_indices[ EDGE_VERTICES[edge_index][1] ];

			// Set the flags for these to true
			d_m_dash[ voxel_index_1] = true;
			d_m_dash[ voxel_index_2] = true;

			i++;
		}
	}
}


/* ******************************************************************************************
 * **                                                                                      **
 * **  HOST FUNCTIONS                                                                      **
 * **                                                                                      **
 * **                                                                                      **
 * ******************************************************************************************/
/**
 * Launch classify voxel kernel
 * @param volume 			The TSDFVolume. Not modified by this method. Provides the raw data
 * @param num_cubes 		The number of cubes in the TSDF
 * @param vertices_per_cube An array of num_cubes uint8s which will be populated with the count of vertices contributed by the given cube.
 * @param num_occupied_cubes The number of cubes in the space that contribute to the mesh. Populated by this call
 * @param num_vertices 		The total number of vertices contributed by all cubes. Populated by this call
 */
__host__
void launch_get_num_mesh_vertices_for_cube(	const TSDFVolume * const 	volume,
           			                        int 						num_cubes,
                    		                uint8_t *& 					vertices_per_cube,
                            		        int& 						num_occupied_cubes,
                                    		int& 						num_vertices ) {

	dim3 grid_size = volume->size( );

	// Allocate storage for the vertex per cube count
	uint8_t * d_vertices_per_cube;
	cudaSafeAlloc( 	(void **) &d_vertices_per_cube, num_cubes * sizeof( uint8_t ), "d_vertices_per_cube" );

	// invoke the kernel
	dim3 block( 512 );
	dim3 grid( divUp( num_cubes, block.x ) );
	krnl_get_num_mesh_vertices_for_cube <<< grid, block >>>( grid_size, volume->distance_data(), num_cubes, d_vertices_per_cube );
	cudaDeviceSynchronize( );
	cudaError_t err = cudaGetLastError( );
	check_cuda_error( "krnl_get_num_mesh_vertices_for_cube failed", err );

	// Copy vertex count back to host
	cudaSafeCopyToHost( vertices_per_cube, d_vertices_per_cube, num_cubes * sizeof( uint8_t), "vertices_per_cube");
	cudaSafeFree( d_vertices_per_cube, "d_vertices_per_cube");

	// compute occupied cubes and num_vertices
	num_occupied_cubes = 0;
	num_vertices = 0;
	for ( int i = 0; i < num_cubes; i++ ) {
		num_vertices += vertices_per_cube[i];
		if ( vertices_per_cube[i] > 0 ) {
			num_occupied_cubes++;
		}
	}
}

/* ******************************************************************************** */



/**
 * Generate vertices for each cube. vertices are generated into the vertices array (space is allocated in this method)
 * Triangles of the mesh are implicitly formed by every 3 vertices
 * @param volume 				The TSDF volume source of the data
 * @param num_cubes 			The number of cubes bing considered and size of cube_indices nd thread_write_offsets
 * @param cube_indices 			The indices of the cubes which contribute to the surface. Note that cube indices run to grid x,y and z LESS 1
 * @param thread_write_offsets 	The offset (per cube) into d_mesh_vertices to which vertices for that cube should be written. 
 * 								Offsets are in vertex count rather than bytes
 * @param num_vertices 			The total number of vertices expected to process
 * @param d_mesh_vertices 		Preallocated device array of num_vertices float3. Populated by this mthod with the actual vertex data
 * @param d_m_dash 				Preallocated device array of num_voxels byte flags. Each flag will be set to true if the corresponding gridpoint in the TSDF 
 * 								(ie voxel centre) is adjacent to the extracted mesh and false if not. 
 */
void launch_generate_vertices(	const TSDFVolume	* volume,				// in
                                int 				num_cubes,				// in
                                const int * const 	cube_indices,			// in
                                const int * const 	thread_write_offsets,	// in
                                int 				num_vertices,			// in
                                float3 * const		d_mesh_vertices,		// out
                                uint8_t* const		d_m_dash)				// out
{	
	// Copy cube indices to device
	std::cout << "   -- copying cube indices to device" << std::endl;
	int * d_cube_indices;
	cudaSafeAlloc( (void **) &d_cube_indices, num_cubes * sizeof( int ), "d_cube_indices" );
	cudaSafeCopyToDevice( (void *) cube_indices, (void *) d_cube_indices, num_cubes * sizeof( int ), "d_cube_indices");

	// Copy vertex offsets to device
	std::cout << "   -- copying vertex scatter addresses to device" << std::endl;
	int * d_thread_write_offsets;
	cudaSafeAlloc( (void **) &d_thread_write_offsets, num_cubes * sizeof( int ), "d_thread_write_offsets");
	cudaSafeCopyToDevice( (void *) thread_write_offsets, (void *) d_thread_write_offsets, num_cubes * sizeof( int ), "d_thread_write_offsets");

	std::cout << "   -- running generation" << std::endl;
	dim3 block( 512 );
	dim3 grid( divUp( num_cubes, block.x ));
	krnl_generate_vertices <<< grid, block >>>(	volume->size(),							// in
	                                        	volume->distance_data(),				// in
	                                        	volume->offset(),						// in
                        						(float3)volume->voxel_size(),
	                                        	num_cubes,								// in
	                                        	d_cube_indices,							// in
	                                        	d_thread_write_offsets,					// in
	                                        	d_mesh_vertices,						// out
	                                        	d_m_dash);								// out
	cudaDeviceSynchronize();
	cudaError_t err = cudaGetLastError( );
	check_cuda_error( "Generate vertices kernel failed", err );

	cudaSafeFree( d_cube_indices, "d_cube_indices" );
	cudaSafeFree( d_thread_write_offsets, "d_thread_write_offsets" );
}

/**
 * Extract the surface from the given TSDF using mark and sweep MC
 * @param volume 			The TSDF
 * @param num_vertices 		Populated by the method, returns the total number of vertices found
 * @param d_mesh_vertices 	A ref to a float3 pointer. This is allocated by the function and on return points to the vertex data, 
 * 							Vertex data is arranged in triangles oriented anti-clockwise.
 * @param d_m_dash 			A pointer to a set of flags (one per voxel in the TSDF) popu;ated with true or false as the voxel is in M'
 */
void extract_surface_ms( const TSDFVolume * const 	volume, 					// in
                         int& 						num_vertices,				// out 
                         float3 * & 				d_mesh_vertices,			// out (allocated here)
                         uint8_t * const 			d_m_dash)
{

	std::cout << "Extracting surface" << std::endl;

	// Each cube is a block of 8 adjacent voxels.
	dim3 grid_size = volume->size();
	int num_cubes = (grid_size.x - 1 ) * ( grid_size.y - 1) * ( grid_size.z - 1 );

	/* **************************************************************************************************************
	 * *                                                                                                            *
	 * *  Get each cube's contribution to vertices                                                                  *
	 * *                                                                                                            *
	 * **************************************************************************************************************/

	std::cout << "-- get individual cube contribution" << std::endl;
	// Allocate host storage for number of vertices per cube
	uint8_t *vertices_per_cube = new uint8_t[ num_cubes ];
	if ( !vertices_per_cube) {
		std::cout << "Couldn't allocate storage for verts per cube on host" << std::endl;
		exit( -1 );
	}

	// Launch the kernel to populate this
	int num_occupied_cubes = 0;
	launch_get_num_mesh_vertices_for_cube( volume, num_cubes, vertices_per_cube, num_occupied_cubes, num_vertices );
	std::cout << "-- found " << num_occupied_cubes << " occupied cubes and " << num_vertices << " vertices" << std::endl;

	// Sanity check
	if ( num_occupied_cubes == 0 || num_vertices == 0 ) {
		std::cout << "Either no occupied cubes or no vertices. Either way a bit sus." << std::endl;
		exit( -1 );
	}


	/* **************************************************************************************************************
	 * *                                                                                                            *
	 * *  Compute a prefix sum for offsets of vertex output data                                                    *
	 * *                                                                                                            *
	 * **************************************************************************************************************/
	// Allocate storage for thread write offets
	std::cout << "-- allocating storage for " << num_occupied_cubes << " offsets" << std::endl;
	int * thread_write_offsets = new int[ num_occupied_cubes ];
	if ( !thread_write_offsets ) {
		std::cout << "Couldn't allocate host storage for thread write offsets" << std::endl;
		delete[] vertices_per_cube;
		exit( -1 );
	}

	// Allocate storage for cube indices
	std::cout << "-- allocating storage for " << num_occupied_cubes << " cube indices" << std::endl;
	int * cube_indices = new int[ num_occupied_cubes ];
	if ( !cube_indices ) {
		std::cout << "Couldn't allocate host storage for cube indices" << std::endl;
		delete[] thread_write_offsets;
		delete[] vertices_per_cube;
		exit( -1 );
	}

	// Populate the thread write offsets
	std::cout << "-- populating thread write offsets" << std::endl;
	int current_offset = 0;
	int output_index = 0;
	for ( int cube_index = 0; cube_index < num_cubes; cube_index++ ) {

		if ( vertices_per_cube[ cube_index ] > 0 ) {
			// Store current offset
			thread_write_offsets[ output_index ] = current_offset;

			// And the cube index
			cube_indices[ output_index ] = cube_index;

			// Update the count
			current_offset += vertices_per_cube[ cube_index ];

			// And the output index
			output_index ++;
		}
	}
	delete[] vertices_per_cube;

	/* **************************************************************************************************************
	 * *                                                                                                            *
	 * *  Get the vertices corresponding to each cube                                                               *
	 * *                                                                                                            *
	 * **************************************************************************************************************/
	// Allocate device vertex storage
	std::cout << "   -- allocating storage for " << num_vertices << " vertices on device" << std::endl;
	cudaSafeAlloc( (void **) &d_mesh_vertices, num_vertices * sizeof( float3 ), "d_mesh_vertices" );

	// Finally generate the vertex data
	std::cout << "-- generating vertices" << std::endl;
	launch_generate_vertices(	volume, 					// in
								num_occupied_cubes, 		// in
								cube_indices, 				// in
								thread_write_offsets, 		// in
								num_vertices,	 			// in
								d_mesh_vertices,			// out
								d_m_dash);					// out

	// And tidy up
	delete [] thread_write_offsets;
	delete [] cube_indices;
	std::cout << "-- done" << std::endl;
}


/**
 * Implement marching cubes on the specified GPUTSDF
 * @param volume The TSDF
 * @param vertices A vector of vertices
 * @param triangles A vector of Triangles
 */
void extract_surface( const TSDFVolume * volume, std::vector<float3>& vertices, std::vector<int3>& triangles) {

	// Allocate m_dash (voxel use flags)
	dim3 grid_size = volume->size();
	int num_voxels = grid_size.x * grid_size.y  * grid_size.z ;
	uint8_t * d_m_dash;
	cudaSafeAlloc( (void **) &d_m_dash,  num_voxels * sizeof( uint8_t ), "d_m_dash" );
	// Zero m_dash
	cudaMemset( d_m_dash, 0, num_voxels * sizeof( uint8_t ) );

	float3 	* d_mesh_vertices;
	int 	num_vertices;
	extract_surface_ms( volume, num_vertices, d_mesh_vertices, d_m_dash);

	// Don't care for these
	cudaSafeFree( d_m_dash, "d_m_dash" );


	// Copy vertices back off device
	float3 * h_mesh_vertices = new float3[ num_vertices ];
	if( ! h_mesh_vertices ) {
		std::cout << " Couldn't allocate storage on host for mesh vertices" << std::endl;
		exit( -1 );
	}
	cudaSafeCopyToHost( h_mesh_vertices, d_mesh_vertices, num_vertices * sizeof( float3 ), "h_mesh_vertices");

	// Don't need device vertices now
	cudaSafeFree( d_mesh_vertices, "d_mesh_vertices" );

	// Convert to vector or vertices and triangles
	for ( int i = 0; i < num_vertices; i++ ) {
		vertices.push_back( h_mesh_vertices[i] );
		if ( i % 3 == 0 ) {
			triangles.push_back( int3{ i, i + 2, i + 1});
		}
	}

	// Delete host vertices
	delete[] h_mesh_vertices;
}
