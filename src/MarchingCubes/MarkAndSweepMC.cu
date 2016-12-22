#include "../include/TSDFVolume.hpp"
#include "../include/cuda_utilities.hpp"
#include "../include/TSDF_utilities.hpp"

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

__device__
void voxel_indices_for_cube_index( const int cube_index, const int grid_size_x, const int grid_size_y, int voxel_indices[8] ) {
	int cube_xy_slab_size = ( ( grid_size_x - 1) * (grid_size_y - 1));

	int cube_z = cube_index / cube_xy_slab_size;
	int cube_y = ( cube_index - (cube_z * cube_xy_slab_size) ) / (grid_size_x - 1);
	int cube_x = ( cube_index - (cube_z * cube_xy_slab_size) ) % (grid_size_x - 1);

	int front_left_bottom_voxel_index = ( cube_z) * ( grid_size_x * grid_size_y) + (cube_y * grid_size_x) + cube_x;

	int dx = 1;
	int dy = grid_size_x;
	int dz = dy * grid_size_y;

	// Compute voxel indices for other voxels
	voxel_indices[0] = front_left_bottom_voxel_index + 		dz;		// back left bottom
	voxel_indices[1] = front_left_bottom_voxel_index + dx + dz;		// back rght bottom
	voxel_indices[2] = front_left_bottom_voxel_index + dx;			// frnt rght bottom
	voxel_indices[3] = front_left_bottom_voxel_index;				// frnt left bottom
	voxel_indices[4] = voxel_indices[0] + dy;						// back left top
	voxel_indices[5] = voxel_indices[1] + dy;						// back rght top
	voxel_indices[6] = voxel_indices[2] + dy;						// frnt rght top
	voxel_indices[7] = voxel_indices[3] + dy;						// frnt left top
}

/**
 * For a given cube in the voxel grid, compute the cube_index
 * @param voxel_index_0 (in) The voxel index of the root voxel of the cube (vertex 0 above)
 * @param dy (in) The number by which to increment a voxel index to get the voxel above it
 * @param dz (in) The number by which to increment a voxel index to get the voxel behind it
 * @param d_distance_data (in) The array of depths defining the iso surface
 * @return The cube index, 0-255
 */
__device__
uint8_t calculate_cube_type( const int 		voxel_indices[8],
                             const float 	* const distance_data ) {
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

/**
 * For a voxel in the voxel grid, compute the number of vertices the cbe rooted at that point contributes to the iso surfac mesh
 * @param grid_size (in) in voxels
 * @param distance_data (in) The distance data from the TSDF indexed [z][y][x]
 * @param d_vertices_per_cube (in) Pre-allocated Array of the number of vertices contributed by the indexed cube (may be 0) 5 triangles yields 15 vertices regardless of the reuse of vertices
 */
__global__
void get_cube_contribution( const dim3 			grid_size, 				//	in
                            const int 			max_cubes,				//	in
                            const float * const distance_data, 			//	in
                            uint8_t * const 	d_vertices_per_cube) 	//	out
{
	int cube_index = blockIdx.x * blockDim.x  + threadIdx.x;

	if ( cube_index < max_cubes) {

		int voxel_indices[8];
		voxel_indices_for_cube_index( cube_index, grid_size.x, grid_size.y, voxel_indices);

		int cube_type = calculate_cube_type( voxel_indices, distance_data );

		// Lookup the num triangles from corner_flags
		uint8_t num_verts_in_cube = VERTICES_FOR_CUBE_TYPE[ cube_type ];

		// Update the output arrays
		d_vertices_per_cube[ cube_index ] = num_verts_in_cube;
	}
}

/**
 * Launch classify voxel kernel
 * @param volume The TSDFVolume. Not modified by this method. Provides the raw data
 * @param vertices_per_cube Populated by this method, a list of vertices for each cube in the voxel space. Every cube is represented. Some cubes may contribute zero vertices.
 * @param num_occupied_cubes The number of cubes in the space that contribute to the mesh. Populated by this call
 */
__host__
void launch_get_cube_contribution(	const TSDFVolume * const 	volume,
                                    int 						max_cubes,
                                    uint8_t *& 					vertices_per_cube,
                                    int& 						num_occupied_cubes,
                                    int& 						num_vertices ) {

	dim3 grid_size = volume->size( );

	// Allocate storage for the vertex per cube count
	uint8_t * d_vertices_per_cube;
	cudaError_t err = cudaMalloc( &d_vertices_per_cube, max_cubes * sizeof( uint8_t ));
	check_cuda_error( "Couldn't allocate space for vertex count", err );


	// invoke the kernel
	dim3 block( 1024 );
	dim3 grid( divUp( max_cubes, block.x ) );
	get_cube_contribution <<< grid, block >>>( grid_size, max_cubes, volume->distance_data(), d_vertices_per_cube );
	cudaDeviceSynchronize( );
	err = cudaGetLastError( );
	check_cuda_error( "Classify voxels failed", err );


	// Copy vertex count back to host
	cudaMemcpy( vertices_per_cube, d_vertices_per_cube, max_cubes * sizeof( uint8_t), cudaMemcpyDeviceToHost );
	err = cudaGetLastError( );
	check_cuda_error( "Copy of vertex count data to host failed", err );

	err = cudaFree( d_vertices_per_cube);
	check_cuda_error( "Problem freeing device vertices per cube array", err );

	// compute occupied cubes and num_vertices
	num_occupied_cubes = 0;
	num_vertices = 0;
	for ( int i = 0; i < max_cubes; i++ ) {
		num_vertices += vertices_per_cube[i];
		if ( vertices_per_cube[i] > 0 ) {
			num_occupied_cubes++;
		}
	}
}

/* ******************************************************************************** */

/**
 * Generate vertices for each cube.
 * @param grid_size 		The size of the grid. Used to translate a cube index into a voxel index
 * @param voxel_centres 	Used to warp the grid
 * @param distance_data 	The Isosurface data
 * @param cube_indices 		Array of indices of the cubes which form the surface
 * @param num_cubes 		Number of entries to process
 * @param vertex_offstes 	Where in vertices a cube should writre its output
 * @param vertices 			The output data
 * @param vertex_use_count	The number of times each voxel is used in this mesh
 * @param mesh_voxel_entries Two entries per mesh vertex, for the voxels that form either end of the edge the vertex lies on
 */
__global__
void generate_vertices( const dim3 				grid_size, 				// in
                        const float3 * const 	voxel_centres,			// in (device)
                        const float * const 	distance_data,			// in (device)
                        const int * const		cube_indices,			// in (device)
                        const int 				num_cubes,				// in
                        const int * const 		thread_write_offsets,	// in
                        float3 					* vertices,				// out (preallocated device)
                        uint8_t					* voxel_use_count,		// out (preallocated device)
                        int 					* mesh_voxel_indices	// out (preallocated device)
                        ) {

	int data_index = blockIdx.x * blockDim.x + threadIdx.x;

	if ( data_index < num_cubes ) {

		// Pick the cube index to address
		int cube_index = cube_indices[ data_index ];

		// Now get a voxel index for the base corner of the cube
		int voxel_indices[8];
		voxel_indices_for_cube_index( cube_index, grid_size.x, grid_size.y, voxel_indices );

		// Get weights of each adjacent vertex
		float w0 = distance_data[ voxel_indices[0] ];
		float w1 = distance_data[ voxel_indices[1] ];
		float w2 = distance_data[ voxel_indices[2] ];
		float w3 = distance_data[ voxel_indices[3] ];
		float w4 = distance_data[ voxel_indices[4] ];
		float w5 = distance_data[ voxel_indices[5] ];
		float w6 = distance_data[ voxel_indices[6] ];
		float w7 = distance_data[ voxel_indices[7] ];

		// Get the vertex coordinates
		float3 v0 = voxel_centres[ voxel_indices[0] ];
		float3 v1 = voxel_centres[ voxel_indices[1] ];
		float3 v2 = voxel_centres[ voxel_indices[2] ];
		float3 v3 = voxel_centres[ voxel_indices[3] ];
		float3 v4 = voxel_centres[ voxel_indices[4] ];
		float3 v5 = voxel_centres[ voxel_indices[5] ];
		float3 v6 = voxel_centres[ voxel_indices[6] ];
		float3 v7 = voxel_centres[ voxel_indices[7] ];

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
		int output_index = thread_write_offsets[ data_index ];
		int cube_type = calculate_cube_type( voxel_indices, distance_data );
		while ( (edge_index = TRIANGLE_TABLE[cube_type][i]) != -1 ) {
			vertices[ output_index ] = vertex[edge_index];

			//	Get the indices of the vertex end points
			int voxel_index_1 = voxel_indices[ EDGE_VERTICES[edge_index][0] ];
			int voxel_index_2 = voxel_indices[ EDGE_VERTICES[edge_index][1] ];

			// Store the voxel indices for this mesh vertex
			mesh_voxel_indices[ output_index * 2 ] = voxel_index_1;
			mesh_voxel_indices[ output_index * 2 + 1 ] = voxel_index_2;

			// Update the count for number of times used for each voxel index
			atomicIncUint8( &(voxel_use_count[ voxel_index_1 ] ) );
			atomicIncUint8( &(voxel_use_count[ voxel_index_2 ] ) );

			output_index ++;
			i++;
		}
	}
}


/**
 * Generate vertices for each cube. vertices are generated into the vertices array (space is allocated in this method)
 * Triangles of the mesh are implicitly formed by every 3 vertices
 * @param volume The TSDF volume source of the data
 * @param cube_indices The indices of the cubes which contribute to the surface. Note that cube indices run to grid x,y and z LESS 1
 * @param thread_write_offsets The offset (per cube) into vertices where the vertices for that cube should be stored. Offsets are in vertex count rather than bytes
 * @param num_occiupied_cubes the number of entries to process
 * @param vertices (out) aloocated externally and populated by this method.
 */
void launch_generate_vertices(	const TSDFVolume	* volume,
                                const int 			* const cube_indices,
                                const int 			* const thread_write_offsets,
                                int 				num_cubes,
                                int 				num_vertices,
                                float3				* vertices,
                                int *&				d_mesh_voxel_indices ) {

	// Allocate device vertex storage
	float3 * d_vertices;
	cudaError_t err = cudaMalloc( &d_vertices, num_vertices * sizeof( float3 ));
	check_cuda_error( "Couldn't allocate device memory for vertex storage", err);

	// Allocate voxel use count on device
	uint8_t * d_voxel_use_count;
	dim3 volume_size = volume->size();
	int num_voxels = volume_size.x * volume_size.y * volume_size.z;
	err = cudaMalloc( &d_voxel_use_count, num_voxels * sizeof( uint8_t ));
	check_cuda_error( "Couldn't allocate device memory for voxel usage count", err);
	err = cudaMemset( d_voxel_use_count, num_voxels * sizeof( uint8_t ), 0 );
	check_cuda_error( "Couldn't zero device memory for voxel usage count", err);

	// Allocate mesh voxel index memory
	err = cudaMalloc( &d_mesh_voxel_indices, num_vertices * 2 * sizeof( int ));
	check_cuda_error( "Couldn't allocate device memory for mesh voxel indices count", err);

	// Copy cube indices to device
	int * d_cube_indices;
	err = cudaMalloc( &d_cube_indices, num_cubes * sizeof( int ));
	check_cuda_error( "Couldn't allocate device memory for cube indices", err);
	err = cudaMemcpy( d_cube_indices, cube_indices, num_cubes * sizeof( int ), cudaMemcpyHostToDevice);
	check_cuda_error( "Couldn't copy cube indicesto device", err);

	// Copy vertex offsets to device
	int * d_thread_write_offsets;
	err = cudaMalloc( &d_thread_write_offsets, num_cubes * sizeof( int ));
	check_cuda_error( "Couldn't allocate device memory for vertex offsets", err);
	err = cudaMemcpy( d_thread_write_offsets, thread_write_offsets, num_cubes * sizeof( int ), cudaMemcpyHostToDevice);
	check_cuda_error( "Couldn't copy vertex offstes to device", err);

	dim3 block( 1024 );
	dim3 grid( divUp( num_cubes, block.x ));
	generate_vertices <<< grid, block >>>(	volume->size(),
	                                        (float3 *) volume->translation_data(),
	                                        volume->distance_data(),
	                                        d_cube_indices,
	                                        num_cubes,
	                                        d_thread_write_offsets,
	                                        d_vertices,
	                                        d_voxel_use_count,
	                                        d_mesh_voxel_indices);
	cudaDeviceSynchronize();
	err = cudaGetLastError( );
	check_cuda_error( "Generate vertices kernel failed", err );

	// Copy device storage back to host
	err = cudaMemcpy( vertices, d_vertices, num_vertices * sizeof( float3 ), cudaMemcpyDeviceToHost);
	char s [ 500];
	sprintf( s, "copy %d verts from device to host failed", num_vertices );
	check_cuda_error( s, err );

	err = cudaFree( d_vertices );
	check_cuda_error( "Failed to free vertex memory from device", err );
	err = cudaFree( d_cube_indices );
	check_cuda_error( "Failed to free cube index memory from device", err );
	err = cudaFree( d_thread_write_offsets );
	check_cuda_error( "Failed to free thread write offsets memory from device", err );

	// For now, delete these data stores
	err = cudaFree( d_voxel_use_count );
	check_cuda_error( "Failed to free voxel_use_count memory from device", err );
}

/**
 * Extract the surface from the given TSDF using mark and sweep MC
 * @param volume The TSDF
 * @param vertices A ref to a float3 pointer. This is allocated by the cuntion and on return points to the vertex data, Vertex data is arranged in triangles oriented anti-clockwise.
 * @param num_vertices populated by the method contains the total number of vertices found
 */
void extract_surface_ms( const TSDFVolume * const 	volume, 	// in
                         float3 *& 					vertices,
                         int& 						num_vertices,
                         int*& 						voxel_indices ) {
	std::cout << "Extracting surface" << std::endl;

	// Each cube is a block of 8 adjacent voxels.
	dim3 grid_size = volume->size();
	int max_cubes = (grid_size.x - 1 ) * ( grid_size.y - 1) * ( grid_size.z - 1 );

	/* **************************************************************************************************************
	 * *                                                                                                            *
	 * *  Get each cubes contribution to vertices                                                                   *
	 * *                                                                                                            *
	 * **************************************************************************************************************/

	// Allocate host storage for number of vertices per cube
	uint8_t *vertices_per_cube = vertices_per_cube = new uint8_t[ max_cubes ];
	if ( !vertices_per_cube) {
		std::cout << "Couldn't allocate storage for verts per cube on host" << std::endl;
		exit( -1 );
	}

	// Launch the kernel to populate this
	int num_occupied_cubes = 0;
	launch_get_cube_contribution( volume, max_cubes, vertices_per_cube, num_occupied_cubes, num_vertices );
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
	std::cout << "Allocating storage for " << num_occupied_cubes << " offsets" << std::endl;
	int * thread_write_offsets = new int[ num_occupied_cubes ];
	if ( !thread_write_offsets ) {
		std::cout << "Couldn't allocate host storage for thread write offsets" << std::endl;
		delete[] vertices_per_cube;
		delete[] vertices;
		exit( -1 );
	}

	// Allocate storage for cube indices
	std::cout << "Allocating storage for " << num_occupied_cubes << " cube indices" << std::endl;
	int * cube_indices = new int[ num_occupied_cubes ];
	if ( !cube_indices ) {
		std::cout << "Couldn't allocate host storage for thread write offsets" << std::endl;
		delete[] thread_write_offsets;
		delete[] vertices_per_cube;
		delete[] vertices;
		exit( -1 );
	}

	// Populate the thread write offstes
	std::cout << "Populating thread write offsets" << std::endl;
	int current_offset = 0;
	int output_index = 0;
	for ( int cube_index = 0; cube_index < max_cubes; cube_index++ ) {

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
	// Allocate storage to hold enough vertices
	std::cout << "Allocating storage for " << num_vertices << " vertices" << std::endl;
	vertices = new float3[ num_vertices ];
	if ( !vertices ) {
		std::cout << "Couldn't allocate host storage for vertices" << std::endl;
		delete[] vertices_per_cube;
		exit( -1 );
	}

	// Finally generate the vertex data
	std::cout << "Generating vertices" << std::endl;
	launch_generate_vertices( volume, cube_indices, thread_write_offsets, num_occupied_cubes, num_vertices, vertices, voxel_indices);

	// And tidy up
	delete [] thread_write_offsets;
	delete [] cube_indices;
}


/**
 * Implement marching cubes on the specified GPUTSDF
 * @param volume The TSDF
 * @param vertices A vector of vertices
 * @param triangles A vector of Triangles
 */
void extract_surface( const TSDFVolume * volume, std::vector<float3>& vertices, std::vector<int3>& triangles) {
	float3 	* f3_vertices;
	int 	* voxel_indices;
	int 	num_vertices;
	extract_surface_ms( volume, f3_vertices, num_vertices, voxel_indices );

	for ( int i = 0; i < num_vertices; i++ ) {
		vertices.push_back( f3_vertices[i] );
		if ( i % 3 == 0 ) {
			triangles.push_back( int3{ i, i + 2, i + 1});
		}
	}

	// Delete f3 vertices
	delete[] f3_vertices;
	cudaError_t err = cudaFree( voxel_indices );
	check_cuda_error( "Failed to delete voxel indices from device", err );
}
