#include "../include/TSDFVolume.hpp"
#include "../include/cuda_utilities.hpp"
#include "../include/TSDF_utilities.hpp"

#include "MC_edge_table.cu"
#include "MC_triangle_table.cu"

/*
 *            3--------(7)---------7
 *           /|                   /|
 *          / |                  / |
 *         /  |                 /  |
 *       (2)  |               (A)  |
 *       /    |               /    |
 *      /    (3)             /    (B)
 *     /      |             /      |
 *    2---------(6)--------6       |
      |       |            |       |
 *    |       0------(4)---|-------4
 *    |      /             |      /
 *   (1)    /             (9)    /
 *    |    /               |    /
 *    |  (0)               |  (8)
 *    |  /                 |  /
 *    | /                  | /
 *    |/                   |/
 *    1---------(5)--------5
 */
/*
 * We first find all grid cells that contain a zero crossing based on a data-parallel prefix sum. One thread per valid grid cell is 
 * used to extract the final list of triangles. Since each valid grid cell potentially generates multiple triangles, we use atomic 
 * counters to manage the list. The resulting vertices are immediately deformed according to the current deformation field, 
 * resulting in a polygonal approximation of P.
 */



__device__
float3 interpolate( float3 v0, float3 v1, float w0, float w1 ) {

	if( ( w0 > 0 ) && ( w1 < 0 ) ) {
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
 * Convert the index of a cube into the index of the root (0) voxel vertex of that cube
 * @param cube_index the index of the cube where front, left, bottom is 0
 * @param grid_size_x Size of voxel grid (x)
 * @param grid_size_y Size of voxel grid (y)
 * @return the voxel index of the root of the cube
 */
__device__
int voxel_index_for_cube_index( const int cube_index, const int grid_size_x, const int grid_size_y ) {
	int cube_xy_slab_size = ( ( grid_size_x-1) * (grid_size_y - 1));
	int cube_z = cube_index / cube_xy_slab_size;
	int cube_y = ( cube_index - (cube_z * cube_xy_slab_size) ) / grid_size_x;
	int cube_x = ( cube_index - (cube_z * cube_xy_slab_size) ) % grid_size_x;

	int voxel_index = ( cube_z + 1) * ( grid_size_x * grid_size_y) + (cube_y * grid_size_x) + cube_x;
	return voxel_index;

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
uint8_t calculate_cube_type(	const int 				voxel_index_0, 
								const int 				dy, 
								const int 				dz, 
								const float 	* const distance_data ) {

	// Compute vertex indices for the remaining corners of the cube
	int voxel_index_1 = voxel_index_0 - dz;
	int voxel_index_2 = voxel_index_1 + dy;
	int voxel_index_3 = voxel_index_0 + dy;
	int voxel_index_4 = voxel_index_0 + 1;
	int voxel_index_5 = voxel_index_1 + 1;
	int voxel_index_6 = voxel_index_2 + 1;
	int voxel_index_7 = voxel_index_3 + 1;

	// Compute the cube_type
	uint8_t cube_type = 0;
	cube_type |= ( ( distance_data[ voxel_index_0 ] < 0 ) << 0 );
	cube_type |= ( ( distance_data[ voxel_index_1 ] < 0 ) << 1 );
	cube_type |= ( ( distance_data[ voxel_index_2 ] < 0 ) << 2 );
	cube_type |= ( ( distance_data[ voxel_index_3 ] < 0 ) << 3 );
	cube_type |= ( ( distance_data[ voxel_index_4 ] < 0 ) << 4 );
	cube_type |= ( ( distance_data[ voxel_index_5 ] < 0 ) << 5 );
	cube_type |= ( ( distance_data[ voxel_index_6 ] < 0 ) << 6 );
	cube_type |= ( ( distance_data[ voxel_index_7 ] < 0 ) << 7 );

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

	if( cube_index < max_cubes) {

		// Now get a voxel index for the base corner of the cube
		int voxel_index = voxel_index_for_cube_index( cube_index, grid_size.x, grid_size.y );

		int dy = grid_size.x;
		int dz = grid_size.x * grid_size.y;
		uint8_t cube_type = calculate_cube_type( voxel_index, dy, dz, distance_data );

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
									uint8_t *& 					vertices_per_cube,
									int& 						num_occupied_cubes, 
									int& 						num_vertices ) {

	dim3 grid_size = volume->size( );
	int max_cubes = (grid_size.x - 1) * (grid_size.y - 1) * (grid_size.z - 1);

	uint8_t * d_vertices_per_cube;
	cudaError_t err = cudaMalloc( &d_vertices_per_cube, max_cubes * sizeof( uint8_t ));
	check_cuda_error( "Couldn't allocate space for vertex count", err );



	dim3 block( 1024 );
	dim3 grid( divUp( max_cubes, block.x ) );
	get_cube_contribution<<< grid, block >>>( grid_size, max_cubes, volume->distance_data(), d_vertices_per_cube );
	cudaDeviceSynchronize( );
	err = cudaGetLastError( );
	check_cuda_error( "Classify voxels failed", err );


	// Copy vertex count back to host
	vertices_per_cube = new uint8_t[ max_cubes ];
	if( !vertices_per_cube) {
		std::cout << "Couldn't allocate storage for verts per cube on host" << std::endl;
		cudaFree( d_vertices_per_cube );
		exit( -1 );
	}

	cudaMemcpy( vertices_per_cube, d_vertices_per_cube, max_cubes * sizeof( uint8_t), cudaMemcpyDeviceToHost );
	err = cudaGetLastError( );
	check_cuda_error( "Copy of vertex count data to host failed", err );

	err = cudaFree( d_vertices_per_cube);
	check_cuda_error( "Problem freeing device vertices per cube array", err );

	// compute occupied cubes and num_vertices
	num_occupied_cubes = 0;
	num_vertices = 0;
	for( int i=0; i< max_cubes; i++ ) {
		num_vertices += vertices_per_cube[i];
		if( vertices_per_cube[i] > 0 ) {
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
 */
__global__
void generate_vertices( const dim3 				grid_size, 			// in
						const float3 * const 	voxel_centres,		// in (device)
						const float * const 	distance_data,		// in (device)
						const int * const		cube_indices,		// in (device)
						const int 				num_cubes,			// in
						const int * const 		vertex_offsets,		// in
						float3 					* vertices ) {		// out (preallocated device)

	int data_index = blockIdx.x * blockDim.x + threadIdx.x;

	if( data_index < num_cubes ) {

		// Pick the cube index to address
		int cube_index = cube_indices[ data_index ];

		// Now get a voxel index for the base corner of the cube
		int voxel_index = voxel_index_for_cube_index( cube_index, grid_size.x, grid_size.y );

		int dx = 1;
		int dy = grid_size.x;
		int dz = dy * grid_size.y;

		// Get weights of each adjacent vertex
		float w0 = distance_data[ voxel_index ];
		float w1 = distance_data[ voxel_index - dz ];
		float w2 = distance_data[ voxel_index - dz + dy ];
		float w3 = distance_data[ voxel_index      + dy ];
		float w4 = distance_data[ voxel_index           + dx ];
		float w5 = distance_data[ voxel_index - dz      + dx ];
		float w6 = distance_data[ voxel_index - dz + dy + dx ];
		float w7 = distance_data[ voxel_index      + dy + dx ];

		// Get the vertex coordinates
		float3 v0 = voxel_centres[ voxel_index ];
		float3 v1 = voxel_centres[ voxel_index - dz ];
		float3 v2 = voxel_centres[ voxel_index - dz + dy ];
		float3 v3 = voxel_centres[ voxel_index      + dy ];
		float3 v4 = voxel_centres[ voxel_index           + dx ];
		float3 v5 = voxel_centres[ voxel_index - dz      + dx ];
		float3 v6 = voxel_centres[ voxel_index - dz + dy + dx ];
		float3 v7 = voxel_centres[ voxel_index      + dy + dx ];

		// Build cube index (we need to extract w)
		int cube_type = 0;
		cube_type |= ( ( w0 < 0 ) << 0 );
		cube_type |= ( ( w1 < 0 ) << 1 );
		cube_type |= ( ( w2 < 0 ) << 2 );
		cube_type |= ( ( w3 < 0 ) << 3 );
		cube_type |= ( ( w4 < 0 ) << 4 );
		cube_type |= ( ( w5 < 0 ) << 5 );
		cube_type |= ( ( w6 < 0 ) << 6 );
		cube_type |= ( ( w7 < 0 ) << 7 );

		// Compute the edge intersections (some may not be valid)
		float3 vertex[12];
		vertex[0] = interpolate(  v1, v0, w1, w0 );
		vertex[1] = interpolate(  v1, v2, w1, w2 );
		vertex[2] = interpolate(  v2, v3, w2, w3 );
		vertex[3] = interpolate(  v0, v3, w0, w3 );

		vertex[4] = interpolate(  v0, v4, w0, w4 );
		vertex[5] = interpolate(  v1, v5, w1, w5 );
		vertex[6] = interpolate(  v2, v6, w2, w6 );
		vertex[7] = interpolate(  v3, v7, w3, w7 );

		vertex[8] = interpolate(  v5, v4, w5, w4 );
		vertex[9] = interpolate(  v5, v6, w5, w6 );
		vertex[10] = interpolate( v6, v7, w6, w7 );
		vertex[11] = interpolate( v4, v7, w4, w7 );

		// Extract the vertex offset; 
		int vertex_offset = vertex_offsets[ data_index ];

		int edge_index;
		int i = 0;
		while( (edge_index = TRIANGLE_TABLE[cube_type][i]) != -1 ) {
			vertices[ vertex_offset ] = vertex[edge_index];
			vertex_offset ++;
			i++;
		}
	}
}


/**
 * Generate vertices for each cube. vertices are generated into the vertices array (space is allocated in this method)
 * Triangles of the mesh are implicitly formed by every 3 vertices
 * @param volume The TSDF volume source of the data
 * @param cube_indices The indices of the cubes which contribute to the surface. Note that cube indices run to grid x,y and z LESS 1
 * @param vertex_offsets The offset (per cube) into vertices where the vertices for that cube should be stored. Offsets are in vertex count rather than bytes
 * @param num_occiupied_cubes the number of entries to process
 * @param vertices (out) aloocated externally and populated by this method.
 */
void launch_generate_vertices(	const TSDFVolume	* volume, 
								const int 			* cube_indices,
								const int 			* const vertex_offsets,
								int 				num_cubes,
								int 				num_vertices,
								float3				* vertices ) {

	// Allocate device vertex storage
	float3 * d_vertices;
	cudaError_t err = cudaMalloc( &d_vertices, num_vertices * sizeof( float3 ));
	check_cuda_error( "Couldn't allocate device memory for vertex storage", err);

	// Copy cbe indices to device
	int * d_cube_indices;
	err = cudaMalloc( &d_cube_indices, num_cubes * sizeof( int ));
	check_cuda_error( "Couldn't allocate device memory for cube indices", err);
	err = cudaMemcpy( d_cube_indices, cube_indices, num_cubes * sizeof( int ), cudaMemcpyHostToDevice);
	check_cuda_error( "Couldn't copy cube indicesto device", err);

	// Copy vertex offsets to device
	int * d_vertex_offsets;
	err = cudaMalloc( &d_vertex_offsets, num_cubes * sizeof( int ));
	check_cuda_error( "Couldn't allocate device memory for vertex offsets", err);
	err = cudaMemcpy( d_vertex_offsets, vertex_offsets, num_cubes * sizeof( int ), cudaMemcpyHostToDevice);
	check_cuda_error( "Couldn't copy vertex offstes to device", err);

	dim3 block( 1024 );
	dim3 grid( divUp( num_cubes, block.x ));
	generate_vertices<<< grid, block >>>( volume->size(), (float3 *) volume->translation_data(), volume->distance_data(), d_cube_indices, num_cubes, d_vertex_offsets, d_vertices );
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
	err = cudaFree( d_vertex_offsets );
	check_cuda_error( "Failed to free vertex offsets memory from device", err );
}

/**
 * Create an array of vertex offsets for parallel extraction of trinagles
 * Also creates a list of the indices of those voxels.
 * @param max_cubes		 	The total number of cubes in the voxel grid
 * @param occupied_cubes	The number of cubes that contribute vertices to the mesh
 * @param vertices_per_cube List of max_cubes ints containing the number of vertices for that voxel
 * @param vertex_offsets 	A reference to a pointer to ints that is populated by this method
 * @param voxel_indices 	A reference to a points to ints that is poplulated by this method
 * @return true if this code executeds wihtout fault or false otherwise.
 */
bool generate_vertex_offsets(	int max_cubes, 
								int occupied_cubes, 
								const uint8_t * const vertices_per_cube, 
								int *& vertex_offsets, 
								int *& cube_indices) {
	if( occupied_cubes == 0 ) {
		std::cout << "No occupied voxels. This seems wrong" << std::endl;
		return false;
	}

	// Now construct the vertex offset array
	vertex_offsets = new int[ occupied_cubes ];
	if( !vertex_offsets ) {
		std::cout << "Couldn't allocate storage for vertex offsets" << std::endl;
		return false;
	}

	cube_indices = new int[ occupied_cubes ];
	if( !cube_indices ) {
		std::cout << "Couldn't allocate storage for voxel indices" << std::endl;
		delete [] vertex_offsets;
		return false;
	}

	int current_offset = 0;
	int output_index = 0;
	for( int cube_index=0; cube_index< max_cubes; cube_index++ ) {

		if( vertices_per_cube[ cube_index ] > 0 ) {
			// Store current offset
			vertex_offsets[ output_index ] = current_offset;

			// Store the index
			cube_indices[ output_index ] = cube_index;

			// Update the count
			current_offset += vertices_per_cube[ cube_index ];

			// And the output index
			output_index ++;
		}
	} 

	return true;
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
					  	 int*& 						cube_indices ) {


	std::cout << " -- mark" << std::endl;

	// Mark
	uint8_t *vertices_per_cube;
	int num_occupied_cubes;
	
	launch_get_cube_contribution( volume, vertices_per_cube, num_occupied_cubes, num_vertices );
	std::cout << "    found " << num_occupied_cubes << " occupied cubes and " << num_vertices << " vertices" << std::endl;


	// Compact streams
	std::cout << " -- compacting " << std::endl;

	dim3 grid_size = volume->size();
	int max_cubes = (grid_size.x - 1 ) * ( grid_size.y - 1) * ( grid_size.z - 1 );
	int * vertex_offsets;
	if (! generate_vertex_offsets( max_cubes, num_occupied_cubes, vertices_per_cube, vertex_offsets, cube_indices) )  {
		exit( -1 );
	}

	// Now generate the actual vertices into our mesh
	vertices = new float3[ num_vertices ];
	if( vertices ) {
		launch_generate_vertices( volume, cube_indices, vertex_offsets, num_occupied_cubes, num_vertices, vertices);
	} else {
		std::cout << "Couldn't allocate host storage for vertices" << std::endl;
	}
}


/**
 * Implement marching cubes on the specified GPUTSDF
 * @param volume The TSDF
 * @param vertices A vector of vertices
 * @param triangles A vector of Triangles
 */
void extract_surface( const TSDFVolume * volume, std::vector<float3>& vertices, std::vector<int3>& triangles) {
	using namespace Eigen;
	std::cout << "Extracting mesh" << std::endl;

	float3 * f3_vertices;
	int * cube_indices;
	int num_vertices;
	extract_surface_ms( volume, f3_vertices, num_vertices, cube_indices );

	std::cout << "Done extract, copying to vectors" << std::endl;
	for( int i=0; i<num_vertices; i++ ) {
		vertices.push_back( f3_vertices[i] );
		if( i % 3 == 0 ) {
			triangles.push_back( int3{ i, i+2, i+1});
		}
	}
	std::cout << "Done" << std::endl;
}
