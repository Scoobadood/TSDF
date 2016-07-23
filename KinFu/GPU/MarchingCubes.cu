#include "MarchingCubes.hpp"
#include "cu_common.hpp"
#include "TSDF_utilities.hpp"

#include "math_constants.h"

#include <Eigen/Dense>


#include "MC_edge_table.cu"

#include "MC_triangle_table.cu"


/**
 * Compute the intersection of an edge
 */
__device__
float3 compute_intersection_for_edge( int edge_index,
                                      const float voxel_values[8],
                                      const float3& voxel_centre,
                                      const float3& voxel_size) {

	// Get indices of vertices bounding this edge
	int v1_index = EDGE_VERTICES[edge_index][0];
	int v2_index = EDGE_VERTICES[edge_index][1];

	float numerator = 0 - voxel_values[v1_index];
	float denom = voxel_values[v2_index] - voxel_values[v1_index];
	float ratio = numerator / denom;

	float3 intersection = voxel_centre;
	// This code critically dependent on the edge ordering in MC_edge_table
	switch ( edge_index ) {
	// Parallel to X axis
	case 4:
		intersection.x += voxel_size.x * ratio;
		break;

	case 5:
		intersection.x += voxel_size.x * ratio;
		intersection.z += voxel_size.z;
		break;

	case 6:
		intersection.x += voxel_size.x * ratio;
		intersection.y += voxel_size.y;
		intersection.z += voxel_size.z;
		break;

	case 7:
		intersection.x += voxel_size.x * ratio;
		intersection.y += voxel_size.y;
		break;

	// Parallel to Y axis
	case 1:
		intersection.y += voxel_size.y * ratio;
		intersection.z += voxel_size.z;
		break;
	case 3:
		intersection.y += voxel_size.y * ratio;
		break;
	case 9:
		intersection.x += voxel_size.x;
		intersection.y += voxel_size.y * ratio;
		intersection.z += voxel_size.z;
		break;
	case 11:
		intersection.x += voxel_size.x;
		intersection.y += voxel_size.y * ratio;
		break;

	// Parallel to Z axis
	case 0:
		intersection.z += voxel_size.z * ratio;
		break;
	case 2:
		intersection.y += voxel_size.y;
		intersection.z += voxel_size.z * ratio;
		break;
	case 8:
		intersection.x += voxel_size.x;
		intersection.z += voxel_size.z * ratio;
		break;
	case 10:
		intersection.x += voxel_size.x;
		intersection.y += voxel_size.y;
		intersection.z += voxel_size.z * ratio;
		break;
	}

	return intersection;
}


/**
 * @param cube_index the value descrbining which cube this is
 * @param voxel_values The distance values from the TSDF corresponding to the 8 voxels forming this cube
 * @param voxel_centre The coordinates of the centre of this voxel
 * @param voxel_size The physical size of a voxel
 * @param intersects Populated by this function, the point on wach of the 12 edges where an intersection occurs
 * There are a maximum of 12. Non-intersected edges are skipped that is, if only edge 12 is intersected then intersects[11]
 * will have a value the other values will be NaN
 * @return The number of intersects found
 */
__device__
int compute_edge_intersects( uint8_t cube_index,
                             const float voxel_values[8],
                             const float3& voxel_centre,
                             const float3& voxel_size,
                             float3 intersects[12]) {
	// Get the edges impacted
	int num_edges_impacted = 0;
	if ( ( cube_index != 0x00) && ( cube_index != 0xFF ) ) {
		uint16_t intersected_edge_flags = EDGE_TABLE[cube_index];
		uint16_t mask = 0x01;
		for ( int i = 0; i < 12; i++ ) {
			if ( ( intersected_edge_flags & mask ) > 0 ) {

				intersects[i] = compute_intersection_for_edge( i, voxel_values, voxel_centre, voxel_size);

				num_edges_impacted++;
			} else {
				intersects[i].x = CUDART_NAN_F;
				intersects[i].y = CUDART_NAN_F;
				intersects[i].z = CUDART_NAN_F;
			}
			mask = mask << 1;
		}
	}
	return num_edges_impacted;
}



/**
 * @param values An array of eight values from the TSDF indexed per the edge_table include file
 * return an 8 bit value representing the cube type (where a bit is set if the value in tha location
 * is negative i.e. behind the surface otherwise it's clear
 */
__device__
uint8_t cube_type_for_values( const float values[8] ) {
	uint8_t mask = 0x01;
	uint8_t cube_type = 0x00;
	for ( int i = 0; i < 8; i++ ) {
		if (values[i] < 0) {
			cube_type = cube_type | mask;
		}
		mask = mask << 1;
	}
	return cube_type;
}

/**
 * Compute the triangles for two planes of data
 * @param tsdf_values_layer_1 The first layer of values
 * @param tsdf_values_layer_2 The second layer of values
 * @param voxel_grid_size Dimensions of the grid
 * @param voxel_space_size How big the grid is physically
 * @param voxel_size THe size of an individual voxel
 * @param offset The location of the origin of the voxel space in world coords
 * @param vz The index of the plane being considered
 * @param vertices An array of 12 vertices per cube
 * @param triangels An array of 5 triangles per cube
 */
__global__
void mc_kernel( const float * tsdf_values_layer_1,
                const float * tsdf_values_layer_2,
                const dim3    voxel_grid_size,
                const float3  voxel_space_size,
                const float3  voxel_size,
                const float3  offset,
                int   vz,

                // Output variables
                float3 *vertices,
                int3   *triangles ) {


	// Extract the voxel X and Y coordinates which describe the position in the layers
	// We use layer1 = z0, layer2 = z1
	int vx = threadIdx.x + (blockIdx.x * blockDim.x);
	int vy = threadIdx.y + (blockIdx.y * blockDim.y);

	// If this thread is in range (we only go up to penultimate X and Y values)
	if ( ( vx < voxel_grid_size.x - 1 ) && ( vy < voxel_grid_size.y - 1 ) ) {

		// Compute index of the voxel to address (used to index voxel data)
		int voxel_index =  (voxel_grid_size.x * vy) + vx;

		// Compute cube index (ised to index output tris and verts)
		int cube_index =      ((voxel_grid_size.x - 1) * vy) + vx;
		int vertex_index   =  cube_index * 12;
		int triangle_index =  cube_index *  5;


		// Compute the centre of the voxel at (0,0,0)
		float3 centre_of_voxel{
			((vx + 0.5f) * voxel_size.x ) + offset.x,
			((vy + 0.5f) * voxel_size.y ) + offset.y,
			((vz + 0.5f) * voxel_size.z ) + offset.z };

		// Load voxel values for the cube
		float voxel_values[8] = {
			tsdf_values_layer_1[voxel_index],							//	vx,   vy,   vz
			tsdf_values_layer_2[voxel_index],							//	vx,   vy,   vz+1
			tsdf_values_layer_2[voxel_index + voxel_grid_size.x],		//	vx,   vy+1, vz+1
			tsdf_values_layer_1[voxel_index + voxel_grid_size.x],		//	vx,   vy+1, vz
			tsdf_values_layer_1[voxel_index + 1],						//	vx+1, vy,	vz
			tsdf_values_layer_2[voxel_index + 1],						//	vx+1, vy, 	vz+1
			tsdf_values_layer_2[voxel_index + voxel_grid_size.x + 1],	//	vx+1, vy+1, vz+1
			tsdf_values_layer_1[voxel_index + voxel_grid_size.x + 1],	//	vx+1, vy+1, vz
		};

		// Compute the cube type
		uint8_t cube_type = cube_type_for_values( voxel_values );

		// If it's a non-trivial cube_type, process it
		if ( ( cube_type != 0 ) && ( cube_type != 0xFF ) ) {

			// Compute intersects (up to 12 per cube)
			float3 	intersects[12];
			compute_edge_intersects( cube_type, voxel_values, centre_of_voxel, voxel_size, intersects );

			// Copy these back into the return vaue array at the appropriate point for this thread
			for ( int i = 0; i < 12; i++ ) {
				vertices[vertex_index + i] = intersects[i];
			}

			// These intersects form triangles in line with the MC triangle table
			// We compute all five triangles because we're writing to a fixed size array
			// and we need to ensure that every thread knows where to write.
			int i = 0;
			for ( int t = 0; t < 5; t++ ) {
				triangles[triangle_index + t].x = TRIANGLE_TABLE[cube_type][i++];
				triangles[triangle_index + t].y = TRIANGLE_TABLE[cube_type][i++];
				triangles[triangle_index + t].z = TRIANGLE_TABLE[cube_type][i++];
			}
		} else {
			// Set all triangle to have -ve indices
			for ( int t = 0; t < 5; t++ ) {
				triangles[triangle_index + t].x = -1;
				triangles[triangle_index + t].y = -1;
				triangles[triangle_index + t].z = -1;
			}
		}
	}
}


/**
 * Transfer variables into a device usable form prior to invoking the kernel
 */
__host__
void prep_for_kernel( const phd::TSDFVolume 	*volume,
                      dim3& 					voxel_grid_size,
                      float3& 					voxel_space_size,
                      float3& 					voxel_size,
                      float3& 					offset) {

	using namespace Eigen;

	Vector3i vsize = volume->size();
	voxel_grid_size = dim3{
		static_cast<unsigned int> (vsize[0]),
		static_cast<unsigned int> (vsize[1]),
		static_cast<unsigned int> (vsize[2])
	};

	Vector3f vpsize = volume->physical_size();
	voxel_space_size = float3{
		vpsize[0],
		vpsize[1],
		vpsize[2]
	};

	Vector3f voffset = volume->offset();
	offset = float3 {
		voffset[0],
		voffset[1],
		voffset[2]
	};

	voxel_size = float3 {
		vpsize[0] / vsize[0],
		vpsize[1] / vsize[1],
		vpsize[2] / vsize[2],
	};
}


/**
 * Process kernel output into vector of vertices and triagles
 * @param volume THe TSDF Volume
 * @param h_vertices The host array of vertices populateed by the kernel
 * @param h_triangles Host array of triangles
 * @param vertices A vectore of vertcies to be populated by this function
 * @param triangles a vector of triangles to be populated by this function
 */
__host__
void process_kernel_output( const phd::TSDFVolume * volume,
                            const float3          * h_vertices,
                            const int3            * h_triangles,
                            std::vector<float3>&    vertices,
                            std::vector<int3>&      triangles) {
	using namespace Eigen;

	// Assemble triangles into given vectors
	Vector3i vsize = volume->size();

	// For all but last row of voxels
	int cube_index = 0;
	for ( int y = 0; y < vsize.y() - 1; y++ ) {

		// For all but last column of voxels
		for ( int x = 0; x < vsize.x() - 1; x++ ) {

			// get pointers to vertices and triangles for this voxel
			const float3 * verts = h_vertices  + ( cube_index * 12 );
			const int3   * tris  = h_triangles + ( cube_index * 5  );

			// Iterate until we have 5 triangles or there are none left
			int tri_index = 0;
			while ( ( tri_index < 5) && ( tris[tri_index].x != -1 ) ) {

				// Get the raw vertex IDs
				int tri_vertices[3];
				tri_vertices[0] = tris[tri_index].x;
				tri_vertices[1] = tris[tri_index].y;
				tri_vertices[2] = tris[tri_index].z;


				// Remap triangle vertex indices to global indices
				int remap[] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
				for ( int tv = 0; tv < 3; tv++ ) {

					int vid = tri_vertices[tv];
					int vertexid = remap[ vid ];
					if ( vertexid == -1 ) {
						// This vertex hasnt been remapped (ie stored) yet
						vertices.push_back( verts[ vid ] );

						// Get the new ID
						vertexid = vertices.size() - 1;

						// And enter in remap table
						remap[ vid ] = vertexid;
					}
					tri_vertices[tv] = vertexid;
				}

				// Store the triangle
				int3 triangle{tri_vertices[0], tri_vertices[1], tri_vertices[2]};
				triangles.push_back( triangle );

				tri_index++;
			}
			cube_index++;
		}
	}
}

/**
 * Implement marching cubes on the specified GPUTSDF
 * @param volume The TSDF
 * @param vertices A vector of vertices
 * @param triangles A vector of Triangles
 */
__host__
void extract_surface( const phd::TSDFVolume * volume, std::vector<float3>& vertices, std::vector<int3>& triangles) {
	using namespace Eigen;

	// Convert input parms into kernel parameters
	dim3		voxel_grid_size;
	float3		voxel_space_size;
	float3		offset;
	float3		voxel_size;
	prep_for_kernel( volume, voxel_grid_size, voxel_space_size, voxel_size, offset );


	// Allocate storage on device and locally
	//	Fail if not possible
	size_t num_cubes_per_layer = (voxel_grid_size.x - 1) * (voxel_grid_size.y - 1);

	// Device vertices
	float3 * d_vertices;
	size_t num_vertices =  num_cubes_per_layer * 12;
	cudaError_t err = cudaMalloc( &d_vertices, num_vertices * sizeof( float3 ) );
	if ( err != cudaSuccess ) {
		std::cout << "Couldn't allocate device memory for vertices" << std::endl;
		throw std::bad_alloc( );
	}

	// Device triangles
	int3   * d_triangles;
	size_t num_triangles  = num_cubes_per_layer * 5;
	err = cudaMalloc( &d_triangles, num_triangles * sizeof( int3 ) );
	if ( err != cudaSuccess ) {
		cudaFree( d_vertices );
		std::cout << "Couldn't allocate device memory for triangles" << std::endl;
		throw std::bad_alloc( );
	}

	// Host vertices
	float3 * h_vertices = new float3[ num_vertices ];
	if ( !h_vertices ) {
		cudaFree( d_vertices);
		cudaFree( d_triangles);
		std::cout << "Couldn't allocate host memory for vertices" << std::endl;
		throw std::bad_alloc( );
	}

	// Host triangles
	int3 *h_triangles = new int3[  num_triangles ];
	if ( !h_triangles ) {
		delete [] h_vertices;
		cudaFree( d_vertices);
		cudaFree( d_triangles);
		std::cout << "Couldn't allocate host memory for triangles" << std::endl;
		throw std::bad_alloc( );
	}

	// Now iterate over each slice
	const float * volume_distance_data = volume->distance_data( );
	size_t layer_size = voxel_grid_size.x * voxel_grid_size.y;
	for ( int vz = 0; vz < voxel_grid_size.z-1; vz++ ) {

		// Set up for layer
		const float * layer1_data = &(volume_distance_data[vz * layer_size] );
		const float * layer2_data = &(volume_distance_data[(vz + 1) * layer_size] );


		// invoke the kernel
		dim3 block( 32, 32, 1 );
		dim3 grid ( divUp( voxel_grid_size.x, block.x ), divUp( voxel_grid_size.y, block.y ), 1 );
		mc_kernel <<< grid, block >>>( layer1_data, layer2_data,
		                               voxel_grid_size, voxel_space_size,
		                               voxel_size, offset,
		                               vz, d_vertices, d_triangles );

		err = cudaDeviceSynchronize( );
		check_cuda_error( "device synchronize failed " , err);

		// Copy the device vertices and triangles back to host
		err = cudaMemcpy( h_vertices, d_vertices, num_vertices * sizeof( float3 ), cudaMemcpyDeviceToHost);
		check_cuda_error( "Copy of vertex data fom device failed " , err);

		err = cudaMemcpy( h_triangles, d_triangles, num_triangles * sizeof( int3 ), cudaMemcpyDeviceToHost);
		check_cuda_error( "Copy of triangle data from device failed " , err);

		// All through all the triangles and vertices and add them to master lists
		process_kernel_output( volume, h_vertices, h_triangles, vertices, triangles);

	}

	// Free memory and done
	cudaFree( d_vertices);
	cudaFree( d_triangles);
	delete [] h_vertices;
	delete [] h_triangles;
}
