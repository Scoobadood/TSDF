#include "../include/GPUMarchingCubes.hpp"
#include "../include/SceneFlowUpdater.hpp"
#include "../include/cuda_utilities.hpp"

const float THRESHOLD = 2.0f;

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



__global__
/**
 * We have scene flow data for a number of mesh vertcies (ie points in 3-space)
 * We want to apply this scene flow update to the TSDF volume deformation by
 * updating voxels within a radius of the mesh node
 * Voxels within the neghbourhood of more than one mesh node may receive multiple
 * updates. These are weighted.
 * @param mesh_scene_flow Array of N displacements
 * @param mesh_vertices Array of N vertex coords corresponding to the displacements
 * @param num_mesh_vertices Number of elements in the first two arrays
 * @param voxel_translations The existing deformation field
 * @size The dimensions of the defrmation field
 */
void apply_scene_flow_to_tsdf_kernel(
    const float3		*mesh_scene_flow,			//	The scene flow per mesh vertex
    const float3	  	*mesh_vertices,				//	The coordinates of the mesh vertex
    int 				num_mesh_vertices,			//	Number of vertices in the mesh
    float3				*voxel_translations,		//	Deformation data for the TSDF
    dim3				size						//	Deimsnions of the TSDF in voxels
) {

	// Construct the base pointer in TSDF space from y and z
	int vy = threadIdx.y + blockIdx.y * blockDim.y;
	int vz = threadIdx.z + blockIdx.z * blockDim.z;

	// If this y/z cordinate is legitimate
	if ( vy < size.y && vz < size.z ) {

		// The next (x_size) elements from here are the x coords
		size_t base_voxel_index =  ((size.x * size.y) * vz ) + (size.x * vy);

		// Iterate across X coordinate
		size_t voxel_index = base_voxel_index;

		for ( int vx = 0; vx < size.x; vx++ ) {

			// For any vertex in the mesh which is within a given neighbourhood of this voxel centre
			// Update the voxel centre coordinates with the scene flow of that vertex
			float3 deformation{ 0.0f, 0.0f, 0.0f};
			int    num_impacting_mesh_nodes = 0;

			for ( long i = 0; i < num_mesh_vertices; i++ ) {
				// TODO: Replace this with a radial basis function for weighted deformation
				float3 vector_to_vertex = f3_sub( voxel_translations[voxel_index], mesh_vertices[i]);
				float dist_to_vertex = f3_norm( vector_to_vertex);

				if ( dist_to_vertex < THRESHOLD ) {
					deformation = f3_add( deformation, mesh_scene_flow[i]);
					num_impacting_mesh_nodes++;
				}
			}
			if ( num_impacting_mesh_nodes > 0 ) {
				deformation = f3_mul_scalar( 1.0f / num_impacting_mesh_nodes, deformation );
				voxel_translations[voxel_index]  = f3_add( voxel_translations[voxel_index],  deformation );
			}

			voxel_index++;
		}
	}
}


/**
 * The mesh scene flow kernel extracts the scene flow value for each vertex in the input mesh and stores
 * it in d_meshscene_flow
 */
__global__
void mesh_scene_flow_kernel(
    float3 * mesh_vertices, 		// Input mesh (device memory)
    uint32_t num_vertices,
    float3 * scene_flow, 			//	Input raw scene flow for whole image (device memory)
    uint32_t sf_width, 				// 	Dimesnions of scene flow
    uint32_t sf_height,
    Mat44 	 inv_pose,				// Camera data
    Mat33	 k,
    float3 * mesh_scene_flow	 	// Output of scene flow for each point in the input mesh
) {

	// Vertex index -
	int vertex_index = 	threadIdx.x + (blockIdx.x * blockDim.x);
	if ( vertex_index < num_vertices ) {
		// Grab the vertex
		float3 vertex = mesh_vertices[vertex_index];

		// Transform to camera space
		int3 camera_coord = world_to_pixel( vertex, inv_pose, k );

		// Scene flow index...
		int scene_flw_index = camera_coord.y * sf_width + camera_coord.x;

		// Dereference scene flow
		float3 sf_at_vertex = scene_flow[scene_flw_index];

		// Stick it into the out mesh
		mesh_scene_flow[vertex_index] = sf_at_vertex;
	}
}


/**
 * Kernel to obtain scene flow vector for each point in the surface mesh
 * @param vertices The mesh vertices
 * @param camera The Camera
 * @param sf_width The width of the scene flow image
 * @param sf_height The height of the scene flow image
 * @param scene_flow The scene flow image data
 * @param mesh_scene_flow An output vector fo the scene flow values for each vertex of he mesh
 */
__host__
void get_scene_flow_for_mesh(	const std::vector<float3> vertices,
                                const Camera * camera,
                                uint32_t sf_width,
                                uint32_t sf_height,
                                const float3 * scene_flow,
                                std::vector<float3>& mesh_scene_flow ) {

	std::cout << "-- get_scene_flow_for_mesh" << std::endl;

	size_t alloc_size = vertices.size() * sizeof( float3 );

	// Allocate memory for mesh scene flow values on the device
	float3 * d_mesh_scene_flow;
	cudaError_t err = cudaMalloc( &d_mesh_scene_flow, alloc_size );
	if ( err != cudaSuccess ) {
		std::cout << "Couldn't allocate device memory for scene flow output for mesh" << std::endl;
		throw std::bad_alloc( );
	}

	// Allocate memory for mesh vertices on the device
	float3 * d_mesh_vertices;
	err = cudaMalloc( &d_mesh_vertices, alloc_size );
	if ( err != cudaSuccess ) {
		cudaFree( d_mesh_scene_flow );
		std::cout << "Couldn't allocate device memory for mesh vertices" << std::endl;
		throw std::bad_alloc( );
	}

	// Allocare memory for the raw scene flow data on the device
	float3 * d_scene_flow;
	err = cudaMalloc( &d_scene_flow, sf_width * sf_height  * sizeof( float3 ) );
	if ( err != cudaSuccess ) {
		cudaFree( d_mesh_scene_flow );
		cudaFree( d_mesh_vertices );
		std::cout << "Couldn't allocate device memory for raw scene flow input" << std::endl;
		throw std::bad_alloc( );
	}

	// Allocate host memory for output scene flow values
	float3 * h_mesh_scene_flow = new float3[vertices.size()];
	if ( !h_mesh_scene_flow ) {
		cudaFree( d_mesh_scene_flow );
		cudaFree( d_mesh_vertices );
		cudaFree( d_scene_flow );
		std::cout << "Couldn't allocate host memory for output of mesh scene flow" << std::endl;
		throw std::bad_alloc( );
	}

	// Now copy all data onto device
	// Scene flow copies directly:
	err = cudaMemcpy( d_scene_flow, scene_flow, sf_width * sf_height * sizeof( float3 ), cudaMemcpyHostToDevice);
	check_cuda_error( "Copy of input scene flow data to device failed " , err);

	// Vertex data
	err = cudaMemcpy( d_mesh_vertices, &(vertices[0]), alloc_size, cudaMemcpyHostToDevice);
	check_cuda_error( "Copy of input mesh vertices to device failed " , err);

	// Invoke kernel
	Mat44 inv_pose;
	memcpy( &inv_pose, (void *)(camera->inverse_pose().data()) , 16 * sizeof( float ) );

	Mat33 k;
	memcpy( &k, (void *) (camera->k().data()), 9 * sizeof( float ) );

	dim3 block( 128, 1, 1 );
	dim3 grid ( divUp( vertices.size(), block.x ), 1, 1 );
	std::cout << "--- Launch mesh_scene_flow_kernel: grid [" << grid.x << ", " << grid.y << ", " << grid.z << "] " << std::endl;
	mesh_scene_flow_kernel <<< grid, block >>>(
	    d_mesh_vertices, 		// Input mesh (device memory)
	    vertices.size(),
	    d_scene_flow, 			//	Input raw scene flow for whole image
	    sf_width, 				// 	Dimesnions of scene flow
	    sf_height,
	    inv_pose,				// Camera data
	    k,
	    d_mesh_scene_flow );
	cudaDeviceSynchronize();
	err = cudaGetLastError();
	check_cuda_error( "mesh_scene_flow kernel failed " , err);

	// Copy mesh scene flow back from device
	err = cudaMemcpy( h_mesh_scene_flow, d_mesh_scene_flow, alloc_size, cudaMemcpyDeviceToHost);
	check_cuda_error( "Copy of output mesh scene flow to host failed " , err);

	// Now unpack from memory to vector
	mesh_scene_flow.assign( h_mesh_scene_flow, h_mesh_scene_flow + vertices.size());

	// Now tidy up memory
	err = cudaFree( d_mesh_scene_flow );
	check_cuda_error( "get_scene_flow_for_mesh: Couldn't free device mesh scene flow " , err);
	err = cudaFree( d_mesh_vertices );
	check_cuda_error( "get_scene_flow_for_mesh: Couldn't free device vertices " , err);
	err = cudaFree( d_scene_flow );
	check_cuda_error( "get_scene_flow_for_mesh: Couldn't free device scene flow " , err);
	delete [] h_mesh_scene_flow;

	std::cout << "-- get_scene_flow_for_mesh exit ok" << std::endl;
}

__host__
/**
 * @param volume The TSDF Volume to update
 * @param mesh_scene_flow The scene flow per mesh node
 * @param mesh_vertices The vertices of the mesh
 * @param num_vertices Number of vertices
 */
void update_voxel_grid_from_mesh_scene_flow(
    const TSDFVolume *volume,
    const float3 * mesh_scene_flow,		// Assumed to be on device
    const float3 * mesh_vertices,		// Assumed to be on device
    int num_vertices) {

	std::cout << "-- update_voxel_grid_from_mesh_scene_flow began for " << num_vertices << " vertices" << std::endl;

	dim3 block( 1, 32, 32 );
	dim3 grid ( 1, divUp( volume->size().y, block.y ), divUp( volume->size().z, block.z ));

	float3 *translation_data = (float3 *) volume->translation_data();

	std::cout << "--- Launch apply_scene_flow_to_tsdf_kernel: grid [" << grid.x << ", " << grid.y << ", " << grid.z << "] " << std::endl;
	apply_scene_flow_to_tsdf_kernel <<< grid, block >>> (
	    mesh_scene_flow,			//	The scene flow per mesh vertex
	    mesh_vertices,				//	The coordinates of the mesh vertex
	    num_vertices,				//	Number of vertices in the mesh
	    translation_data,			//	Deformation data for the TSDF
	    volume->size()				//	Dimensions of the TSDF in voxels
	);
	cudaDeviceSynchronize();
	cudaError_t err = cudaGetLastError();
	check_cuda_error( "update_voxel_grid_from_mesh_scene_flow kernel failed " , err);


	std::cout << "-- update_voxel_grid_from_mesh_scene_flow ended ok" << std::endl;
}



/**
 * Update the Given TSDF volume's per voxel translation using the input Scene Flow
 * @param volume The TSDF Volume to update
 * @param translation The global translation
 * @param rotation The Global rotation
 * @param residuals Per voxel ransation after globals are appliedd
 */
void update_tsdf(	const TSDFVolume 								* volume,
                    const Camera 									* camera,
                    uint16_t 										width,
                    uint16_t 										height,
                    const Eigen::Vector3f 							translation,
                    const Eigen::Vector3f 							rotation,
                    const Eigen::Matrix<float, 3, Eigen::Dynamic> 	residuals ) {

	std::cout << "- update_tsdf" << std::endl;
	// Get the mesh from the current TSDF as a set of triangles and vertices
	// though we only really care about the vertices
	std::vector<float3> vertices;
	std::vector<int3> triangles;
	extract_surface( volume, vertices, triangles);

	// If the mesh exists
	if ( vertices.size() > 0 ) {


		// Convert residual data to an array of float3
		float3 * scene_flow = new float3[ width * height];
		if ( scene_flow ) {
			for ( int i = 0; i < width * height; i++ ) {
				scene_flow[i].x = residuals( 0, i );
				scene_flow[i].y = residuals( 1, i );
				scene_flow[i].z = residuals( 2, i );
			}


			// Construct another vector to hold the scene flow just for the mesh
			std::vector<float3> mesh_scene_flow;

			// Populate this from the original scene flow data
			// mesg_scene_flow now contains SF data for each vertex in vertices
			get_scene_flow_for_mesh( vertices, camera, width, height, scene_flow, mesh_scene_flow );
			// Delete scene flow as it's no longer needed
			delete[] scene_flow;

			if ( mesh_scene_flow.size() == vertices.size() ) {

				// Transfer the mesh scene flow data nd the vertex data into device memory
				float3 * d_mesh_vertices;
				cudaError_t err = cudaMalloc( &d_mesh_vertices, vertices.size() * sizeof( float3 ) );
				if ( err != cudaSuccess ) {
					std::cout << "update_tsdf: Couldn't allocate device memory for mesh vertices" << std::endl;
					throw std::bad_alloc( );
				}

				float3 * d_mesh_scene_flow;
				err = cudaMalloc( &d_mesh_scene_flow, vertices.size() * sizeof( float3) );
				if ( err != cudaSuccess ) {
					cudaFree( d_mesh_vertices);
					std::cout << "update_tsdf: Couldn't allocate device memory for mesh scene flow" << std::endl;
					throw std::bad_alloc( );
				}
				err = cudaMemcpy( d_mesh_vertices, & (vertices[0]), vertices.size() * sizeof( float3), cudaMemcpyHostToDevice);
				check_cuda_error( "update_tsdf: Failed to copy mesh vertex data to device", err );

				err = cudaMemcpy( d_mesh_scene_flow, & (mesh_scene_flow[0]), vertices.size() * sizeof( float3), cudaMemcpyHostToDevice);
				check_cuda_error( "update_tsdf: Failed to copy mesh scene flow data to device", err );

				// Now update the TSDF voxel centres using the mesh data
				update_voxel_grid_from_mesh_scene_flow( volume, d_mesh_scene_flow, d_mesh_vertices, vertices.size());

				std::cout << "- update_tsdf exited ok" << std::endl;
			} else {
				std::cout << "update_tsdf: Mesh scene flow vector is the wrong size [" << mesh_scene_flow.size() << "] when it should have [" << vertices.size() << "] elements" << std::endl;
			}
		} else {
			std::cout << "Couldn't allocate memory for scene flow" << std::endl;
		}
	} else {
		std::cout << "No mesh in voxel grid" << std::endl;
	}
}


/**
 * Extract mesh into device memory
 */
void get_mesh( const TSDFVolume * volume , int&  num_vertices, float3 * d_mesh_vertices, int& num_triangles, int3 * d_mesh_triangles) {

	std::cout << "-- get_mesh" << std::endl;


	// First extract the mesh corresponding to the ISO surface in the TSDF in frame t-1
	std::vector<float3> vertices;
	std::vector<int3> triangles;
	extract_surface( volume, vertices, triangles);

	// If the mesh doesn't exist, we're done
	if ( vertices.size() == 0 ) {
		std::cout << "get_mesh(): No mesh in voxel grid" << std::endl;
		return;
	}

	// Map vertices and triangles into device memory
	cudaError_t err = cudaMalloc( &d_mesh_vertices, vertices.size() * sizeof( float3 ) );
	if ( err != cudaSuccess ) {
		std::cout << "get_mesh(): Couldn't allocate device memory for mesh vertices" << std::endl;
		throw std::bad_alloc( );
	}

	err = cudaMalloc( &d_mesh_triangles, triangles.size() * sizeof( int3) );
	if ( err != cudaSuccess ) {
		cudaFree( d_mesh_vertices);
		std::cout << "get_mesh(): Couldn't allocate device memory for mesh triangles" << std::endl;
		throw std::bad_alloc( );
	}
	err = cudaMemcpy( d_mesh_vertices, & (vertices[0]), vertices.size() * sizeof( float3), cudaMemcpyHostToDevice);
	check_cuda_error( "get_mesh(): Failed to copy mesh vertex data to device", err );

	err = cudaMemcpy( d_mesh_triangles, & (triangles[0]), triangles.size() * sizeof( int3), cudaMemcpyHostToDevice);
	check_cuda_error( "get_mesh(): Failed to copy mesh triangle data to device", err );
}





/*
	Process is s follows
	1. Extract the mesh from the TSDF
		a. Use marching cubes on the undeformed TSDF
		b. Apply warp field to the vertices of the mesh
	2. For each vertex in the mesh, if the vertex is visible, compute scene flow
		a. Compute the scene flow from frame n to frame n+1
		b. Project each mesh vertex m into the depth image from frame n giving D(x,y)
		c. Extract depth and reproject point into 3D space. If this points is within some threshold of original mesh vertex then its a match
		d. Store scene flow for the matched vertex
	3. For each vertex in the mesh fo which we have scene flow values, update deformation of surrounding voxels
		a.	

 */


/**
 * Compute the scene flow for the given mesh by:
 * Porjecting each vertex of the mesh into pixel coordinates
 * reprojecting the corresponding pixek into 3D space
 * If te mesh vertex and pixel projections are 'close' then they correspond
 * Look up the corresponding scene flow and assign to that mesh vertex
 *
 * The output number of mesh vertices will be greatly reduced from the input number
 * and we can actually dispose of the input mesh once we've completed this stage
 * of the pipeline.
 */
__global__
void scene_flow_for_mesh_kernel(const float3 	*mesh_vertices, 
								int 			num_mesh_vertices,
								uint16_t 		width, 
								uint16_t 		height,
								const float 	*depth_map, 
								const float3 	*scene_flow,
								const Mat33 	k,
								const Mat33		kinv,
								const Mat44		pose,
								const Mat44 	inv_pose,
								const float 	threshold
								) {
	int mesh_vertex_index = threadIdx.x + ( blockIdx.x * blockDim.x );
	if( mesh_vertex_index >= num_mesh_vertices ) return;

	float3 mesh_vertex = mesh_vertices[ mesh_vertex_index ];

	int3 pixel = world_to_pixel( inv_pose, k, mesh_vertex );

	if( pixel.x < width && pixel.x >= 0 && pixel.y < height && pixel.y > 0 ) {

		// Project this pixel point back into space
		int pixel_index = width * pixel.y + pixel.x;
		float depth = depth_map[ pixel_index];

		float3 pixel_cam_coordinate = m3_i3_mul( kinv, pixel );
		pixel_cam_coordinate = f3_mul_scalar( depth, pixel_cam_coordinate);

		float3 pixel_world_coordinate {
			pose.m11 * pixel_cam_coordinate.x + pose.m12 * pixel_cam_coordinate.y + pose.m13 * pixel_cam_coordinate.z + pose.m14,
			pose.m21 * pixel_cam_coordinate.x + pose.m22 * pixel_cam_coordinate.y + pose.m23 * pixel_cam_coordinate.z + pose.m24,
			pose.m31 * pixel_cam_coordinate.x + pose.m32 * pixel_cam_coordinate.y + pose.m33 * pixel_cam_coordinate.z + pose.m34
		};
		float w = pose.m41 * pixel_cam_coordinate.x + pose.m42 * pixel_cam_coordinate.y + pose.m43 * pixel_cam_coordinate.z + pose.m44;
		pixel_world_coordinate.x /= w;
		pixel_world_coordinate.y /= w;
		pixel_world_coordinate.z /= w;


		// get the distance between this reprojected point and the oruginal mesh point
		float3 delta = f3_sub( mesh_vertex, pixel_world_coordinate);

		// And distance
		float distance = f3_norm( delta );


		// If this is essentially the same point, handle it
		if( distance < threshold) {
			// Look up scne flow for this point and add it to the outbound list
			// outbound list can essentially be a list of mesh indices and scene flow indices.

		}
	} // else not in view frustrum

}
/**
 * Update the Given TSDF volume's per voxel translation using the input Scene Flow
 * @param volume The TSDF Volume to update
 * @param translation The global translation
 * @param rotation The Global rotation
 * @param residuals Per voxel ransation after globals are appliedd
 */
void update_tsdf_2(	const TSDFVolume 								* volume,
                    const Camera 									* camera,
                    uint16_t 										width,
                    uint16_t 										height,
                    const Eigen::Vector3f 							translation,
                    const Eigen::Vector3f 							rotation,
                    const Eigen::Matrix<float, 3, Eigen::Dynamic> 	residuals ) {

	std::cout << "- update_tsdf_2" << std::endl;

	// Extract the mesh from the TSDF
	int num_vertices, num_triangles;
	float3 * d_vertices = nullptr;
	int3   * d_triangles= nullptr;
	get_mesh( volume, num_vertices, d_vertices, num_triangles, d_triangles );


	// Now for each vertex of the mesh, get the scene flow (if it's 'at the front' )
//	scene_flow_for_mesh( num_vertices, vertices, num_triangles, triangles, camera, num_vertices_out, vertc)


	// Allocate memory for each vertex in the mesh to store voxel updates

	// Call kernel to compute voxel updates for neighbours of each mesh vertex

	// Coalesce these into a single set of voxel updates


}

