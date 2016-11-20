#include "../include/GPUMarchingCubes.hpp"
#include "../include/SceneFlowUpdater.hpp"
#include "../include/cu_common.hpp"

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
	pixel_coordinate.y = round( image_coordinate.x / image_coordinate.z);

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

			for ( int i = 0; i < num_mesh_vertices; i++ ) {
				// TODO: Replace this with a radial basis function for weighted deformation
				float3 vector_to_vertex = f3_sub( voxel_translations[voxel_index], mesh_vertices[i]);
				float dist_to_vertex = f3_norm( vector_to_vertex);

				if ( dist_to_vertex < THRESHOLD ) {
					deformation = f3_add( deformation, mesh_scene_flow[i]);
					num_impacting_mesh_nodes++;
				}
			}
			if( num_impacting_mesh_nodes > 0 ) {
				deformation = f3_mul_scalar( deformation, 1.0f / num_impacting_mesh_nodes );
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
    float3 * mesh_vertices, 		// Input mesh
    uint32_t num_vertices,
    float3 * scene_flow, 			//	Input raw scene flow for whole image
    uint32_t sf_width, 				// 	Dimesnions of scene flow
    uint32_t sf_height,
    Mat44 	 inv_pose,				// Camera data
    Mat33	 k,
    float3 * mesh_scene_flow	 	// Output of scene flow for each point in the input mesh
) {

	// Vertex index -
	int vertex_index = 	threadIdx.x + (blockIdx.x * blockDim.x);

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
void get_scene_flow_for_mesh( const std::vector<float3> vertices, const Camera * camera, uint32_t sf_width, uint32_t sf_height, const float3 * scene_flow, std::vector<float3> mesh_scene_flow ) {
	//Allocate memory for mesh scene flow values on the device
	float3 * d_mesh_scene_flow;
	cudaError_t err = cudaMalloc( &d_mesh_scene_flow, vertices.size() * sizeof( float3 ) );
	if ( err != cudaSuccess ) {
		std::cout << "Couldn't allocate device memory for scene flow output for mesh" << std::endl;
		throw std::bad_alloc( );
	}

	// Store mesh vertices on the device
	float3 * d_mesh_vertices;
	err = cudaMalloc( &d_mesh_vertices, vertices.size() * sizeof( float3 ) );
	if ( err != cudaSuccess ) {
		cudaFree( d_mesh_scene_flow );
		std::cout << "Couldn't allocate device memory for mesh vertices" << std::endl;
		throw std::bad_alloc( );
	}

	// Store the raw scene flow data on the device
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
	std::vector<double> v;
	const float3 *vertices_as_array = &vertices[0];
	err = cudaMemcpy( d_mesh_vertices, vertices_as_array, vertices.size() * sizeof( float3 ), cudaMemcpyHostToDevice);
	check_cuda_error( "Copy of input mesh vertices to device failed " , err);

	// Invoke kernel
	Mat44 inv_pose;
	memcpy( &inv_pose, (void *)(camera->inverse_pose().data()) , 16 * sizeof( float ) );

	Mat33 k;
	memcpy( &k, (void *) (camera->k().data()), 9 * sizeof( float ) );

	dim3 block( 128, 1, 1 );
	dim3 grid ( divUp( vertices.size(), block.x ), 1, 1 );
	mesh_scene_flow_kernel <<< grid, block >>>(
	    d_mesh_vertices, 		// Input mesh
	    vertices.size(),
	    d_scene_flow, 			//	Input raw scene flow for whole image
	    sf_width, 				// 	Dimesnions of scene flow
	    sf_height,
	    inv_pose,				// Camera data
	    k,
	    d_mesh_scene_flow );

	// Copy mesh scene flow back from device
	err = cudaMemcpy( h_mesh_scene_flow, d_mesh_scene_flow, vertices.size() * sizeof( float3 ), cudaMemcpyDeviceToHost);
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
    const float3 * mesh_scene_flow,
    const float3 * mesh_vertices,
    int num_vertices) {

	dim3 block( 1, 32, 32 );
	dim3 grid ( 1, divUp( volume->size().y, block.y ), divUp( volume->size().z, block.z ));

	float3 *translation_data = (float3 *) volume->translation_data();

	apply_scene_flow_to_tsdf_kernel <<< grid, block >>> (
	    mesh_scene_flow,			//	The scene flow per mesh vertex
	    mesh_vertices,				//	The coordinates of the mesh vertex
	    num_vertices,				//	Number of vertices in the mesh
	    translation_data,			//	Defromation data for the TSDF
	    volume->size()				//	Dimensions of the TSDF in voxels
	);
}



/**
 * Update the Given TSDF volume's per voxel translation using the input Scene Flow
 * @param volume The TSDF Volume to update
 * @param translation The global translation
 * @param rotation The Global rotation
 * @param residuals Per voxel ransation after globals are appliedd
 */
void update_tsdf( const TSDFVolume * volume, const Camera * camera, uint16_t width, uint16_t height, const Eigen::Vector3f translation, const Eigen::Vector3f rotation, const Eigen::Matrix<float, 3, Eigen::Dynamic> residuals ) {

	// Get the mesh from the current TSDF
	std::vector<float3> vertices;
	std::vector<int3> triangles;
	extract_surface( volume, vertices, triangles);

	// Populate the scene_flow with residula data
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
		get_scene_flow_for_mesh( vertices, camera, width, height, scene_flow, mesh_scene_flow );

		// And update the TSDF voxel centres using the mesh data
		const float3 * vertex_data = (const float3 *) & (vertices[0]);
		const float3 * mesh_scene_flow_data = (const float3 *) & (mesh_scene_flow[0]);
		update_voxel_grid_from_mesh_scene_flow( volume, mesh_scene_flow_data, vertex_data, vertices.size());

		// Delete the scene flow
		delete[] scene_flow;
	} else {
		std::cout << "Couldn't allocate memory for scene flow" << std::endl;
	}
}

