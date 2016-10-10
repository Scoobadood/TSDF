#include "../include/SceneFlowUpdater.hpp"
#include "../include/cu_common.hpp"



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
/**
 * THe mesh scene flow kernel currently does nothing
 */
__global__
void mesh_scene_flow_kernel(  
	float3 * d_mesh_vertices, 		// Input mesh
	uint32_t num_vertices,			
	float3 * d_scene_flow, 			//	Input raw scene flow for whole image
	uint32_t sf_width, 				// 	Dimesnions of scene flow
	uint32_t sf_height, 
	Mat44 	 inv_pose,				// Camera data
	Mat33	 k, 
	float3 * d_mesh_scene_flow	 	// Output of scene flow for each point in the input mesh
	) {

	// Vertex index - 
	int vertex_index = 	threadIdx.x + (blockIdx.x * blockDim.x);

	// Grab the vertex
	float3 vertex = d_mesh_vertices[vertex_index];

	// Transform to camera space
	int3 camera_coord = world_to_pixel( vertex, inv_pose, k );

	// Scene flow index...
	int scene_flw_index = camera_coord.y * sf_width + camera_coord.x;

	// Dereference scene flow
	float3 sf_at_vertex = d_scene_flow[scene_flw_index];

	// Stick it into the out mesh
	d_mesh_scene_flow[vertex_index] = sf_at_vertex;
}


/**
 * Kernel to obtain scene flow vector for each point in the surface mesh
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
	if( !h_mesh_scene_flow ) {
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
	mesh_scene_flow.assign( h_mesh_scene_flow, h_mesh_scene_flow+vertices.size());

	// Now tidy up memory
		cudaFree( d_mesh_scene_flow );
		cudaFree( d_mesh_vertices );
		cudaFree( d_scene_flow );
		delete [] h_mesh_scene_flow;
}

/**
 * Update the Given TSDF volume's per voxel translation using the input Scene Flow 
 * @param volume The TSDF Volume to update
 * @param translation The global translation
 * @param rotation The Global rotation
 * @param residuals Per voxel ransation after globals are appliedd
 */
void update_tsdf( const TSDFVolume * volume, const Eigen::Vector3f translation, const Eigen::Vector3f rotation, const Eigen::Matrix<float, 3, Eigen::Dynamic> residuals ) {
}

