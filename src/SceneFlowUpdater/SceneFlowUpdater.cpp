

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
	Mat44 pose,						// Camera data
	Mat33 k, 
	Mat33 kinv, 
	float3 * d_mesh_scene_flow	 	// Output of scene flow for each point in the input mesh
	) {

}


/**
 * Kernel to obtain scene flow vector for each point in the surface mesh
 */
__host__
void get_scene_flow_for_mesh( const std::vector<float3> vertices, const Camera * camera, uint32_t sf_width, uint32_t sf_height, const float3 * scene_flow, std::vector<float3> mesh_scene_flow ) {
	//Allocate memory for mesh scene flow values on the device
	float3 * d_mesh_scene_flow;
	cudaError_t err = cudaMalloc( &d_mesh_scene_flow, vertices.length * sizeof( float3 ) );
	if ( err != cudaSuccess ) {
		std::cout << "Couldn't allocate device memory for scene flow output for mesh" << std::endl;
		throw std::bad_alloc( );
	}

	// Store mesh vertices on the device
	float3 * d_mesh_vertices;
	cudaError_t err = cudaMalloc( &d_mesh_vertices, vertices.length * sizeof( float3 ) );
	if ( err != cudaSuccess ) {
		cudaFree( d_mesh_scene_flow );
		std::cout << "Couldn't allocate device memory for mesh vertices" << std::endl;
		throw std::bad_alloc( );
	}

	// Store the raw scene flow data on the device
	float3 * d_scene_flow;
	cudaError_t err = cudaMalloc( &d_scene_flow, sf_width * sf_height  * sizeof( float3 ) );
	if ( err != cudaSuccess ) {
		cudaFree( d_mesh_scene_flow );
		cudaFree( d_mesh_vertices );
		std::cout << "Couldn't allocate device memory for raw scene flow input" << std::endl;
		throw std::bad_alloc( );
	}

	// Allocate host memory for output scene flow values
	float3 * h_mesh_scene_flow = new float3[vetrices.length];
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
	float3 *vertices_as_array = &vertices[0];
	err = cudaMemcpy( d_mesh_vertices, vertices_as_array, vertices.length * sizeof( float3 ), cudaMemcpyHostToDevice);
	check_cuda_error( "Copy of input mesh vertices to device failed " , err);

	// Invoke kernel
	Mat44 pose;
    memcpy( &pose, camera.pose().data(), 16 * sizeof( float ) );

    Mat33 k;
    memcpy( &k, camera.k().data(), 9 * sizeof( float ) );

    Mat33 kinv;
    memcpy( &kinv, camera.kinv().data(), 9 * sizeof( float ) );

	dim3 block( 32, 32, 1 );
	dim3 grid ( divUp( voxel_grid_size.x, block.x ), divUp( voxel_grid_size.y, block.y ), 1 );
	mesh_scene_flow_kernel <<< grid, block >>>( 
		d_mesh_vertices, 		// Input mesh
		vertices.length,			
		d_scene_flow, 			//	Input raw scene flow for whole image
		sf_width, 				// 	Dimesnions of scene flow
		sf_height, 
		pose,						// Camera data
		k, 
		kinv, 
		d_mesh_scene_flow );

	// Copy mesh scene flow back from device
	err = cudaMemcpy( h_mesh_scene_flow, d_mesh_scene_flow, vertices.length * sizeof( float3 ), cudaMemcpyDeviceToHost);
	check_cuda_error( "Copy of output mesh scene flow to host failed " , err);

	// Now unpack from memory to vector
	mesh_scene_flow.assign( h_mesh_scene_flow, h_mesh_scene_flow+vertices.length);

	// Now tidy up memory
		cudaFree( d_mesh_scene_flow );
		cudaFree( d_mesh_vertices );
		cudaFree( d_scene_flow );
		free[] h_mesh_scene_flow;
}

/**
 * Update the Given TSDF volume's per voxel translation using the input Scene Flow 
 * @param volume The TSDF Volume to update
 * @param translation The global translation
 * @param rotation The Global rotation
 * @param residuals Per voxel ransation after globals are appliedd
 */
void update_tsdf( const phd::TSDFVolume * volume, const Eigen::Vector3f translation, const Eigen::Vector3f rotation, const Eigen::Matrix<float, 3, Eigen::Dynamic> residuals ) {
}

