#include "../include/SceneFusion.hpp"
#include "../include/SceneFusion_krnl.hpp"
#include "../include/MarkAndSweepMC.hpp"
#include "../include/TSDFVolume.hpp"
#include "../include/Camera.hpp"
#include "../include/ply.hpp"

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


/**
 * The main class for SceneFusion
 * Responsible for pulling frames from the Device class and merging them into the TSDFVolume
 *
 */
SceneFusion::SceneFusion( SceneFlowAlgorithm * sfa, RGBDDevice * rgbd_device ) {

	// Construct the TSDFVolume
	m_volume = new TSDFVolume(400, 400 , 400, 2000, 2000, 2000);

	// And camera (from FREI 1 IR calibration data at TUM)
	m_camera = Camera::default_depth_camera( );

	// Pose the camera
	m_camera->move_to( 1000, 1000, 0 );
	m_camera->look_at( 1000, 1000, 1000 );


	m_last_depth_image = nullptr;


	// Check parms are valid
	m_rgbd_device = rgbd_device;
	m_scene_flow_algorithm = sfa;

	// Register for callbacks
	using namespace std::placeholders;
	RGBDDeviceCallback callback = std::bind( &SceneFusion::process_frames, this, _1, _2  );
	m_rgbd_device->addObserver( callback );
}

SceneFusion::~SceneFusion() {
	if ( m_volume ) delete m_volume;
	if ( m_camera ) delete m_camera;
	if ( m_last_depth_image ) delete[] m_last_depth_image;
}


/**
 * Run SceneFusion
 */
__host__
void SceneFusion::process_frames( const DepthImage * depth_image, const PngWrapper * colour_image ) {
	static int frames = 0;

	std::cout << "------------------------------------------------------------" << std::endl;
	std::cout << "processFrames Called" << std::endl;
	std::cout << "-- Depth image :  " << depth_image << std::endl;
	std::cout << "          data :  " << depth_image->data() << std::endl;


	assert( depth_image );
	assert( colour_image );

	uint16_t width = depth_image->width();
	uint16_t height = depth_image->height();
	assert( width > 0 );
	assert( height > 0 );


	// If this has been called before, we have a prior depth and can compute the next scene flow so do so.
	if ( m_last_depth_image != nullptr ) {
		std::cout << "Called for second or subsequent time" << std::endl;

		// Compute the scene flow
		std::cout << "-- getting scene flow" << std::endl;
		Eigen::Vector3f translation;
		Eigen::Vector3f rotation;
		Eigen::Matrix<float, 3, Eigen::Dynamic> residuals;
		m_scene_flow_algorithm->compute_scene_flow( depth_image, colour_image, translation, rotation, residuals );


		// Global translation and rotation are ignored here.
		// Residuals is in host memory and is the actual scene flow
		std::cout << "-- got it" << std::endl;

		// Process the update with the last depth image data and this scene flow.
		::process_frames( m_volume, m_camera, width, height, m_last_depth_image, (float3 *)residuals.data() );

	} 

	// Otherwise, this is the first call and we should allocate storage for the depth image for the next one
	else {
		std::cout << "Called for first time" << std::endl;
		m_last_depth_image = (uint16_t *)malloc( sizeof( uint16_t) * width * height );
		if ( !m_last_depth_image) {
			std::cout << "-- Couldn't create storage for host depth data" << std::endl;
		}
	}

	std::cout << "-- Storing depth and colour data" << std::endl;
	// Save the current image to the last. 
	// Ultimately store RGB too
	memcpy( (void *)m_last_depth_image, (void *)depth_image->data(), sizeof( uint16_t) * width * height );

	// Now update the depth map into the TSDF
	std::cout << "-- Integrating the new depth image into the TSDF" << std::endl;
	m_volume->integrate(  depth_image->data(), width, height, *m_camera );

	// This line for DEBUG reasons while testing deformtaion code
	// if ( frames == 0  ) m_volume->integrate(  depth_image->data(), width, height, *m_camera );



	// Dump output periodically
	frames++;
	if( (frames <20) || (frames%10 == 0 ) ) {
	    char out_file_name[1000];

		 // Save to PLY file
        std::vector<int3> triangles;
        std::vector<float3> verts;
        extract_surface( verts, triangles );
	    std::cout << "Writing to PLY" << std::endl;
	    sprintf( out_file_name, "/home/dave/Desktop/mesh_%03d.ply", frames);
   	    write_to_ply( out_file_name, verts, triangles);
	}



	std::cout << "------------------------------------------------------------" << std::endl;
}


/*
 * Extract the mesh
 */
void SceneFusion::extract_surface( std::vector<float3>& verts, std::vector<int3>& triangles ) const {
	::extract_surface( m_volume, verts, triangles);
}
