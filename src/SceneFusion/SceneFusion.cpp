#include "../include/SceneFusion.hpp"
#include "../include/SceneFusion_krnl.hpp"
#include "../include/FileUtilities.hpp"
#include "../include/MarkAndSweepMC.hpp"
#include "../include/TSDFVolume.hpp"
#include "../include/Camera.hpp"
#include "../include/ply.hpp"

/*
	This is the Plan A variant

	At time t=1
	Compute sf_1 ( D_0, D_1, RGB_0. RGB_1)

	Extract the mesh and in the process, also
		Extract and store the indices of the voxels which form the end points of the 
		lines on which the mesh vertex falls

	For each mesh vertex, determine if it's visible by:
		Project mesh vertex into depth map
		Find corresponding 3D point for the depth at that pixel
		If the points are 
			Within some predefined distance [current]
			Within some predefined depth    ]possibly preferred alternative]
		Then the original mesh point is visible

	For each visible mesh point determine the scene flow
		Project the mesh point into the image plane (pixels) 
		Lookup the associated scene flow at that pixel point

	For each voxel which is adjacent to a voxel mesh
		Apply the mean translation to that mesh point

	Now project the new depth image into the updated transformed voxel grid
	


*/


/**
 * The main class for SceneFusion
 * Responsible for pulling frames from the Device class and merging them into the TSDFVolume
 *
 */
SceneFusion::SceneFusion( SceneFlowAlgorithm * sfa, RGBDDevice * rgbd_device ) {

	// Construct the TSDFVolume
	m_volume = new TSDFVolume(255, 255 , 255, 2550, 2550, 2550);

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

        const char * homedir = get_home_directory( );
        if( homedir ) {
		    sprintf( out_file_name, "%s/Desktop/mesh_%03d_normal.ply", homedir, frames);
        } else {
        	sprintf( out_file_name, "mesh_%03d_normal.ply", frames);
        }
        std::cout << "Writing normal PLY to " << out_file_name << std::endl;
   	write_to_ply( out_file_name, verts, triangles);
	
	if( homedir ) {
		    sprintf( out_file_name, "%s/Desktop/mesh_%03d_warped.ply", homedir, frames);
        } else {
        	sprintf( out_file_name, "mesh_%03d_warped.ply", frames);
        }
	    std::cout << "Writing warped PLY to " << out_file_name << std::endl;
	    m_volume->deform_mesh( verts.size(), verts.data() );
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
