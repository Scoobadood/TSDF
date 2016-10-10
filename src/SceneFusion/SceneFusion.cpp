#include "SceneFusion.hpp"
#include "../include/SceneFlowUpdater.hpp"
#include "../include/GPUMarchingCubes.hpp"
#include "../include/GPUTSDFVolume.hpp"
#include "../include/Camera.hpp"

#include "vector_types.h"

#include <vector>

/**
 * The main class for SceneFusion
 * Responsible for pulling frames from the Device class and merging them into the TSDFVolume
 *
 */
SceneFusion::SceneFusion( SceneFlowAlgorithm * sfa, RGBDDevice * rgbd_device ) {

	// Construct the TSDFVolume
	m_volume = new GPUTSDFVolume();
	m_volume->offset( -1500, -1500, -1500);

    // And camera (from FREI 1 IR calibration data at TUM)
    m_camera = new Camera{ 591.1f, 590.1f, 331.0f, 234.6f };


	// Check parms are valid
	m_rgbd_device = rgbd_device;
	m_scene_flow_algorithm = sfa;

	// Register for callbacks
	using namespace std::placeholders;
	RGBDDeviceCallback callback = std::bind( &SceneFusion::process_frames, this, _1, _2  );
	m_rgbd_device->addObserver( callback );
}

/**
 * Run SceneFusion
 */
void SceneFusion::process_frames( const DepthImage * depth_image, const PngWrapper * colour_image ) {
	std::cout << "processFrames Called" << std::endl;
	// Compute the scene flow
	Eigen::Vector3f translation;
	Eigen::Vector3f rotation;
	Eigen::Matrix<float, 3, Eigen::Dynamic> residuals;
	m_scene_flow_algorithm->compute_scene_flow( depth_image, colour_image, translation, rotation, residuals );


	// Process them into TSDF
	std::vector<float3> vertices;
	std::vector<int3> triangles;
	extract_surface( m_volume, vertices, triangles);

	std::vector<float3> mesh_scene_flow;
	const float3* scene_flow = (const float3 *) residuals.data();
	get_scene_flow_for_mesh( vertices, m_camera, colour_image->width(), colour_image->height(), scene_flow, mesh_scene_flow );
}