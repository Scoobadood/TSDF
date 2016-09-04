#include "SceneFusion.hpp"

/**
 * The main class for SceneFusion
 * Responsible for pulling frames from the Device class and merging them into the TSDFVolume
 *
 */
SceneFusion::SceneFusion( SceneFlowAlgorithm * sfa, RGBDDevice * rgbd_device ) {

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

	// And dispose of them
}