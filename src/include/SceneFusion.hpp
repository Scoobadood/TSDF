#ifndef SCENE_FUSION_HPP
#define SCENE_FUSION_HPP

#include "TSDFVolume.hpp"
#include "SceneFlowAlgorithm.hpp"
#include "RGBDDevice.hpp"
#include "Camera.hpp"
#include "PngWrapper.hpp"
#include "DepthImage.hpp"



typedef struct {
	int vertex_index;
	float3	scene_flow;
} MeshSceneFlowItem;


/**
 * The main class for SceneFusion
 * Responsible for pulling frames from the Device class and merging them into the TSDFVolume
 *
 */
class SceneFusion {
public:
	/**
	 * Construct a new SceneFusion object
	 */
	SceneFusion( SceneFlowAlgorithm * sfa, RGBDDevice * rgbd_device );

	~SceneFusion();


	/**
	 * Run SceneFusion
	 */
	void process_frames( const DepthImage * depth_image, const PngWrapper *  colour_image );

	/*
	 * Extract the mesh
	 */
	void extract_surface( std::vector<float3>& verts, std::vector<int3>& triangles ) const;


private:
	TSDFVolume				* m_volume;

	SceneFlowAlgorithm		* m_scene_flow_algorithm;

	RGBDDevice				* m_rgbd_device;

	Camera					* m_camera;

	const uint16_t	 		* m_last_depth_image;
};

#endif