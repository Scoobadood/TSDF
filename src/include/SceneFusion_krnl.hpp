#ifndef SCENE_FUSION_KRNL_HPP
#define SCENE_FUSION_KRNL_HPP

#include "DepthImage.hpp"
#include "PngWrapper.hpp"

__host__
void process_frames( 
	TSDFVolume 		* volume,
	const Camera 	* camera,
	uint16_t 		width, 
	uint16_t 		height, 
	const uint16_t 	* depth_data,
	const float 	* scene_flow_data ) ;
#endif