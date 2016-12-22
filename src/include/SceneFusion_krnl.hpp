#ifndef SCENE_FUSION_KRNL_HPP
#define SCENE_FUSION_KRNL_HPP

#include "DepthImage.hpp"
#include "PngWrapper.hpp"

__host__
void process_frames(	TSDFVolume *			volume,
						const Camera * const 	camera,
						const uint16_t			width,
						const uint16_t 			height,
						const uint16_t * const 	h_depth_data,
						const float3 * const 	h_scene_flow );
#endif