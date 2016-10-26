#ifndef SCENE_FLOW_ALGORITHM_HPP
#define SCENE_FLOW_ALGORITHM_HPP


#include "DepthImage.hpp"
#include "PngWrapper.hpp"

#include <Eigen/Core>


class SceneFlowAlgorithm {
public:
	/**
	 * Compute the scene flow from previous and current colour and depth images
	 */
	virtual void compute_scene_flow(const DepthImage * 						 p_depth_image, 
									const PngWrapper * 						 p_colour_image,
						   			Eigen::Vector3f&   						 translation, 
						   			Eigen::Vector3f&   						 rotation,
						   			Eigen::Matrix<float, 3, Eigen::Dynamic>& residuals ) = 0;
protected:
	SceneFlowAlgorithm( ){};

};


#endif