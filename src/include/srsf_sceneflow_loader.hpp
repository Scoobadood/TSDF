#ifndef SRSF_SCENEFLOW_LOADER_HPP		
		
#include <Eigen/Core>		
#include <iostream>		
		
bool read_scene_flow( const std::string & file_name, Eigen::Vector3f& translation, Eigen::Vector3f& rotation, Eigen::Matrix<float, 3, Eigen::Dynamic>& residuals);		
		
#define  SRSF_SCENEFLOW_LOADER_HPP		
		
#endif 