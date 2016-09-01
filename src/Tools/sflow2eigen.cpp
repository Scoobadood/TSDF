/**
 */

#include "../DataLoader/srsf_sceneflow_loader.hpp"

#include <Eigen/Core>
#include <iostream>

int main( int argc, char *argv[] ) {
	if( argc == 2 ) {
		// Load file using TinyXML
		Eigen::Vector3f translation;
		Eigen::Vector3f rotation;
		Eigen::Matrix<float, 3, Eigen::Dynamic> residuals;
		read_scene_flow( argv[1], translation, rotation, residuals);

		// Save the whole thing
	} else {
		std::cerr << "Must specify file to load from" << std::endl;
	}
}

