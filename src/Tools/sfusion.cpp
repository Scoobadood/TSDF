#include "../include/SceneFusion.hpp"
#include "../include/MockKinect.hpp"
#include "../include/SRSFMockSceneFlowAlgorithm.hpp"

int main( int argc, const char * argv[] ) {
	if ( argc == 3 ) {
		RGBDDevice * device     = new MockKinect( argv[1] );
		SceneFlowAlgorithm *sfa = new SRSFMockSceneFlowAlgorithm( argv[2] );

		SceneFusion * sf = new SceneFusion( sfa, device );
		device->initialise( );
		device->start( );
	} else {
		std::cerr << "Usage: " << argv[0] << " <rgbd directory> <sceneflow directory>" << std::endl;
	}
}