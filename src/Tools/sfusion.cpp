#include "../include/SceneFusion.hpp"
#include "../include/MockKinect.hpp"
#include "../include/PDSFMockSceneFlowAlgorithm.hpp"
#include "../include/ply.hpp"

int main( int argc, const char * argv[] ) {
	if ( argc == 3 ) {
		RGBDDevice * device     = new MockKinect( argv[1] );
		SceneFlowAlgorithm *sfa = new PDSFMockSceneFlowAlgorithm( argv[2] );
		sfa->init();

		SceneFusion * sf = new SceneFusion( sfa, device );
		device->initialise( );
		device->start( );

        // Save to PLY file
        std::vector<int3> triangles;
        std::vector<float3> verts;
        sf->extract_surface( verts, triangles );
	    std::cout << "Writing to PLY" << std::endl;
   	    write_to_ply( "/home/dave/Desktop/mesh.ply", verts, triangles);


	} else {
		std::cerr << "Usage: " << argv[0] << " <rgbd directory> <sceneflow directory>" << std::endl;
	}
}