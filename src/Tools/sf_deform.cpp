#include "TSDFVolume.hpp"


int main( int argc, char *argv[] ) {
	// Construct a TSDF
	TSDFVolume *volume = make_volume( TSDFVolume::GPU );


	// Load next depth image and project into TSDF

	//Repeat
     	// Load Scene Flow image
     	// For each voxel - extract vertices
     	// For each vertex, project into scene flow image
     	// Apply corrections to voxel coords
     	// Extract deformed mesh from TSDF
     	// merge depth image
	// Until end of sequence	

	// Extract canonical mesh


	delete TSDFVolume;
	return 0;

}