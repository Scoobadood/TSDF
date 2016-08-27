#include "TSDFVolume.hpp"
#include "DataLoader/DepthImage.hpp"
#include "Utilities/PngWrapper.hpp"

#include <sstream>
#include <string>
#include <iomanip>

// Image names. Change to read from args later
const std::string BASE_COLOR_FILE_NAME = "/home/dave/Projects/Code/Datasets/BerkeleyMHAD/Kinect/Kin01/S01/A05/R03/kin_k01_s01_a05_r03_color_";
const std::string BASE_DEPTH_FILE_NAME = "/home/dave/Projects/Code/Datasets/BerkeleyMHAD/Kinect/Kin01/S01/A05/R03/kin_k01_s01_a05_r03_depth_";

/**
 * Compute the next scene flow image
 * Or in this case simply load it
 */
void computeSceneFlow( ) {
}

/**
 * Get the next depth and colour images
 */
bool getNextImagePair( DepthImage * p_depth_image, PngWrapper * p_colour_image) {
	static int frame_counter = 0;

 	std::ostringstream oss;
  	oss << std::setfill('0') << std::setw(5) << frame_counter;
  	std::string frame_str = oss.str();

	std::string depth_image_file_name = BASE_DEPTH_FILE_NAME + frame_str + ".png";
	std::string color_image_file_name = BASE_COLOR_FILE_NAME + frame_str + ".png";

	p_depth_image = new DepthImage( depth_image_file_name );

	bool ok = false;
	if( p_depth_image) {
		p_colour_image = new PngWrapper( color_image_file_name, PngWrapper::COLOUR );
		if( p_colour_image ) {
			ok = true;

			frame_counter++;
		} else {
			delete p_depth_image;
		}
	}

	return ok;
}

/**
 * Process a frame at a time
 */
void processFrame( ) {
	DepthImage * p_depth_image;
	PngWrapper * p_colour_image;

	int success = getNextImagePair( p_depth_image, p_colour_image );

     	// Compute the Scene Flow image
	computeSceneFlow( );

     	// For each voxel - extract vertices
     	// For each vertex, project into scene flow image
     	// Apply corrections to voxel coords
     	// Extract deformed mesh from TSDF
     	// merge depth image

	// Release the image pair
	delete p_depth_image;
	delete p_colour_image;
}


int main( int argc, char *argv[] ) {
	// Construct a TSDF
	phd::TSDFVolume *volume = phd::TSDFVolume::make_volume( phd::TSDFVolume::GPU );


	// Load next depth image and project into TSDF

	//Repeat
	processFrame();
	// Until end of sequence	

	// Extract canonical mesh


	delete volume;
	return 0;
}