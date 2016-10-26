#include "TSDFVolume.hpp"
#include "DataLoader/DepthImage.hpp"
#include "DataLoader/srsf_sceneflow_loader.hpp"
#include "Utilities/PngWrapper.hpp"

#include <sstream>
#include <string>
#include <iomanip>

// Image names. Change to read from args later
const std::string BASE_COLOR_FILE_NAME = "/home/dave/Projects/Code/Datasets/BerkeleyMHAD/Kinect/Kin01/S01/A05/R03/kin_k01_s01_a05_r03_color_";
const std::string BASE_DEPTH_FILE_NAME = "/home/dave/Projects/Code/Datasets/BerkeleyMHAD/Kinect/Kin01/S01/A05/R03/kin_k01_s01_a05_r03_depth_";
const std::string BASE_SCFLW_FILE_NAME = "/home/dave/Projects/Code/Datasets/BerkeleyMHAD/SceneFlow/sflow_";

/**
 * Compute the next scene flow image
 * Or in this case simply load it
 */
bool computeSceneFlow( const DepthImage& last_depth, 
					   const DepthImage& this_depth,
					   const PngWrapper& last_colour,
					   const PngWrapper& this_coolour,
					   Eigen::Vector3f & translation,
					   Eigen::Vector3f & rotation,
 					   Eigen::Matrix<float, 3, Eigen::Dynamic>& residuals ) {

	static int frame_counter = 0;

	bool ok = false;

	// Attempt to load scene flow file from disk for now
	// This is an XML file containing a global rigid transformation and rotation along with non-rigid residuals
 	std::ostringstream oss;
  	oss << std::setfill('0') << std::setw(5) << frame_counter;
  	std::string frame_str = oss.str();

	std::string scene_flow_file_name = BASE_SCFLW_FILE_NAME + frame_str + ".xml";

	// Extract the data
	ok = read_scene_flow( scene_flow_file_name.c_str(), translation, rotation, residuals );

	return ok;
}

/**
 * Get the next depth and colour images
 */
bool getNextImagePair( DepthImage ** pp_depth_image, PngWrapper ** pp_colour_image) {
	static int frame_counter = 0;

 	std::ostringstream oss;
  	oss << std::setfill('0') << std::setw(5) << frame_counter;
  	std::string frame_str = oss.str();

	std::string depth_image_file_name = BASE_DEPTH_FILE_NAME + frame_str + ".png";
	std::string color_image_file_name = BASE_COLOR_FILE_NAME + frame_str + ".png";

	DepthImage * p_depth_image = new DepthImage( depth_image_file_name );

	bool ok = false;
	if( p_depth_image) {
		PngWrapper * p_colour_image = new PngWrapper( color_image_file_name, PngWrapper::COLOUR );
		if( p_colour_image ) {
			ok = true;

			*pp_depth_image = p_depth_image;
			*pp_colour_image = p_colour_image;

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
	DepthImage * p_depth_image       = nullptr;
	DepthImage * p_last_depth_image  = nullptr;
	PngWrapper * p_colour_image      = nullptr;
	PngWrapper * p_last_colour_image = nullptr;

	bool success = getNextImagePair( &p_depth_image, &p_colour_image );

	if( success) {

     	// Compute the Scene Flow image
     	Eigen::Vector3f translation;
     	Eigen::Vector3f rotation;
     	Eigen::Matrix<float, 3, Eigen::Dynamic> scene_flow;

		success = computeSceneFlow( *p_last_depth_image, *p_depth_image,
			                        *p_last_colour_image, *p_colour_image,
			                        translation,
			                        rotation, 
			                        scene_flow );

		if( success ) {
     		// For each voxel - extract vertices
     		

    	 	// For each vertex, project into scene flow image

	     	// Apply corrections to voxel coords

	     	// Extract deformed mesh from TSDF

     		// merge depth image
		}

		// Release the image pair
		delete p_colour_image;
		delete  p_depth_image;
	} else {
		std::cout << "Problem loading next image pair" << std::endl;
	}
}


int main( int argc, char *argv[] ) {
	// Construct a TSDF
	TSDFVolume *volume = TSDFVolume::make_volume( TSDFVolume::GPU );

	// Load next depth image and project into TSDF

	//Repeat
	processFrame();
	// Until end of sequence	

	// Extract canonical mesh


	delete volume;
	return 0;
}