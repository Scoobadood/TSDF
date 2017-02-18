/**
 * This utility computes the incremental camera pose between the given TSDF and depth images
 */

#include "ICP_CUDA/ICPOdometry.h"
#include <sophus/se3.hpp>
#include <iostream>

#include "../include/DepthImage.hpp"
#include "../include/TSDFVolume.hpp"
#include "../include/GPURaycaster.hpp"

const int IMAGE_WIDTH = 640;
const int IMAGE_HEIGHT = 480;
const float K_CENTRE_X = 331.0f;
const float K_CENTRE_Y = 234.6f;
const float K_FOCAL_X =  591.1f;
const float K_FOCAL_Y =  590.1f;

typedef struct {
	bool			update_tsdf;
	uint16_t        num_threads;
	uint16_t 		num_blocks;
	std::string		tsdf_file_name;
	std::string		depth_file_name;
} t_arguments;

void usage( ) {
	std::cout <<  
    "-u      : If present update the TSDF, else don't"	<< std::endl << 
	"-d <depth_file_name> : The name of the depth file to use" << std::endl <<
	"-v <colume_file_name> : The name of the volme (TSDF) file to use"<< std::endl <<
	"-b <num_blocks> : The number of blocks to use for updates [96]" << std::endl <<
	"-t <num threads>:	The number of threads o use." << std::endl;
}
/**
 * Extract the TSDF file name, depth file name and other arguments
 * Args are expected to be in the following format
 * @param argc The number of arguments provided
 * @param argv the arguments
 *
 * @return true if the argumentd were a valid set of arguments, otherwise false.
 */
bool parse_arguments( int argc, const char * const argv[], t_arguments& arguments ) {

	arguments.tsdf_file_name ="";
	arguments.depth_file_name = "";
    arguments.update_tsdf =  false;
    arguments.num_threads = 224;
    arguments.num_blocks = 96;
	int arg_index = 1;
	while( arg_index < argc ) {
		if( argv[arg_index][0] == '-' && argv[arg_index][2] == 0  ) {
			char c = argv[arg_index][1];
			switch( c ) {
				case 'v':
					if( arg_index < argc - 1 ) {
						arguments.tsdf_file_name = argv[++arg_index];
					} else {
						usage();
						return false;
					}
					break;

				case 'd':
					if( arg_index < argc - 1 ) {
						arguments.depth_file_name = argv[++arg_index];
					} else {
						usage();
						return false;
					}
					break;

				case 'u':
					arguments.update_tsdf = true;
					break;

				case 'b':
					if( arg_index < argc - 1 ) {
						arguments.num_blocks = atoi( argv[++arg_index] );
					} else {
						usage();
						return false;
					}
					break;
	
				case 't':
					if( arg_index < argc - 1 ) {
						arguments.num_threads = atoi( argv[ ++arg_index] );	
					} else {
						usage();
						return false;
					}
					break;

				default:
					usage();
					return false;
			}
		} else {
			usage( );
			return false;
		}
		arg_index++;
	}

	if( arguments.num_threads <= 0 || arguments.num_blocks <= 0 || arguments.tsdf_file_name == ""|| arguments.depth_file_name == "") {
		usage();
		return false;
	} else {
		return true;
	}
}

int main( int argc, char * argv[] ) {

	// Parse arguments
	t_arguments arguments;
	bool args_ok = parse_arguments( argc, argv, arguments );
	if( !args_ok ) exit(-1);

	float distThresh = 0.10f;
    float angleThresh = sinf(20.f * 3.141592654f / 180.0f);
	ICPOdometry icp ( IMAGE_WIDTH, IMAGE_HEIGHT, K_CENTRE_X, K_CENTRE_Y, K_FOCAL_X, K_FOCAL_Y, distThresh, angleThresh );


	// Load a depth image
	DepthImage di1{ arguments.depth_file_name };
	uint16_t * mesh_image = (uint16_t *)di1.data();;


	// Load the TSDF
	TSDFVolume volume{ arguments.tsdf_file_name };
	GPURaycaster * raycaster = new GPURaycaster( );

	Camera * camera = Camera::default_depth_camera();

	// Get the current global transform and rotation and invert for camera pose
	float3 grot = volume.global_rotation();
	float3 gtran= volume.global_translation();	// This is a set of rotations Z Y X

	// Compute the rotation matrix
	Eigen::Matrix4f pose;
	float c1 = cos( grot.x );
	float c2 = cos( grot.y );
	float c3 = cos( grot.z );
	float s1 = sin( grot.x );
	float s2 = sin( grot.y );
	float s3 = sin( grot.z ); 
	pose(0,0) = (c2 * c3);
	pose(0,1) = - (c2 * s3 );
	pose(0,2) = s2;
	pose(0,3) = gtran.x;

	pose(1,0) = (c1*s3 + s1*s2*c3);
	pose(1,1) = (c1*c3-s1*s2*s3);
	pose(1,2) = - (s1 * c2 );
	pose(1,3) = gtran.y;

	pose(2,0) = (s1*s3 - c1*s2*c3);
	pose(2,1) = (s1*c3 + c1*s2*s3);
	pose(2,2) = (c1 * c2 );
	pose(2,3) = gtran.z;

	pose(3,0) = 0;
	pose(3,1) = 0;
	pose(3,2) = 0;
	pose(3,3) = 1;

	// Now invert this to get a camera pose
	pose = pose.inverse().eval();
	camera->set_pose( pose );

	DepthImage * di2 = raycaster->render_to_depth_image( volume, *camera );
	uint16_t * depth_image = (uint16_t *)di2->data();

	// Call ICP and get an estimate
	icp.initICPModel( mesh_image );
	icp.initICP( depth_image );

	Sophus::SE3d mesh_to_depth_transform;
	icp.getIncrementalTransformation( mesh_to_depth_transform, arguments.num_threads, arguments.num_blocks);

	// Tidy up
	delete raycaster;
	delete di2;

	// Output to stdout
	pose = mesh_to_depth_transform.cast<float>().matrix();
	Eigen::Vector3f trans = pose.topRightCorner(3, 1);
    Eigen::Matrix3f rot = pose.topLeftCorner(3, 3);

	std::cout << "trans : " << trans << std::endl;
	std::cout << "  rot : " << rot << std::endl;

	// Optionally update the TSDF

}
