#include "../include/TUMDataLoader.hpp"
#include "../include/FileUtilities.hpp"

#include <sys/stat.h>
#include <stdexcept>
#include <vector>
#include <fstream>
#include <Eigen/Dense>
#include <iostream>


TUMDataLoader::TUMDataLoader( const std::string& directory ) {
	// Check directory exists
	bool is_directory = false;
	if ( file_exists( directory, is_directory) && is_directory) {

		m_directory_name = directory;

		// We expect to find a file called ground_truth.txt in this directory
		std::string gt_file_name = directory + "/ground_truth.txt";
		if ( file_exists( gt_file_name, is_directory) && !is_directory ) {
			load_data_from( gt_file_name );
		} else {
			throw std::invalid_argument( "Ground truth file not found " + gt_file_name);
		}
	} else {
		throw std::invalid_argument( "Directory not found " + directory);
	}
}

	/**
	 */
	TUMDataLoader::~TUMDataLoader( ) {
		std::cout << "TUMDataLoader::destructor" << std::endl;
	}


/**
 * Create a pose matrix based on
 * 7 float parameters (as provided by TUM groundtruth data)
 * qx qy qz qw (4 floats) give the orientation of the optical center of the color camera 
 * in form of a unit quaternion with respect to the world origin as defined by the 
 * motion capture system.
 * We assume this is a left handed coordinate system and +ve Z is into the scene
 * @param vars 0, 1 and 2 are a translation, 3,4,5,6 are x,y,z, w components of a quaternion describing the facing
 */
Eigen::Matrix4f TUMDataLoader::to_pose( float vars[7] ) const {
    using namespace Eigen;

    float w = vars[6];
    float x = vars[3];
    float y = vars[4];
    float z = vars[5];

    // Convert the implicit quaternion in vars 3-6 into a RH rotation matrix
    Matrix4f pose = Matrix4f::Zero();
    pose(0,0) = 1 - 2 * ( y*y + z*z );
    pose(0,1) = 2 * ( x*y + w*z);
    pose(0,2) = 2 * ( x*z - w*y);

    pose(1,0) = 2 * ( x*y - w*z );
    pose(1,1) = 1 - 2 * ( x*x + z*z );
    pose(1,2) = 2 * ( y*z + w*x );

    pose(2,0) = 2 * ( x*z + w*y );
    pose(2,1) = 2 * ( y*z - w*x );
    pose(2,2) = 1 - 2 * ( x*x + y*y );



    // Location
    pose( 0, 3) = vars[0] * 1000.0f;
    pose( 1, 3) = vars[1] * 1000.0f;
    pose( 2, 3) = vars[2] * 1000.0f;
    pose(3,3) = 1.0f;

    return pose;
}



/**
 * @param pose The camera pose, populated by the next method
 * @return The next DepthImage to be read or nullptr if none
 */
DepthImage * TUMDataLoader::next( Eigen::Matrix4f& pose )  {
	DepthImage * image = nullptr;

	if ( m_current_idx < m_data_records.size() ) {

		struct DATA_RECORD dr = m_data_records[m_current_idx];
		bool is_directory = false;


		if ( file_exists( dr.file_name, is_directory ) && !is_directory ) {
			image = new DepthImage( dr.file_name );
			pose = to_pose( dr.data );
		} else {
			std::cerr << "Couldn't find file " << dr.file_name << std::endl;
		}

		m_current_idx ++;

	}
	return image;
}


void TUMDataLoader::process_line( const std::string& line ) {
	if ( line.size() > 0 && line[0] != '#' ) {

		std::stringstream iss(line);
		struct DATA_RECORD dr;

		std::string base_file_name;
		iss >> base_file_name;

		dr.file_name = m_directory_name + "/depth/" + base_file_name + ".png";

		for ( int i = 0; i < 7; i++ ) {
			iss >> dr.data[i];
		}

		m_data_records.push_back( dr );
	}
}

void TUMDataLoader::load_data_from( const std::string& gt_file_name ) {

	std::function<void( const std::string & )> f = std::bind(&TUMDataLoader::process_line, this, std::placeholders::_1 );

	bool is_ok = process_file_by_lines( gt_file_name, f );
	if ( !is_ok ) {
		throw std::runtime_error( "Failed to parse the ground truth file" );
	} else {
		m_current_idx = 0;
	}
}