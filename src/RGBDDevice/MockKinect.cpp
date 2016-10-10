#include "../include/MockKinect.hpp"
#include "../include/FileUtilities.hpp"

#include <iostream>
#include <algorithm>
#include <regex>

/**
 * @param directory Place from which to load colour and depth images
 */
MockKinect::MockKinect( const std::string directory ) {
	m_directory = directory;
}

/**
 * Initialise the device by confirming that the directory exists and counting the number of entries in it
 * We look for color_nnnnnn.pgm and depth_nnnnnn.ppm files
 */
void MockKinect::initialise( ) {

	std::cout<< "Initialising MockKinect" << std::endl;

	// Does directory exist
	bool exists;
	bool is_directory;
	exists = file_exists( m_directory, is_directory );

	if ( exists && is_directory ) {
		m_colour_file_names.clear();
		m_depth_file_names.clear();

		// Enumerate colour files
		files_in_directory( m_directory, m_colour_file_names, []( std::string name ) {
			// If name is color_nnnnn.pgm or depth.ppm then we accept it
			bool is_valid = false;
			is_valid = std::regex_match( name, std::regex("color_\\d{4}.png") );
			return is_valid;
		});

		// Enumerate depth files
		files_in_directory( m_directory, m_depth_file_names, []( std::string name ) {
			// If name is color_nnnnn.pgm or depth.ppm then we accept it
			bool is_valid = false;
			is_valid = std::regex_match( name, std::regex("depth_\\d{4}.png")  );
			return is_valid;
		});

		// Sort both
		std::sort( m_colour_file_names.begin(), m_colour_file_names.end() );
		std::sort( m_depth_file_names.begin(), m_depth_file_names.end() );
	} else {
		std::cerr << "Directory not found " << m_directory << std::endl;
	}
}

/**
 * Start to produce image pairs to call back
 */
void MockKinect::start( ) {
	// Iterate over all of the colur and depth images, invoking the observer
	size_t num_image_pairs = std::min( m_colour_file_names.size() , m_depth_file_names.size() );
	std::cerr << "Processing " << num_image_pairs << " pairs" << std::endl;
	for ( int i = 0; i < num_image_pairs; i++ ) {
		std::string colour_file_name = m_colour_file_names[i];
		std::string depth_file_name = m_depth_file_names[i];

		// Affirm that these have the same numbers
		int colour_file_index = stoi( colour_file_name.substr( 6, 4 ) );
		int depth_file_index = stoi( depth_file_name.substr( 6, 4 ) );

		if ( colour_file_index == depth_file_index ) {
			// Construct the PngWrapper
			PngWrapper * colour_image = new PngWrapper( m_directory + "/" + colour_file_name, PngWrapper::COLOUR );
			DepthImage * depth_image = new DepthImage( m_directory + "/" + depth_file_name );

			// Call callback
			std::cerr << "Process pair (" << depth_file_name << ", " << colour_file_name << ")" << std::endl;
			notify( depth_image, colour_image );

			// delete data
			delete colour_image;
			delete depth_image;
		} else {
			std::cerr << "Mismatched files: depth " << depth_file_name << ", colour " << colour_file_name << std::endl;
			break;
		}
	}
};

/**
 * Stop producing image pairs
 */
void MockKinect::stop( ) { };
