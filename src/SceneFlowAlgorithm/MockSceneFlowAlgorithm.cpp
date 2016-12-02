#include "MockSceneFlowAlgorithm.hpp"
#include "FileUtilities.hpp"

#include <iostream>

MockSceneFlowAlgorithm::MockSceneFlowAlgorithm( const std::string & scene_flow_directory_name ) {
	// Check directory exists
	bool is_directory;
	if ( file_exists( scene_flow_directory_name, is_directory) && is_directory) {
		// Stash this
		m_directory = scene_flow_directory_name;

		// And set index to first entry
		m_current_file_index = 0;

	} else {
		std::cerr << "Couldn't find directory " << scene_flow_directory_name << std::endl;
	}
}

bool MockSceneFlowAlgorithm::init( ) {
		// And read and sort file names
		// Count the mnumber of scene flow files of the form sflow_nnnn.xml
		files_in_directory( m_directory, m_scene_flow_file_names, [&]( std::string name ) {
			return is_matched( name );
		});

		// Sort them
		std::sort( m_scene_flow_file_names.begin(), m_scene_flow_file_names.end() );

		return true;
}

/**
 * Given a string, parse it into the specified number of floats
 * @param string The source string
 * @param numFloats The number of floats to parse
 * @param readValues A pointer into which to store the values of floats read
 * @return true if the number of floats werer read successfully, otherwise false
 */
bool MockSceneFlowAlgorithm::read_floats_from_string( const char * string, uint num_floats, float * read_values) {
	bool read_ok = true;

	if ( !read_values ) {
		std::cerr << "Warning: read_values is null in read_floats_from_string. Values will be counted but not returned" << std::endl;
	}

	if ( num_floats == 0 ) {
		std::cerr << "Warning: num_floats is 0 in read_floats_from_string. This is valid but probably an error" << std::endl;
		return true;
	}


	if ( string ) {
		std::istringstream iss( string );
		double d ;
		for ( size_t i = 0; i < num_floats; i++ ) {
			if ( !(iss >> d) ) {
				// Clear fail flag
				iss.clear( std::ios::goodbit) ;
				std::string possibleNan(3,'_');
				iss.read( &possibleNan[0], 3);
				if( possibleNan == "Nan") {
					d = std::numeric_limits<double>::quiet_NaN();
				} else {

					std::cerr << "Problem reading floats from string. Expected " << num_floats  << "failed at "<< i << std::endl;
					std::cerr << "Error: " << strerror(errno) << std::endl;
					read_ok = false;

					// Backtrack through the array to find last non-zero value read
					while( read_values[--i] == 0.0f );
					std::cout << "Last good non-zero value read was at index " << i << "and was " << read_values[i] << std::endl;

					break;
				}
			}
			read_values[i] = d;
		}
	} else {
		std::cerr << "String is null in readFloatsFromString" << std::endl;
		read_ok = false;
	}
	return read_ok;
}



/**
 * Compute the scene flow from previous and current colour and depth images
 */
void MockSceneFlowAlgorithm::compute_scene_flow(	
		const DepthImage 						* pDepthImage,
        const PngWrapper 						* pColourImage,
        Eigen::Vector3f&   						translation,
        Eigen::Vector3f&   						rotation,
        Eigen::Matrix<float, 3, Eigen::Dynamic>& residuals ) {

	if ( m_current_file_index < m_scene_flow_file_names.size() ) {
		std::string path_to_file = m_directory + "/" + m_scene_flow_file_names[m_current_file_index];
		// Read the file
		if (!read_scene_flow( path_to_file, translation, rotation, residuals ) ) {
			std::cerr << "Failed to read scene flow from file " << path_to_file << std::endl;
		} else {
			m_current_file_index ++;
		}
	} else {
		std::cerr << "Tried to read scene flow file that doesn't exist" << std::endl;
	}
}