#include "PDSFMockSceneFlowAlgorithm.hpp"
#include "FileUtilities.hpp"

#include <iostream>

/**
 * Read the scene flow data from the given file
 * @param fileName The name of XML file
 * @param translation The global translation. Set by this method
 * @param rotation The global translation. Set by this method
 * @param residuals The residual translation per pixel
 * @return true if the data were read correctly
 */
bool PDSFMockSceneFlowAlgorithm::read_scene_flow( const std::string & file_name, Eigen::Vector3f& translation, Eigen::Vector3f& rotation, Eigen::Matrix<float, 3, Eigen::Dynamic>& residuals) {
	struct float3 {
			float x;
			float y;
			float z;
		};


	// Structure to handle processing line
	struct SceneFlowData {
		// File name to read
		std::string file_name;

		// Pointer to the pareent lagorithm
		PDSFMockSceneFlowAlgorithm * parent;

		// Vector of displacements
		std::vector<float3> v;

		// Image dimensions
		int image_width = 0;
		int image_height = 0;

		// Construct one, stash the parent 
		// Also read the last line fo the file to obtain the X,Y pixel dimensions
		SceneFlowData( PDSFMockSceneFlowAlgorithm* container, const std::string& file_name ) {
			this->file_name = file_name;
			parent = container;

			// Read last line of file
			std::string last_line;
			if( read_last_line( file_name, last_line )) {
				// Parse dimensions from first two elements
				float dims[2];
				if ( ! parent->read_floats_from_string( last_line.c_str(), 2, dims ) ) {
					std::cerr << "Error: Problem reading dimensions from PD scene flow file " << std::endl;
				} else {
					image_width = (int)dims[0];
					image_height = (int)dims[1];
				}
			}
		}



		void process_line( const std::string& line ) {
			// Line is int x, int y, float sfx, sfy, sfz
			float values[5];

			if ( ! parent->read_floats_from_string( line.c_str(), 5, values ) ) {
				std::cerr << "Error: Problem entries from PD scene flow file " << std::endl;
			} else {
				int px = (int)values[0];
				int py = (int)values[1];
				v.push_back( float3{values[2], values[3], values[4]});
			}
		}
	};

	SceneFlowData sfd{ this, file_name };
	
	std::function<void( const std::string & )> f = std::bind(&SceneFlowData::process_line, sfd, std::placeholders::_1 );

	bool read_ok = process_file_by_lines( file_name, f );

	// Work out how many elements we got and transfer them to the Eigen shape we got passed in residuals
	int num_values = sfd.image_width * sfd.image_height;

	residuals.resize( 3, num_values);
	for ( size_t i = 0; i < num_values; i++ ) {
		residuals(0, i) = sfd.v[i].x;
		residuals(1, i) = sfd.v[i].y;
		residuals(2, i) = sfd.v[i].z;
	}

	// Set default global rotation and translation
	translation[0] = 0.0f;
	translation[1] = 0.0;
	translation[2] = 0.0;
	rotation[0] = 0.0;
	rotation[1] = 0.0;
	rotation[2] = 0.0;

	return read_ok;
}

bool PDSFMockSceneFlowAlgorithm::is_matched( const std::string& name ) {
	// Files look like sflow_00356_representation01.png
	bool is_matched = match_file_name( "sflow_", 5, "_results01", "txt", name );

	return is_matched;
}
