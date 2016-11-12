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


	struct ss {

		std::string file_name;
		PDSFMockSceneFlowAlgorithm * parent;

		ss( PDSFMockSceneFlowAlgorithm* container, const std::string& file_name ) {
			this->file_name = file_name;
			parent = container;
		}

		std::vector<float3> v;


		void process_line( const std::string& line ) {
			// Line is int x, int y, float sfx, sfy, sfz
			float values[5];

			if ( ! parent->read_floats_from_string( file_name.c_str(), 5, values ) ) {
				std::cerr << "Error: Problem entries from PD scene flow file " << std::endl;
			} else {
				int px = (int)values[0];
				int py = (int)values[1];
				v.push_back( float3{values[2], values[3], values[4]});
			}
		}
	};

	ss s{ this, file_name };
	
	std::function<void( const std::string & )> f = std::bind(&ss::process_line, s, std::placeholders::_1 );

	bool read_ok = process_file_by_lines( file_name, f );

	return read_ok;
}

bool PDSFMockSceneFlowAlgorithm::is_matched( const std::string& name ) {
	// Files look like sflow_00356_representation01.png
	bool is_matched = match_file_name( "sflow_", 5, "_results01", "txt", name );

	return is_matched;
}
