#include "../include/FileUtilities.hpp"
#include "../include/PngUtilities.hpp"

#include <iostream>
/**
 * Read a PDFlow text file and split it into X, Yand Z planes. Render each as grey scale images @ half resolution
 * so X Y on first ro, Z, all on second row
 */
uint16_t * render_x_y_z_image( uint16_t width, uint16_t height, float min, float max,  Eigen::Matrix<float,3,Eigen::Dynamic>& flow, uint16_t& out_width, uint16_t& out_height ) {
	uint spacing = 20;
	out_width = (width / 2) * 2  + (3 * spacing );
	out_height = (height / 2 ) * 2 + ( 3 * spacing );
    uint x_index = spacing * out_width + spacing;
	uint y_index = spacing * out_width + spacing + (width/2) + spacing;
    uint z_index = (spacing + (height / 2) + spacing) * out_width + spacing;

	float scale = 65535.0f / (max-min);
	// Allocate memory
	uint16_t * out_image = new uint16_t[ out_width * out_height ];
	if( out_image ) {
		memset( (void *) out_image, out_width * out_height * sizeof( uint16_t ), 0 );
		
		int idx = 0;
		for( int yy=0; yy<height; yy+=2 ) {
			for( int xx=0; xx<width; xx+=2 ) {
				idx = (yy * width) + xx;		
				uint16_t  x = static_cast<uint16_t>( round( (flow(0,idx) - min) * scale));
				uint16_t  y = static_cast<uint16_t>( round( (flow(1,idx) - min) * scale));
				uint16_t  z = static_cast<uint16_t>( round( (flow(2,idx) - min) * scale));
				out_image[x_index++] = x;
				out_image[y_index++] = y;
				out_image[z_index++] = z;
			}
			x_index = x_index + (out_width - (width/2) );
			y_index = y_index + (out_width - (width/2) );
			z_index = z_index + (out_width - (width/2) );
		}

    } else {
		std::cerr << "Couldn;t allocate memory for output image" << std::endl;
		exit( -1 );
	}

	return out_image;
}



/**
 * Read the scene flow data from the given file
 * @param fileName The name of XML file
 * @param translation The global translation. Set by this method
 * @param rotation The global translation. Set by this method
 * @param residuals The residual translation per pixel
 * @return true if the data were read correctly
 */
bool read_scene_flow( const std::string & file_name, Eigen::Matrix<float, 3, Eigen::Dynamic>& flow) {
	struct float3 {
		float x;
		float y;
		float z;
	};


	// Structure to handle processing line
	struct SceneFlowData {
		// File name to read
		std::string file_name;


		// Vector of displacements
		std::vector<float3> v;

		// Image dimensions
		int image_width = 0;
		int image_height = 0;

/**
 * Given a string, parse it into the specified number of floats
 * @param string The source string
 * @param numFloats The number of floats to parse
 * @param readValues A pointer into which to store the values of floats read
 * @return true if the number of floats werer read successfully, otherwise false
 */
bool read_floats_from_string( const char * string, uint num_floats, float * read_values) {
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

		// Construct one, stash the parent
		// Also read the last line fo the file to obtain the X,Y pixel dimensions
		SceneFlowData( const std::string& file_name ) {
			this->file_name = file_name;

			// Read last line of file
			std::string last_line;
			if ( read_last_line( file_name, last_line )) {
				// Parse dimensions from first two elements
				float dims[2];
				if ( ! read_floats_from_string( last_line.c_str(), 2, dims ) ) {
					std::cerr << "Error: Problem reading dimensions from PD scene flow file " << std::endl;
				} else {
					image_width = (int)dims[1] + 1;
					image_height = (int)dims[0] + 1;
				}
			}
		}


		void process_line( const std::string& line ) {
			// Line is int x, int y, float sfx, sfy, sfz
			float values[5];

			if ( ! read_floats_from_string( line.c_str(), 5, values ) ) {
				std::cerr << "Error: Problem entries from PD scene flow file " << std::endl;
			} else {
				int px = (int)values[0];
				int py = (int)values[1];
				v.push_back( float3{values[2] * 1000.0f, values[3] * 1000.0f, values[4] * 1000.0f});
			}
		}
	};

	SceneFlowData sfd{ file_name };

	std::function<void( const std::string & )> f = std::bind(&SceneFlowData::process_line, &sfd, std::placeholders::_1 );

	bool read_ok = process_file_by_lines( file_name, f );

	// Work out how many elements we got and transfer them to the Eigen shape we got passed in residuals
	int num_values = sfd.image_width * sfd.image_height;

	flow.resize( 3, num_values);
	for ( size_t i = 0; i < num_values; i++ ) {
		flow(0, i) = sfd.v[i].x;
		flow(1, i) = sfd.v[i].y;
		flow(2, i) = sfd.v[i].z;
	}

	return read_ok;
}

void min_max( float& min, float& max, Eigen::Matrix<float,3,Eigen::Dynamic>& flow ) {
	for( int i=0; i<640*480; i++ ) {
		if( flow(0,i) < min ) min = flow(0,i);
		if( flow(0,i) > max ) max = flow(0,i);
		if( flow(1,i) < min ) min = flow(1,i);
		if( flow(1,i) > max ) max = flow(1,i);
		if( flow(2,i) < min ) min = flow(2,i);
		if( flow(2,i) > max ) max = flow(2,i);
	}
}

int main( int argc, const char * argv[] ) {
	Eigen::Matrix<float, 3, Eigen::Dynamic> flow;
    read_scene_flow( argv[1], flow );

	float flow_min, flow_max;
	min_max( flow_min, flow_max, flow );
	
	uint16_t width,  height;	
	uint16_t * data;
	data = render_x_y_z_image( 640, 480, flow_min, flow_max, flow, width, height );

	if( data ) {
		save_png_to_file( "/home/dave/Desktop/split_pdf.png", width, height, data );
	}

}

