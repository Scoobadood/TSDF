// TSDF Visualiser
// Loads block data
// Renders top left and front image

#include <fstream>
#include "../include/PngUtilities.hpp"


typedef enum  {
	TOP,
	RIGHT,
	FRONT
} ImageType;

/**
 * Load the data from a TSDF file
 */
bool load_distance_data( const char * file_name, uint32_t& volume_width, uint32_t& volume_height, uint32_t& volume_depth, int& volume_data_size, float *& volume_data) {
	using namespace std;
	bool loaded_ok = true;

	ifstream ifs{ file_name, ios::in | ios::binary };
	if ( !ifs ) {
		std::cerr << "Problem loading TSDF file " << file_name << std::endl;
		loaded_ok = false;
	}

	// Load dimensions
	uint32_t dimensions[3];
	if ( loaded_ok ) {
		if ( ifs.read( (char *) &dimensions, sizeof( dimensions )) ) {

			if (( dimensions[0] != dimensions[1] ) || ( dimensions[0] != dimensions[2] ) ) {
				std::cerr << "Only works with TSDF with equal dimensions" << std::endl;
				ifs.close();
				loaded_ok = false;
			}

			// if ( dimensions[0] > 256 ) {
			// 	std::cerr << "Maximum dimension is 256" << std::endl;
			// 	ifs.close();
			// 	loaded_ok = false;
			// }

		} else {
			std::cerr << "Problem loading TSDF file " << file_name << ", too short reading dimensions" << std::endl;
			loaded_ok = false;
		}
	}


	// Skip physical dimensions
	if ( loaded_ok ) {

		if ( !ifs.ignore( 3 * sizeof( float ) ) ) {
			std::cerr << "Problem loading TSDF file " << file_name << ", too short skipping physical dimensions" << std::endl;
			loaded_ok = false;
			ifs.close();
		}
	}


	// Allocate memory for volume data
	int distance_data_size;
	float *distance_data;
	if ( loaded_ok ) {

		distance_data_size = dimensions[0] * dimensions[1] * dimensions[2];
		distance_data = new float[distance_data_size];

		if ( !distance_data ) {
			std::cerr << "Couldn't allocate memory for distance data" << std::endl;
			ifs.close( );
			loaded_ok = false;
		}
	}

	// Read volume data
	if ( loaded_ok ) {
		if ( ! ifs.read( (char *)distance_data, distance_data_size * sizeof( float ) ) ) {
			std::cerr << "Problem loading TSDF file " << file_name << ", too short reading data" << std::endl;
			loaded_ok = false;
		}
		ifs.close();
	}

	if ( loaded_ok ) {
		volume_width = dimensions[0];
		volume_height = dimensions[1];
		volume_depth = dimensions[2];
		volume_data	= distance_data;
		volume_data_size = distance_data_size;
	}

	return loaded_ok;
}






bool save_png( const char *file_name,
               ImageType image_type,
               int volume_width,
               int volume_height,
               int volume_depth,
               float * volume_data ) {

	int volume_size = volume_width * volume_depth * volume_height;


	uint32_t tile_width, tile_height, num_tiles;
	switch ( image_type) {
	case TOP:
		num_tiles = volume_height;
		tile_width = volume_width;
		tile_height = volume_depth;
		break;

	case FRONT:
		num_tiles = volume_depth;
		tile_width = volume_width;
		tile_height = volume_height;
		break;

	case RIGHT:
		num_tiles = volume_width;
		tile_width = volume_depth;
		tile_height = volume_height;
		break;

	}




	// Top image is 16x16 256x256 images
	int image_spacing = 10;

	// tiles should be arraned close to a square
	int num_hor_tiles = ceil( sqrt( num_tiles ) );
	int num_ver_tiles = ceil( (float)num_tiles / num_hor_tiles );

	int image_width = tile_width * num_hor_tiles + (image_spacing * (num_hor_tiles + 1));
	int image_height = tile_height * num_ver_tiles + (image_spacing * (num_ver_tiles + 1));

	size_t image_size_pixels = image_height * image_width;
	size_t image_size_bytes = image_size_pixels * 3 * sizeof( uint8_t);

	uint8_t *image_data = new uint8_t[image_size_bytes];
	if ( ! image_data ) {
		std::cerr << "Couldn't allocate memory for top image" << std::endl;
		return false;
	}



	// Find min and max
	float min = std::numeric_limits<float>::max();
	float max = - std::numeric_limits<float>::max();
	for ( int i = 0; i < volume_size; i++) {
		float f = volume_data[i];
		if ( f < min ) min = f;
		if ( f > max ) max = f;
	}

	// Compute scale factor
	float scale_factor = 1.0f / (max - min);

	// Plot the PNG

	// First gray out the PNG
	memset( image_data, 240, image_size_bytes );

	// Iterate over each plane. Dimensions are stored X,Y,Z
	int data_index = 0;
	for ( int z = 0; z < volume_depth; z++ ) {
		for ( int y = 0; y < volume_height; y++ ) {
			for ( int x = 0; x < volume_width; x++ ) {

				// Compute the pixel location for this voxel
				int tile_row;
				int tile_col;
				switch ( image_type) {
				case TOP:
					tile_row = y / num_hor_tiles;
					tile_col = y % num_hor_tiles;
					break;

				case FRONT:
					tile_row = z / num_hor_tiles;
					tile_col = z % num_hor_tiles;
					break;

				case RIGHT:
					tile_row = x / num_hor_tiles;
					tile_col = x % num_hor_tiles;
					break;

				}

				int tile_base_x = tile_col * ( tile_width + image_spacing ) + image_spacing;
				int tile_base_y = tile_row * ( tile_height + image_spacing ) + image_spacing;

				int pixel_x;
				int pixel_y;
				switch ( image_type) {
				case TOP:
					pixel_x = tile_base_x + x;
					pixel_y = tile_base_y + tile_height - z - 1;
					break;

				case FRONT:
					pixel_x = tile_base_x + x;
					pixel_y = tile_base_y + tile_height - y - 1;
					break;

				case RIGHT:
					pixel_x = tile_base_x + z;
					pixel_y = tile_base_y + tile_height - y - 1;
					break;
				}

				int pixel_index = (pixel_y * image_width) + pixel_x;
				int byte_index = pixel_index * 3;

				float value = volume_data[data_index];

				uint8_t blue, green, red;

				if ( value == 0 ) {
					red = green = blue = 160;
				} else {
					blue  = ((value - min) * scale_factor) * 255;
					green = 0;
					red   = 255 - blue;
				}

				image_data[byte_index] = red;
				image_data[byte_index + 1] = green;
				image_data[byte_index + 2] = blue;

				data_index++;
			}
		}
	}

	// Save as PNG
	save_colour_png_to_file( file_name, image_width, image_height, image_data );
	delete[] image_data;
	return true;
}

int main( int argc, char * argv[] ) {
	// Load data
	using namespace std;

	if ( argc != 2 ) {
		std::cout << "Usage: " << argv[0] << " <tsdf_filename>" << std::endl;
		exit( -1 );
	}


	uint32_t v_width, v_height, v_depth;
	int volume_size;
	float * volume_data;
	bool ok = load_distance_data( argv[1], v_width, v_height, v_depth, volume_size, volume_data);


	// Make top image
	if ( ok ) {
		save_png( "top.png", TOP, v_width, v_depth, v_height, volume_data);
		save_png( "right.png", RIGHT, v_width, v_depth, v_height, volume_data);
		save_png( "front.png", FRONT, v_width, v_depth, v_height, volume_data);
		delete[] volume_data;
	}
}