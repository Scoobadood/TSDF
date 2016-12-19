// Convert freenect output PGM image into an equivalent PNG
// Things to know:
// 1. Depth values are stored in LSB format by freenect-record
// 2. 2048 is an invalid value - map to 0
// 3. Values 0 - 2047 need to be mapped based on the alogorithm below
//    Depth in mm = 1000 / (raw * -0.0030711016 + 3.3309495161)

#include <iostream>
#include <iomanip>
#include <cstdint>
#include <cmath>

#include "../include/PngUtilities.hpp"

#include "pam.h"


float g_depth[2049];


typedef struct { 
	float x, y, z;
} Vec3f;

typedef struct {
	int16_t x, y;
} Vec2i;

typedef struct { 
	float x, y, z, w;
} Vec4f;

typedef struct {
	float m11, m12, m13;
	float m21, m22, m23;
	float m31, m32, m33;
} Mat33;

typedef struct {
	float m11, m12, m13, m14;
	float m21, m22, m23, m24;
	float m31, m32, m33, m34;
	float m41, m42, m43, m44;
} Mat44;


/**
 * INitialise the depth table
 */
 void init_depth_table( ) {
 	for( int i=0; i<2048; i++ ) {
 		float depth_in_mm = 1000.0f / ( ( i * -0.0030711016) + 3.3309495161 );
 		g_depth[i] = depth_in_mm;
 	}
 }


// Some matrices
static const Mat33 rgb_cam_k {	521.179133f,   0.0f,      322.515987f,
								0.0f,        493.033034f, 259.055966f,
	 							0.0f,          0.0f,        1.0f};

static const Mat33 rgb_cam_k_inv {	0.001889, 0.0f,			-0.62156, 
									0.0f,		 0.00190271,	-0.5089,
									0.0f, 		 0.0f, 			1.0f};

static const Mat33  ir_cam_k {594.214,  	0.0,			339.307, 
								0.0f, 		591.0405,		242.739, 
								0.0f,		 0.0, 			1.0f };

static const Mat44 rgb_to_ir_pose{ 0.999846, -0.001478, 0.01747,-0.01979,
									 0.001263, 0.999924,  0.122753, -0.000852,
									-0.017487, -0.0122513, 0.999772, 0.011254,
									0,0,0,1};
									

Vec2i rgb_to_ir( int16_t x, int16_t y ) {
	// Push rgb pixel to cam space
	float rgb_cam_x = x * rgb_cam_k_inv.m11 + y * rgb_cam_k_inv.m12 + rgb_cam_k_inv.m13;
	float rgb_cam_y = x * rgb_cam_k_inv.m21 + y * rgb_cam_k_inv.m22 + rgb_cam_k_inv.m23;
	float rgb_cam_z = x * rgb_cam_k_inv.m31 + y * rgb_cam_k_inv.m32 + rgb_cam_k_inv.m33;

	// Map pose to ir cam space
	float ir_cam_x = rgb_to_ir_pose.m11 * rgb_cam_x + rgb_to_ir_pose.m12 * rgb_cam_y + rgb_to_ir_pose.m13 * rgb_cam_z + rgb_to_ir_pose.m14;
	float ir_cam_y = rgb_to_ir_pose.m21 * rgb_cam_x + rgb_to_ir_pose.m22 * rgb_cam_y + rgb_to_ir_pose.m23 * rgb_cam_z + rgb_to_ir_pose.m24;
	float ir_cam_z = rgb_to_ir_pose.m31 * rgb_cam_x + rgb_to_ir_pose.m32 * rgb_cam_y + rgb_to_ir_pose.m33 * rgb_cam_z + rgb_to_ir_pose.m34;
	float w = rgb_to_ir_pose.m41 * rgb_cam_x + rgb_to_ir_pose.m42 * rgb_cam_y + rgb_to_ir_pose.m43 * rgb_cam_z + rgb_to_ir_pose.m44;

	ir_cam_x /= w;
	ir_cam_y /= w;
	ir_cam_z /= w;

	// Push ir cam into pixel
	w = ir_cam_x * ir_cam_k.m31 + ir_cam_y * ir_cam_k.m32 + ir_cam_z * ir_cam_k.m33;
	float ir_pix_x = roundf( (ir_cam_x * ir_cam_k.m11 + ir_cam_y * ir_cam_k.m12 + ir_cam_z * ir_cam_k.m13) / w );
	float ir_pix_y = roundf( (ir_cam_x * ir_cam_k.m21 + ir_cam_y * ir_cam_k.m22 + ir_cam_z * ir_cam_k.m23) / w );



	return Vec2i {
		static_cast<int16_t>( ir_pix_x ),
		static_cast<int16_t>( ir_pix_y )
	};
}


 /**
  * Process the image data
  * For each pxel in the putput file (which corresponds to the RB image)
  * 1. Convert to a pixel in depth cam space
  * 2. Grab the depth value (if it's in range)
  * 3. Convert to a depth value in mm
  * Then save as PNG
  */
 void register_depth_to_rgb( uint16_t * const output_data, const uint16_t * const depth_data, uint16_t width, uint16_t height ) {
 	size_t rgb_index = 0;
 	for( uint16_t rgb_y=0; rgb_y <height; rgb_y ++ ) {
 		for( uint16_t rgb_x = 0; rgb_x < width; rgb_x ++ ) {

 			uint16_t depth = 0;
 			Vec2i ir_pixel = rgb_to_ir( rgb_x, rgb_y );

 			if( ir_pixel.x >= 0 && ir_pixel.y >= 0 && ir_pixel.x < width && ir_pixel.y < height ) {
 				// inside of depth map image => compute actual depth
 				// For now just copy value
 				depth = depth_data[ ir_pixel.y * width + ir_pixel.x];
 			}

 			output_data[ rgb_index++ ] = depth;
 		}
 	}
 }


 uint16_t * load_file( const char * file_name, uint16_t& width, uint16_t& height ) {
 	uint16_t * data = nullptr;

	FILE * input_file = pm_openr( file_name );
	if( input_file ) {

		// Get header
		struct pam inpam;
		pnm_readpaminit(input_file, &inpam, sizeof( struct pam ));

		if( inpam.format = PGM_FORMAT)  {
			// Read the thing
			size_t num_pixels = inpam.width * inpam.height;
			data = new uint16_t[ num_pixels ];
			if( data ) {

				size_t pixels_read = fread( data, sizeof( uint16_t ), num_pixels, input_file);
				if( pixels_read == num_pixels) {
					width = inpam.width;
					height = inpam.height;
				} else {
					std::cerr << "Incomplete data. only read " << pixels_read << " values "<< std::endl;
					delete[] data;
					data = nullptr;
				}
			} else {
				std::cerr << "Couldn't allocate memory for PGM file" << std::endl;
			}
		} else {
			std::cerr << "Not PGM_FORMAT (" << char(inpam.format>>8) << char(inpam.format &0xff) << ")" << std::endl;
		}
		fclose( input_file );
	} else {
		std::cerr << "Couldn't read from file " << file_name << std::endl;
	}

	return data;
 }


 int main( int argc, char *argv[] ) {
 	// Required init
	pm_init(argv[0], 0);

	uint16_t * data = nullptr;
	uint16_t * out_data = nullptr;
	uint16_t width = 0, height = 0;

	if( argc == 2 ) {
		data = load_file( argv[1], width, height );
		std::cout << "Read image " << width <<"x"<< height << std::endl;

		out_data = new uint16_t[ width * height];
		if( out_data) {
			register_depth_to_rgb( out_data, data, width, height);

			save_png_to_file( "/home/dave/Desktop/depth.png", width, height,  out_data);
		} else {
			std::cerr << "couldn't allocate output buffer" << std::endl;
		}

	} else {
		std::cerr << "Must specify PGM file" << std::endl;
	}

	if( data ) delete[] data;
	if( out_data ) delete[] out_data;
}
