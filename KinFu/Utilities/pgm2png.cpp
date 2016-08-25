//
//  PgmToPng.hpp
//
//  Created by Dave on 25/08/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include "DepthMapUtilities.hpp"
#include "PngUtilities.hpp"

#include <iostream>

int main( int argc, char * argv[] ) {
	// Load PGM
	const std::string file_name{ argv[2] };
	uint16_t *data = read_nyu_depth_map( file_name, width, height );

	size_t dot = file_name.find_last_of(".");

	std::string out_file_name;
	if (dot != std::string::npos) {
        out_file_name = path.substr(0, dot);
    }
    out_file_name = out_file_name + ".png";

	// Save as PNG
	save_png_to_file( out_file_name, width, height, data );
}
