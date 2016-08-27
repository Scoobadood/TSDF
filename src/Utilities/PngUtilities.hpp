//
//  PngUtilities.hpp
//  BilateralFiltering
//
//  Created by Dave on 30/04/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#ifndef PngUtilities_hpp
#define PngUtilities_hpp

#include <iostream>
#include <Eigen/Dense>

uint16_t * load_png_from_file( const std::string file_name, uint32_t & width, uint32_t & height );
uint8_t * load_colour_png_from_file( const std::string file_name, uint32_t & width, uint32_t & height );

bool save_png_to_file( const std::string file_name, uint32_t width, uint32_t height, const uint16_t * pixel_data );
bool save_png_to_file( const std::string file_name, uint32_t width, uint32_t height, const uint8_t * pixel_data );
bool save_colour_png_to_file( const std::string file_name, uint32_t width, uint32_t height, const uint8_t * pixel_data );

#endif /* PngUtilities_hpp */
