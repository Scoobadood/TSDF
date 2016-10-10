//
//  PngUtilities.cpp
//  BilateralFiltering
//
//  Created by Dave on 30/04/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include "../include/PngUtilities.hpp"
#include <png.h>


uint16_t * load_png_from_file( const std::string file_name, uint32_t & width, uint32_t & height ) {
    uint16_t * pixel_data = NULL;
    width = 0;
    height = 0;

    png_structp png_ptr = NULL;
    png_infop info_ptr = NULL;
    FILE *fp = std::fopen( file_name.c_str(), "rb");
    if (fp) {

        unsigned char     header[8];
        if ( fread( header, 1, 8, fp) == 8) {

            bool is_png = !png_sig_cmp(header, 0, 8);
            if ( is_png ) {
                // Setup structs for reading data

                png_ptr = png_create_read_struct (PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

                if (png_ptr) {

                    info_ptr = png_create_info_struct(png_ptr);
                    if (info_ptr) {

                        // png_infop end_info = png_create_info_struct(png_ptr);
                        // if (end_info) {

                            png_init_io(png_ptr, fp);

                            // Notify png of missing sig bytes
                            png_set_sig_bytes(png_ptr, 8);

                            // Do the actual read
                            png_read_png(png_ptr, info_ptr, PNG_TRANSFORM_IDENTITY, NULL);


                            // Get data
                            png_bytep *row_pointers = png_get_rows(png_ptr, info_ptr);
                            // Get width and height
                            width = png_get_image_width(png_ptr, info_ptr);
                            height = png_get_image_height(png_ptr, info_ptr);

                            if( png_get_rowbytes(png_ptr, info_ptr) == 2*width ) {

                                pixel_data = new uint16_t [width * height];
                                size_t dst_idx = 0;
                                for( uint32_t y=0; y<height; y++ ) {
                                    size_t src_idx = 0;
                                    for( uint32_t x=0; x<width ; x++ ) {
                                        uint8_t b1 = row_pointers[y][src_idx++];
                                        uint8_t b2 = row_pointers[y][src_idx++];
                                        uint16_t s = b1 * 256 + b2;
                                        pixel_data[dst_idx++] = s;
                                    }
                                }
                            } else {
                                std::cerr << "Expected 16bpp greyscale file" << std::endl;
                            }
                            png_destroy_read_struct(&png_ptr, &info_ptr, NULL);

                        // } else {
                        //     png_destroy_read_struct(&png_ptr, &info_ptr, (png_infopp)NULL);
                        //     std::cerr << "Problem reading file " << file_name << std::endl;
                        // }
                    } else {
                        png_destroy_read_struct(&png_ptr,(png_infopp)NULL, (png_infopp)NULL);
                        std::cerr << "Problem reading file (out of memory)" << file_name << std::endl;
                    }


                } else {
                    std::cerr << "Problem reading file (out of memory)" << file_name << std::endl;
                }
            } else {
                std::cerr << "File " << file_name << "is not a PNG" << std::endl;
            }
        } else {
            std::cerr << "File " << file_name << "is too short" << std::endl;
        }
    } else {
        std::cerr << "Couldn't open file " << file_name << std::endl;
    }

    // Free memory and structures used
    png_free_data( png_ptr, info_ptr, PNG_FREE_ALL, -1);


    if( fp ) fclose(fp);
    return pixel_data;
}

uint8_t * load_colour_png_from_file( const std::string file_name, uint32_t & width, uint32_t & height ) {
    uint8_t * pixel_data = NULL;
    width = 0;
    height = 0;

    png_structp png_ptr = NULL;
    png_infop info_ptr = NULL;
    FILE *fp = std::fopen( file_name.c_str(), "rb");
    if (fp) {

        unsigned char     header[8];
        if ( fread( header, 1, 8, fp) == 8) {

            bool is_png = !png_sig_cmp(header, 0, 8);
            if ( is_png ) {
                // Setup structs for reading data

                png_ptr = png_create_read_struct (PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

                if (png_ptr) {

                    info_ptr = png_create_info_struct(png_ptr);
                    if (info_ptr) {

                        // png_infop end_info = png_create_info_struct(png_ptr);
                        // if (end_info) {

                            png_init_io(png_ptr, fp);

                            // Notify png of missing sig bytes
                            png_set_sig_bytes(png_ptr, 8);

                            // Do the actual read
                            png_read_png(png_ptr, info_ptr, PNG_TRANSFORM_IDENTITY, NULL);


                            // Get data
                            png_bytep *row_pointers = png_get_rows(png_ptr, info_ptr);
                            // Get width and height
                            width = png_get_image_width(png_ptr, info_ptr);
                            height = png_get_image_height(png_ptr, info_ptr);

                            if( png_get_rowbytes(png_ptr, info_ptr) == 3*width ) {

                                pixel_data = new uint8_t [width * height * 3];
                                size_t dst_idx = 0;
                                for( uint32_t y=0; y<height; y++ ) {
                                    size_t src_idx = 0;
                                    for( uint32_t x=0; x<width ; x++ ) {
                                        pixel_data[dst_idx++] = row_pointers[y][src_idx++];
                                        pixel_data[dst_idx++] = row_pointers[y][src_idx++];
                                        pixel_data[dst_idx++] = row_pointers[y][src_idx++];
                                    }
                                }
                            } else {
                                std::cerr << "Expected 8bpp color file" << std::endl;
                            }
                            png_destroy_read_struct(&png_ptr, &info_ptr, NULL);

                        // } else {
                        //     png_destroy_read_struct(&png_ptr, &info_ptr, (png_infopp)NULL);
                        //     std::cerr << "Problem reading file " << file_name << std::endl;
                        // }
                    } else {
                        png_destroy_read_struct(&png_ptr,(png_infopp)NULL, (png_infopp)NULL);
                        std::cerr << "Problem reading file (out of memory)" << file_name << std::endl;
                    }


                } else {
                    std::cerr << "Problem reading file (out of memory)" << file_name << std::endl;
                }
            } else {
                std::cerr << "File " << file_name << "is not a PNG" << std::endl;
            }
        } else {
            std::cerr << "File " << file_name << "is too short" << std::endl;
        }
    } else {
        std::cerr << "Couldn't open file " << file_name << std::endl;
    }

    // Free memory and structures used
    png_free_data( png_ptr, info_ptr, PNG_FREE_ALL, -1);


    if( fp ) fclose(fp);
    return pixel_data;
}


bool save_png_to_file( const std::string file_name, uint32_t width, uint32_t height, const uint16_t * pixel_data ) {
    bool saved_ok = false;

    FILE *fp = std::fopen( file_name.c_str(), "wb");
    if (fp) {
        png_structp png_ptr = png_create_write_struct (PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
        if (png_ptr) {


            png_infop info_ptr = png_create_info_struct(png_ptr);
            if (info_ptr) {

                if (!setjmp(png_jmpbuf(png_ptr))) {

                    // Set up write data
                    png_set_IHDR(png_ptr, info_ptr, width, height, 16, PNG_COLOR_TYPE_GRAY, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);

                    // Set up rows
                    png_bytepp row_pointers = (png_bytepp)new png_bytep[height];
                    png_bytep ptr = (png_bytep)pixel_data;
                    for( uint32_t i=0; i<height; i++ ) {
                        row_pointers[i] = ptr;
                        ptr += (width*2);
                    }
                    png_init_io(png_ptr, fp);

                    png_set_rows(png_ptr, info_ptr, row_pointers);

                    png_write_png(png_ptr, info_ptr, PNG_TRANSFORM_IDENTITY, NULL);

                    delete[] row_pointers;
                    png_destroy_write_struct(&png_ptr, (png_infopp)NULL);

                    saved_ok = true;
                } else {
                    png_destroy_write_struct(&png_ptr, &info_ptr);
                    std::cerr << "Problem with setjmp" << std::endl;
                }
            } else {
                png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
                std::cerr << "Problem creating PNG info struct for write" << std::endl;
            }
        } else {
            std::cerr << "Problem creating PNG ptr struct for write" << std::endl;
        }

    } else {
        std::cerr << "Couldn't open file " << file_name << " for writing" << std::endl;
    }

    if( fp ) fclose(fp);
    return saved_ok;
}

bool save_png_to_file( const std::string file_name, uint32_t width, uint32_t height, const uint8_t * pixel_data ) {
    bool saved_ok = false;

    FILE *fp = std::fopen( file_name.c_str(), "wb");
    if (fp) {
        png_structp png_ptr = png_create_write_struct (PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
        if (png_ptr) {


            png_infop info_ptr = png_create_info_struct(png_ptr);
            if (info_ptr) {

                if (!setjmp(png_jmpbuf(png_ptr))) {

                    // Set up write data
                    png_set_IHDR(png_ptr, info_ptr, width, height, 8, PNG_COLOR_TYPE_GRAY, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);

                    // Set up rows
                    png_bytepp row_pointers = (png_bytepp)new png_bytep[height];
                    png_bytep ptr = (png_bytep)pixel_data;
                    for( uint32_t i=0; i<height; i++ ) {
                        row_pointers[i] = ptr;
                        ptr += width;
                    }
                    png_init_io(png_ptr, fp);

                    png_set_rows(png_ptr, info_ptr, row_pointers);

                    png_write_png(png_ptr, info_ptr, PNG_TRANSFORM_IDENTITY, NULL);

                    delete[] row_pointers;
                    png_destroy_write_struct(&png_ptr, &info_ptr);

                    saved_ok = true;
                } else {
                    png_destroy_write_struct(&png_ptr, &info_ptr);
                    std::cerr << "Problem with setjmp" << std::endl;
                }
            } else {
                png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
                std::cerr << "Problem creating PNG info struct for write" << std::endl;
            }
        } else {
            std::cerr << "Problem creating PNG ptr struct for write" << std::endl;
        }

    } else {
        std::cerr << "Couldn't open file " << file_name << " for writing" << std::endl;
    }

    if( fp ) fclose(fp);
    return saved_ok;
}

bool save_colour_png_to_file( const std::string file_name, uint32_t width, uint32_t height, const uint8_t * pixel_data ) {
    bool saved_ok = false;

    FILE *fp = std::fopen( file_name.c_str(), "wb");
    if (fp) {
        png_structp png_ptr = png_create_write_struct (PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
        if (png_ptr) {


            png_infop info_ptr = png_create_info_struct(png_ptr);
            if (info_ptr) {

                if (!setjmp(png_jmpbuf(png_ptr))) {

                    // Set up write data
                    png_set_IHDR(png_ptr, info_ptr, width, height, 8, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);

                    // Set up rows
                    png_bytepp row_pointers = (png_bytepp)new png_bytep[height];
                    png_bytep ptr = (png_bytep)pixel_data;
                    for( uint32_t i=0; i<height; i++ ) {
                        row_pointers[i] = ptr;
                        ptr += (width*3);
                    }
                    png_init_io(png_ptr, fp);

                    png_set_rows(png_ptr, info_ptr, row_pointers);

                    png_write_png(png_ptr, info_ptr, PNG_TRANSFORM_IDENTITY, NULL);

                    delete[] row_pointers;
                    png_destroy_write_struct(&png_ptr, &info_ptr);

                    saved_ok = true;
                } else {
                    png_destroy_write_struct(&png_ptr, &info_ptr);
                    std::cerr << "Problem with setjmp" << std::endl;
                }
            } else {
                png_destroy_write_struct(&png_ptr, (png_infopp)NULL);
                std::cerr << "Problem creating PNG info struct for write" << std::endl;
            }
        } else {
            std::cerr << "Problem creating PNG ptr struct for write" << std::endl;
        }

    } else {
        std::cerr << "Couldn't open file " << file_name << " for writing" << std::endl;
    }

    if( fp ) fclose(fp);
    return saved_ok;
}
