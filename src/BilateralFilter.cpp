//
//  BilateralFilter.cpp
//  TSDF
//
//  Created by Dave on 30/04/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include <iostream>
#include <cmath>
#include <cstring>
#include "include/BilateralFilter.hpp"


BilateralFilter::BilateralFilter( float sigma_colour, float sigma_space ) : m_sigma_colour{ sigma_colour }, m_sigma_space{ sigma_space } {
    
    int kernel_radius = std::ceil( sigma_space * 1.5f );
    
    float inv_sigma_colour_squared = 1.0f / ( sigma_colour * sigma_colour );
    float inv_sigma_space_squared = 1.0f / ( sigma_space * sigma_space );
    
    // Kernel size is always odd
    m_kernel_size = kernel_radius * 2 + 1;
    int center = (m_kernel_size - 1) / 2;
    
    // Computer kernel values
    int idx = 0;
    m_kernel = new float[m_kernel_size * m_kernel_size];
    for (int x = -center; x < m_kernel_size-center; x++) {
        for (int y = -center; y < m_kernel_size-center; y++) {
            float dist_squared = (x * x + y * y);
            m_kernel[idx] = std::exp( - dist_squared * inv_sigma_space_squared );
            idx ++;
        }
    }
    
    // precompute all possible similarity values for performance
    m_similarity = new float[256];
    for (int i = 0; i < 256; i++) {
        m_similarity[i] = std::exp( - i * inv_sigma_colour_squared );
    }
}

BilateralFilter::~BilateralFilter( ) {
    if( m_similarity) {
        delete[] m_similarity;
    }
    if( m_kernel ) {
        delete[] m_kernel;
    }
}

void BilateralFilter::filter_bpp( const void * const image, int width, int height, int bpp ) const {
    int kernel_radius = (m_kernel_size - 1) / 2;
    
    size_t image_size = width * height * bpp / 8;
    
    // Apply it
    char * out_image = new char[ image_size];
    
    
    // For each pixel in the image
    size_t image_idx = 0;
    for( int y=0; y<height; y++ ) {
        for( int x=0; x<width; x++ ) {
            
            int current_pixel_intensity;
            if( bpp==8 ) {
                current_pixel_intensity= pixel_at( (const uint8_t * const ) image, width, x, y);
            } else {
                current_pixel_intensity= pixel_at( (const uint16_t * const ) image, width, x, y);
            }

            
            // For the neghbourhood, calculate the weighted average and the sum of weights
            float total_weight = 0;
            float sum = 0;
            
            
            // For each element in the kernel
            int kernel_idx = 0;
            for ( int conv_x = x - kernel_radius; conv_x <= x + kernel_radius; conv_x ++ ) {
                for (int conv_y = y - kernel_radius; conv_y <= y + kernel_radius; conv_y ++ ) {
                    
                    // If the conv pixel is in the image
                    if ( conv_x >= 0 && conv_x < width && conv_y >= 0 && conv_y < height ) {
                        
                        
                        int conv_pixel_intensity;
                        if( bpp == 8 ) {
                            conv_pixel_intensity = pixel_at((const uint8_t * const) image, width, conv_x, conv_y);
                        } else {
                            conv_pixel_intensity = pixel_at((const uint16_t * const) image, width, conv_x, conv_y);
                        }
                        
                        int delta_intensity = std::abs(conv_pixel_intensity - current_pixel_intensity);
                        
                        // Weight to apply is based on the kernel and the similarty of pixels
                        double conv_weight = m_kernel[kernel_idx] * m_similarity[delta_intensity];
                        
                        sum += (conv_weight * conv_pixel_intensity);
                        total_weight += conv_weight;
                        
                        
                        kernel_idx++;
                    }
                }
            }
            out_image[image_idx] = (int)floorf( sum / total_weight );
            
            image_idx++;
        }
    }
    
    // Move the out image over the in image
    std::memcpy( (void * ) image, ( void * )out_image, image_size);
    
    
    // Free up the out image
    delete[] out_image;
}


void BilateralFilter::filter( const uint8_t * const image, int width, int height ) const {
    filter_bpp(image, width, height, 8);
}

void BilateralFilter::filter( const uint16_t * const image, int width, int height ) const {
    filter_bpp(image, width, height, 16);
}