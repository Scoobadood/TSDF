//
//  BilateralFilter.hpp
//  TSDF
//
//  Created by Dave on 30/04/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#ifndef BilateralFilter_hpp
#define BilateralFilter_hpp

class BilateralFilter {
private:
    float   m_sigma_colour;
    float   m_sigma_space;
    float   * m_kernel;
    float   * m_similarity;
    int     m_kernel_size;

    inline uint8_t pixel_at( const uint8_t * const image, uint16_t width, uint16_t x, uint16_t y ) const {
        return image[width*y + x];
    }
    
    inline uint16_t pixel_at( const uint16_t * const image, uint16_t width, uint16_t x, uint16_t y ) const {
        return image[width*y + x];
    }
    
    void filter_bpp( const void * const image, int width, int height, int bpp ) const;
public:
    BilateralFilter( float sigma_colour, float sigma_space );
    ~BilateralFilter( );
    
    void filter( const uint8_t * depth_image, int width, int height ) const;
    void filter( const uint16_t * depth_image, int width, int height ) const;
};

#endif /* BilateralFilter_hpp */
