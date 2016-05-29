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

public:
    BilateralFilter( float sigma_colour, float sigma_space );
    ~BilateralFilter( );
    
    void filter( const unsigned char * image, int width, int height ) const;
};

#endif /* BilateralFilter_hpp */
