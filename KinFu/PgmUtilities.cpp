//
//  PgmUtilities.cpp
//  KinFu
//
//  Created by Dave on 5/05/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include "PgmUtilities.hpp"
#include <fstream>
#include <assert.h>

/*
 A "magic number" for identifying the file type. A pgm image's magic number is the two characters "P5".
 Whitespace (blanks, TABs, CRs, LFs).
 A width, formatted as ASCII characters in decimal.
 Whitespace.
 A height, again in ASCII decimal.
 Whitespace.
 The maximum gray value (Maxval), again in ASCII decimal. Must be less than 65536, and more than zero.
 A single whitespace character (usually a newline).
 A raster of Height rows, in order from top to bottom. Each row consists of Width gray values, in order from left to right. Each gray value is a number from 0 through Maxval, with 0 being black and Maxval being white. Each gray value is represented in pure binary by either 1 or 2 bytes. If the Maxval is less than 256, it is 1 byte. Otherwise, it is 2 bytes. The most significant byte is first.
 A row of an image is horizontal. A column is vertical. The pixels in the image are square and contiguous.
 
 Each gray value is a number proportional to the intensity of the pixel, adjusted by the ITU-R Recommendation BT.709 gamma transfer function. (That transfer function specifies a gamma number of 2.2 and has a linear section for small intensities). A value of zero is therefore black. A value of Maxval represents CIE D65 white and the most intense value in the image and any other image to which the image might be compared.
 
 Note that a common variation on the PGM format is to have the gray value be "linear," i.e. as specified above except without the gamma adjustment. pnmgamma takes such a PGM variant as input and produces a true PGM as output.
 
 In the transparency mask variation on PGM, the value represents opaqueness. It is proportional to the fraction of intensity of a pixel that would show in place of an underlying pixel. So what normally means white represents total opaqueness and what normally means black represents total transparency. In between, you would compute the intensity of a composite pixel of an "under" and "over" pixel as under * (1-(alpha/alpha_maxval)) + over * (alpha/alpha_maxval). Note that there is no gamma transfer function in the transparency mask.
 */

uint32_t read_next_number( std::ifstream & is ) {
    uint32_t value = 0;
    
    // Skip whitespace
    char c;
    while( ( is.get(c) ) && (c == ' ' || c == '\t' || c=='\n' || c == '\r' ) );
    while( c >= '0' && c <= '9' ) {
        value *= 10;
        value += ( c - '0' );
        is.get(c);
    }
    is.putback(c);
    
    return value;
}

uint16_t * read_pgm( const std::string & file_name, uint32_t & width, uint32_t & height ) {
    using namespace std;
    
    ifstream is(file_name, ios::binary );
    uint8_t c;
    
    c = is.get();
    assert(c=='P');
    c = is.get();
    assert(c=='5');
    
    // Read ASCII decimal digits
    uint32_t ww = read_next_number( is );
    uint32_t hh = read_next_number(is);
    uint32_t maxval = read_next_number(is);
    
    c = is.get(); // Should be whitespace
    
    uint16_t * data = new uint16_t[ ww * hh];
    uint32_t idx = 0;
    for( uint32_t r=0; r<hh; r++ ) {
        // Read a row
        for( uint32_t c=0; c<ww; c++ ) {
            uint16_t value;
            if( maxval < 256 ) {
                value = is.get();
            } else {
                value = is.get() * 256;
                value += is.get();
            }
            
            data[idx++] = value;
        }
    }
    
    width = ww;
    height = hh;
    return data;
}