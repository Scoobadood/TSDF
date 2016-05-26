//
//  main.cpp
//  KinFu
//
//  Created by Dave on 1/05/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include <iostream>
#include <fstream>


//// NYU Maps are in mm already but do need to be byte swapped
//uint16_t * read_nyu_depth_map( const std::string & file_name, uint32_t & width, uint32_t & height ) {
//    uint16_t * range_map = read_pgm( file_name, width, height );
//    
//    size_t map_size = width * height;
//    for( size_t i=0; i<map_size; i++ ) {
//        uint16_t v = range_map[i];
//        
//        v = (v >> 8) + ( ( v & 0xFF ) * 256 );
//        
//        // Convert to metres by dividing by 5000, then to millimetres by multiplying by 1000
//        range_map[i] = v;
//    }
//    
//    return range_map;
//}






int main(int argc, const char * argv[]) {
    using namespace std;
    
    float f = 0.001;
    float fa[] = {1.1,2.001,3};
    string fn ="/Users/Dave/Desktop/out.txt";
    
    ofstream ofs{ fn, ios::out };
    
    ofs << 0.003 << endl;
    ofs << f << endl;
    for( int i=0; i<3; i++ ) {
       ofs << fa[i] <<" " ;
    }
    ofs << endl;
    ofs.close();
    
    return 0;
}







