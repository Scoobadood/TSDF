#ifndef DepthMapUtilities_h
#define DepthMapUtilities_h

#include <cstdint>
#include <string>

uint16_t * load_depth_map( std::string file_name, uint16_t & width, uint16_t & height);
uint16_t * read_nyu_depth_map( const std::string & file_name, uint32_t & width, uint32_t & height );
uint16_t * read_tum_depth_map( const std::string & file_name, uint32_t & width, uint32_t & height );

#endif
