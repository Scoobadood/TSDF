#include "../include/DepthMapUtilities.hpp"
#include "../include/PngUtilities.hpp"
#include "../include/PgmUtilities.hpp"

uint16_t * read_tum_depth_map( const std::string & file_name, uint32_t & width, uint32_t & height ) {

    uint16_t *range_map = load_png_from_file(file_name, width, height);
    size_t map_size = width * height;
    for( size_t i=0; i<map_size; i++ ) {
        uint16_t v = range_map[i];

        // Convert to metres by dividing by 5000, then to millimetres by multiplying by 1000
        range_map[i] = v / 5;
    }

    return range_map;
}

// NYU Maps are in mm already but do need to be byte swapped
uint16_t * read_nyu_depth_map( const std::string & file_name, uint32_t & width, uint32_t & height ) {
    uint16_t * range_map = read_pgm( file_name, width, height );

    size_t map_size = width * height;
    for( size_t i=0; i<map_size; i++ ) {
        uint16_t v = range_map[i];

        v = (v >> 8) + ( ( v & 0xFF ) * 256 );

        range_map[i] = v;
    }

    return range_map;
}
