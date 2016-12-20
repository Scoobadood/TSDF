#include "../include/cuda_utilities.hpp"
#include <iostream>

__host__
void check_cuda_error( const std::string& message, const cudaError_t err ) {
    if( err != cudaSuccess ) {
        std::cout << message << " [" << err << "] " << std::endl;
        std::cout << cudaGetErrorString( err ) << std::endl;
        exit( -1 );
    }
}

__device__
uint8_t atomicIncUint8( uint8_t* address ) {
	// Obtain the address of the base integer in which our char is stored
    unsigned int *base_address = (unsigned int *)((size_t)address & ~3);

    unsigned int old, assumed, new_;

    // Get the current value
    old = *base_address;

    // Now loop until success
    do {
    	//When i do atomic CAS later, I expect to see 'assumed'
        assumed = old;


        // Now extract the uint8 that I'm interested in
        // Endianess is little so 
        int byte_index = ((size_t)address & 3);
        unsigned int and_masks[] = { 0xFF000000, 0x00FF0000, 0x0000FF00, 0x000000FF}
        unsigned int shifts[] = { 24, 16, 8, 0 };

        uint8_t old_uint8 = ( old & masks[ byte_index] ) >> shifts[byte_index];
        uint8_t new_uint8 = old_uint8 + 1;
        uint new_int = (new_uint8 << shifts[byte_index]) & and_masks[ byte_index];

        new_ = old & ( ~and_masks[byte_index]);
        new_ = new_ | new_int;

        old = atomicCAS(base_address, assumed, new_);

    } while (assumed != old);

    return old;
}