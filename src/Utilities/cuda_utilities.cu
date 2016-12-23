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
        unsigned int masks[] = { 0xFF000000, 0x00FF0000, 0x0000FF00, 0x000000FF};
        unsigned int shifts[] = { 24, 16, 8, 0 };
        int byte_index = 3 - ((size_t)address & 3);
        unsigned int mask = masks[byte_index];
        unsigned int shift = shifts[byte_index];

        uint8_t old_uint8 = ( old & mask ) >> shift;
        uint8_t new_uint8 = old_uint8 + 1;
        uint new_int = (new_uint8 << shift) & mask;

        new_ = old & ( ~mask );
        new_ = new_ | new_int;

        old = atomicCAS(base_address, assumed, new_);

    } while (assumed != old);

    return old;
}

void cudaSafeAlloc( void ** ptr, size_t sz, const std::string& purpose ) {
    cudaError_t err = cudaMalloc( ptr, sz );
    check_cuda_error( "Failed to allocate device memory for " + purpose , err);
}

void cudaSafeFree( void * ptr, const std::string& purpose ) {
    cudaError_t err = cudaFree( ptr );
    check_cuda_error( "Failed to free device memory for " + purpose , err);
}