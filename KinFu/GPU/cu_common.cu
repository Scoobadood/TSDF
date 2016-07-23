#include "cu_common.hpp"
#include <iostream>

__host__
void check_cuda_error( const std::string& message, const cudaError_t err ) {
    if( err != cudaSuccess ) {
        std::cout << message << err << std::endl;
        std::cout << cudaGetErrorString( err ) << std::endl;
        exit( -1 );
    }
}