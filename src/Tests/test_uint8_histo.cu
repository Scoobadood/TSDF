#include <iostream>
#include "../include/cuda_utilities.hpp"


__global__
void do_histo( int num_entries, const uint8_t * const d_source_array, uint8_t * d_histo ) {
	uint tid = blockIdx.x * blockDim.x + threadIdx.x;

	if( tid < num_entries ) {

		uint8_t value = d_source_array[tid];
		uint8_t * bin_address = d_histo+value;
		atomicIncUint8(  bin_address );
	}
}

__host__
void test_uint8_histo( ) {

	int num_entries = 256*123;

	// Build array
	uint8_t h_source_data[ num_entries];
	for( int i=0; i<num_entries; i++ ) {
		h_source_data[i] = i % 256;
	}


	// Do histo
	uint8_t * d_source_data;
	uint8_t * d_histo;

	cudaSafeAlloc( (void **) &d_source_data, num_entries * sizeof( uint8_t ), "d_source_data" );
	cudaSafeCopyToDevice( (void *) h_source_data, d_source_data, num_entries * sizeof( uint8_t ), "d_source_data");

	cudaSafeAlloc( (void **) &d_histo, 256, "d_histo" );

	cudaMemset( d_histo, 0, 256 * sizeof( uint8_t) );
	check_cuda_error( "Couldn't clear histo data", err );

	dim3 block( 100 );
	dim3 grid( divUp( num_entries, block.x));
	do_histo<<< grid, block >>>( num_entries, d_source_data, d_histo );
	err = cudaDeviceSynchronize();
	check_cuda_error( "Kernel failed", err );

	cudaSafeFree( d_source_data, "d_source_data" );
	check_cuda_error( "Couldn't free source data", err );

	uint8_t * h_histo = (uint8_t *) new uint8_t[256];
	cudaSafeCopyToHost( (void *) h_histo, (void *) d_histo, 256 * sizeof( uint8_t), "h_histo");
	cudaSafeFree( d_histo, "d_histo" );

	for( int i=0; i<256; i++ ) {
		std::cout << "bin " << i << ":   " << (int)h_histo[i] << std::endl;
	}

	delete [] h_histo;
}

int main( int argc, char *argv[] ) {
	test_uint8_histo( );
}