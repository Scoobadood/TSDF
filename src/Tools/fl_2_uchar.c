#include <stdio.h>
#include <malloc.h>
#include <stdlib.h>

typedef unsigned int uint32_t;
typedef unsigned char uint8_t;



int main( int argc, char * argv[] ) {
	char * in_file_name = argv[1];
	char * out_file_name = argv[2];

	FILE * infile = fopen( in_file_name, "rb" );
	FILE * outfile = fopen( out_file_name, "wb" );

	if (!infile ) {
		printf( "Couldn't find input file\n" );
		exit( -1 );
	}

	if (!outfile) {
		printf( "Couldn't find output file\n" );
		exit( -1 );
	}


	uint32_t size[3];
	float psize[3];
	fread( size, sizeof( uint32_t ), 3, infile );
	fread( psize, sizeof( float ), 3, infile );



	// Allocate storage for my depth map
	size_t num_entries = size[0] * size[1] * size[2];

	float *in = (float *)malloc( num_entries * sizeof( float ) );
	if ( !in ) {
		printf( "Couldn't allocate input storage\n" );
		exit( -2 );
	}

	// Allocate out
	uint8_t *out = (uint8_t *) malloc( num_entries * sizeof( uint8_t ) );
	if ( !out ) {
		free (in);
		printf( "Couldn't allocate input storage\n" );
		exit( -2 );
	}



	fread( in, num_entries, sizeof( float ), infile );

	float min = 9999.0f;
	float max = -9999.0f;

	for ( int i = 0; i < num_entries; i++ ) {
		if( in[i] > max) max = in[i];
		if( in[i] < min ) min = in[i];
	}

	printf( "Min: %f, Max : %f\n", min, max );

	float scale = 255.0f / (max - min);

	for ( int i = 0; i < num_entries; i++ ) {
		out[i] = (in[i] - min ) * 255;
	}

	fwrite( out, sizeof( uint8_t ), num_entries, outfile);


	fclose( infile );
	fclose( outfile );

	free (in);
	free (out);
}