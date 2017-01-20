#include "../include/TSDFVolume.hpp"
#include "../include/MarkAndSweepMC.hpp"
#include "../include/ply.hpp"

#include "vector_types.h"

#include <vector>
#include <fstream>

#include <Eigen/Dense>

TSDFVolume * make_sphere_tsdf( const uint16_t vx, const  uint16_t vy, const uint16_t vz, const float px, const float  py, const float pz ) {
	using namespace Eigen;


	TSDFVolume *volume = new TSDFVolume(  vx, vy, vz, px, py, pz );

	// Now make some data
	float * data = new float[ vx * vy * vz];

	float radius = fminf( px, fminf( py, pz ) ) / 2.5f;

	Vector3f vsize{ px / (float)vx, py / (float)vy, pz / (float)vz };

	size_t idx = 0;
	for ( int z = 0; z < vz; z++ ) {
		float dz = ( (z + 0.5f) * vsize.z() ) - ( pz / 2.0f );
		for ( int y = 0; y < vy; y++ ) {
			float dy = ( (y + 0.5f) * vsize.y() ) - ( py / 2.0f );
			for ( int x = 0; x < vx; x++ ) {
				float dx = ( (x + 0.5f) * vsize.x() ) - ( px / 2.0f );

				float dist = sqrt(dx * dx + dy * dy + dz * dz);

				data[idx++] = dist - radius;
			}
		}
	}


	// Set the data
	volume->set_distance_data( data );

	delete [] data;

	return volume;
}


/**
 * Initialise the deformation field with a banana style bend
 * @param translations X x Y x Z array of float3s
 * @param grid_size The size of the voxel grid
 * @param voxel_size The size of an individual voxel
 * @param grid_offset The offset of the grid
 */
TSDFVolume::DeformationNode * build_twist_translation_data( TSDFVolume * volume ) {

	TSDFVolume::Float3 grid_offset = volume->offset( );
	TSDFVolume::Float3 voxel_size  = volume->voxel_size();
	TSDFVolume::UInt3  grid_size   = volume->size();


	// Compute the centre of rotation
	TSDFVolume::Float3 centre_of_rotation {
		grid_offset.x + ( 1.5f * grid_size.x * voxel_size.x),
		grid_offset.y + ( 0.5f * grid_size.y * voxel_size.y),
		grid_offset.z + ( 0.5f * grid_size.z * voxel_size.z),
	};

	int data_size =  grid_size.x * grid_size.y * grid_size.z;

	TSDFVolume::DeformationNode * deformations = new TSDFVolume::DeformationNode[ data_size ];

	// We want to iterate over the entire voxel space
	// Each thre	ad should be a Y,Z coordinate with the thread iterating over x
	int voxel_index = 0;
	for ( int vz = 0; vz < grid_size.z; vz++ ) {
		for ( int vy = 0; vy < grid_size.y; vy++ ) {
			for ( int vx = 0; vx < grid_size.x; vx++ ) {

				// Starting point
				float3 tran{
					(( vx + 0.5f ) * voxel_size.x) + grid_offset.x,
					(( vy + 0.5f ) * voxel_size.y) + grid_offset.y,
					(( vz + 0.5f ) * voxel_size.z) + grid_offset.z
				};

				// Compute current angle with cor and hor axis
				float dx = tran.x - centre_of_rotation.x;
				float dy = tran.y - centre_of_rotation.y;

				float theta = atan2( dy, dx ) * 2;

				float sin_theta = sin( theta );
				float cos_theta = cos( theta );

				// Compute relative X and Y
				float rel_x = ( tran.x - centre_of_rotation.x );
				float rel_y = ( tran.y - centre_of_rotation.y );

				deformations[voxel_index].translation.x = ( ( cos_theta * rel_x ) - ( sin_theta * rel_y ) ) + centre_of_rotation.x;
				deformations[voxel_index].translation.y = ( ( sin_theta * rel_x ) + ( cos_theta * rel_y ) ) + centre_of_rotation.y;
				deformations[voxel_index].translation.z = tran.z;
				deformations[voxel_index].rotation.x = 0.0f;
				deformations[voxel_index].rotation.y = 0.0f;
				deformations[voxel_index].rotation.z = 0.0f;

				voxel_index++;
			}
		}
	}
	return deformations;
}
/**
 * Test the Marching Cubes code
 */
int main( int argc, const char * argv[] ) {
	int retval = 0;

	TSDFVolume *volume = make_sphere_tsdf( 200, 200, 200, 2000, 2000, 2000 );

	if ( volume ) {
		TSDFVolume::DeformationNode * deformation = build_twist_translation_data( volume );

		if ( deformation ) {
			volume->set_deformation( deformation );

			delete [] deformation;

			std::vector<int3> triangles;
			std::vector<float3> vertices;
			extract_surface( volume, vertices, triangles);

			// Save to PLY file
			write_to_ply( "/home/dave/Desktop/sphere.ply", vertices, triangles);
		} else {
			std::cout << "Couldn't build twist data" << std::endl;
			retval = -1;
		}

	} else {
		std::cout << "Couldn't make TSDF volume spehere " << std::endl;
		retval = -1;
	}

	return retval;

}
