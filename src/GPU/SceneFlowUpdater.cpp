#include "GPUMarchingCubes.hpp"
#include "Camera.hpp"



/**
 * Convert vertices to voxel centres.
 */


/**
 * Update the TSDF Volume using the Scene Flow data
 * @param volume The volume
 * @param width The width of the scene flow image
 * @param height The height of the scene flow image
 * @param translation The global transform
 * @param rotation The global rotation
 * @param residuals The per pixel residual translation
 */
void update( phd::TSDFVolume * volume, const Camera * camera, uint32_t width, uint32_t height, const float3 & translation, const float3 & rotation, const float3 * residuals ) {
	// Extract the mesh of the surface
	std::vector<float3> vertices;
	std::vector<int3>   triangles;

	extract_surface( volume, vertices, triangles );

     // For each vertex, project into scene flow image
	for( auto vert_iter = vertices.begin(); vert_iter != vertices.end(); vertices++ ) {
		float3 vertex = (float3) *vert_iter;

		// Project the vertex into the scene flow image using camera stats
		int2 pixel = camera->world_to_pixel( vertex );

		// Dereference the pixel into translations and rotation
		float3 trans = residuals[ pixel.y * width + pixel.x ];

		// Apply the translation to the node.

		// Retro spect these into voxel centres
		volume->translation_data[ ]
	}

     // Apply corrections to voxel coords
     // Extract deformed mesh from TSDF
     // merge depth image
 }