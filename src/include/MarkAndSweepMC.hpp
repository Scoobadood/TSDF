#include "../include/TSDFVolume.hpp"

#include <vector>


/**
 * Implement marching cubes on the specified GPUTSDF
 * @param volume The TSDF
 * @param vertices A vector of vertices
 * @param triangles A vector of Triangles
 */
void extract_surface( const TSDFVolume * volume, std::vector<float3>& vertices, std::vector<int3>& triangles);

/**
 * Extract the surface from the given TSDF using mark and sweep MC
 * @param volume 			The TSDF
 * @param num_vertices 		Populated by the method, returns the total number of vertices found
 * @param d_mesh_vertices 	A ref to a float3 pointer. This is allocated by the function and on return points to the vertex data, 
 * 							Vertex data is arranged in triangles oriented anti-clockwise.
 * @param d_m_dash 			A pointer to a set of flags (one per voxel in the TSDF) populated with true or false as the voxel is in M'
 */
void extract_surface_ms( const TSDFVolume * const 	volume, 					// in
                         int& 						num_vertices,				// out 
                         float3 * & 				d_mesh_vertices,			// out (allocated here)
                         uint8_t * const 			d_m_dash);