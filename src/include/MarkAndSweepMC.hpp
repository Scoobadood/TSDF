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
 * @param volume The TSDF
 * @param vertices A ref to a float3 pointer. This is allocated by the cuntion and on return points to the vertex data, Vertex data is arranged in triangles oriented anti-clockwise.
 * @param num_vertices populated by the method contains the total number of vertices found
 */
void extract_surface_ms( const TSDFVolume * const 	volume, 					// in
                         int& 						num_vertices,				// out 
                         float3 *& 					d_mesh_vertices,			// out (allocated here)
                         int*& 						d_mesh_vertex_voxel_indices,// out
                         uint8_t*&					d_mesh_vertex_voxel_count	// out
                          );

__device__
void voxel_indices_for_cube_index( const int cube_index, const int grid_size_x, const int grid_size_y, int voxel_indices[8] );
