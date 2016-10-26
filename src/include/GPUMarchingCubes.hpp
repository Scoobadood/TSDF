#ifndef MARCHING_CUBES_HPP
#define MARCHING_CUBES_HPP

#include "TSDFVolume.hpp"
#include "vector_types.h"

#include <vector>

/**
 * Implement marching cubes on the specified GPUTSDF 
 * @param volume The TSDF
 * @param vertices A vector of vertices
 * @param triangles A vector of Triangles
 */
void extract_surface( const TSDFVolume * volume, std::vector<float3>& vertices, std::vector<int3>& triangles);


#endif