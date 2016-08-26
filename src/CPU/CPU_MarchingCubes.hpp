//
//  MarchingCubes.hpp
//  KinFu
//
//  Created by Dave on 7/05/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#ifndef MarchingCubes_hpp
#define MarchingCubes_hpp

#include <stdio.h>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include "TSDFVolume.hpp"

// Structure to store
typedef struct VERTEX_DATA {
    Eigen::Vector3f     location;
    float               value;
} VertexData;

/**
 * Determine the eight voxels whose values contribute to this cube in the grid. And extract their values
 *
 *        +----+----+
 *       / 4  / 5  /|
 *      +----+----+ |
 *     / 7  / 6  /| +
 *    +----+----+ |/
 *    |    |    | +
 *    | 7  | 6  |/
 *    +----+----+---+
 *       / 0  / 1  /|
 *      +----+----+ |
 *     / 3  / 2  /| +
 *    +----+----+ |/
 *    |    |    | +
 *    | 3  | 2  |/
 *    +----+----+
 *
 */
uint8_t constructCubeAndIndex( const phd::TSDFVolume & volume, const int vx, const int vy, const int vz, VertexData * vertices );


/**
 * Approximately zero float
 */
bool is_almost_zero( float a );
/**
 * approximately equal floats
 */
bool are_close(float  a, float b);

/**
 * @param v1 first vertex of triangle
 * @param v2 second vertex of triangle
 * @param v3 third vertex of triangle
 * @param p The point to test
 * @param strict If true, the point must be strictly within the bounds of the triangle, otherwise points on the edges or at a vertex are ok
 * @return true if p is inside the triangle defined by v0, v1, v2
 */
bool point_is_in_triangle( const Eigen::Vector3f & v0, const Eigen::Vector3f & v1, const Eigen::Vector3f & v2, const Eigen::Vector3f & p, bool strict );

/**
 * Compute the intersection of a ray and triangle using the Möller
 * @param p1, p2 p3 The vertices of the triangle in anticlockwise order
 * @param origin The point at which the ray originates
 * @param ray THe unit vector in the dircetion of the ray to intesrsect
 * @param p The point of intersection (populated by this function if it exists)
 * @param is_behind Set by this method if the ray travles backwrds through the triangle
 * @return true if the point interscets the triangle else false
 */
bool ray_triangle_intersect( const pcl::PointXYZ & v0, const pcl::PointXYZ & v1, const pcl::PointXYZ & v2, const Eigen::Vector3f & origin, const Eigen::Vector3f & ray, pcl::PointXYZ & p, bool & is_behind );

/**
 * Do marching cubes
 */
pcl::PolygonMesh do_marching_cubes( const phd::TSDFVolume & volume );


#endif /* MarchingCubes_hpp */
