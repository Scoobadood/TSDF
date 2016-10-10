#ifndef SCENE_FLOW_UPDATER_HPP
#define SCENE_FLOW_UPDATER_HPP

#include "../TSDFVolume.hpp"
#include "Eigen/Core"

#include <vector>

/**
 * Update the Given TSDF volume's per voxel translation using the input Scene Flow 
 * @param volume The TSDF Volume to update
 * @param translation The global translation
 * @param rotation The Global rotation
 * @param residuals Per voxel ransation after globals are appliedd
 */
void update_tsdf( const phd::TSDFVolume * volume, const Eigen::Vector3f translation, const Eigen::Vector3f rotation, const Eigen::Matrix<float, 3, Eigen::Dynamic> residuals );

/**
 * Kernel to obtain scene flow vector for each point in the surface mesh
 * @param vertices The vertices of the mesh of the isosurface
 * @param camera The camera used; required for projection
 * @param sf_width The width of the scene flow image data
 * @param sf_height the height of the sf_image data
 * @param scene_flow The scene flow data sf_width*sf_height long
 * @param mesh_scene_flow The out put scene flow for each point in the mesh
 */
void get_scene_flow_for_mesh( const std::vector<float3> vertices, const Camera * camera, uint32_t sf_width, uint32_t sf_height, const float3 * scene_flow, std::vector<float3> mesh_scene_flow );

#endif

