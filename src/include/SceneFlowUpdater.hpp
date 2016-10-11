#ifndef SCENE_FLOW_UPDATER_HPP
#define SCENE_FLOW_UPDATER_HPP

#include "TSDFVolume.hpp"
#include "vector_types.h"
#include "Eigen/Core"

#include <vector>

/**
 * Update the Given TSDF volume's per voxel translation using the input Scene Flow 
 * @param volume The TSDF Volume to update
 * @param translation The global translation
 * @param rotation The Global rotation
 * @param residuals Per voxel ransation after globals are appliedd
 */
void update_tsdf( const TSDFVolume * volume, const Eigen::Vector3f translation, const Eigen::Vector3f rotation, const Eigen::Matrix<float, 3, Eigen::Dynamic> residuals );

#endif

