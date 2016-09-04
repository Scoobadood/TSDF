#include <iostream>
#include "GPURaycaster.hpp"
#include "Raycaster_kernel.hpp"

namespace phd {

/**
 * Raycast the TSDF and store discovered vertices and normals in the ubput arrays
 * @param volume The volume to cast
 * @param camera The camera
 * @param vertices The vertices discovered
 * @param normals The normals
 */
void GPURaycaster::raycast( const TSDFVolume & volume, const Camera & camera,
                            Eigen::Matrix<float, 3, Eigen::Dynamic> & vertices,
                            Eigen::Matrix<float, 3, Eigen::Dynamic> &normals ) const {
    cast( volume, camera, m_width, m_height, vertices, normals );
}
}
