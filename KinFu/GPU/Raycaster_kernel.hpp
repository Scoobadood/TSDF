#include "../../TSDFVolume.hpp"
#include "../../Camera.hpp"
#include <Eigen/Core>

void cast(	const phd::TSDFVolume & volume,
            const phd::Camera & camera,
            uint16_t width,
            uint16_t height,
            Eigen::Matrix<float, 3,  Eigen::Dynamic> & vertices,
            Eigen::Matrix<float, 3,  Eigen::Dynamic> & normals );

void sphere_cast(const phd::TSDFVolume & volume,
                 const phd::Camera & camera,
                 uint16_t width,
                 uint16_t height,
                 Eigen::Matrix<float, 3,  Eigen::Dynamic> & vertices,
                 Eigen::Matrix<float, 3,  Eigen::Dynamic> & normals );
