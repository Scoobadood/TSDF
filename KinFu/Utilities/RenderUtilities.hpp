#ifndef RenderUtilities_h
#define RenderUtilities_h

#include "PngWrapper.hpp"

#include <Eigen/Dense>

namespace phd{
    class Camera;
};

PngWrapper * normals_as_png(uint16_t width, uint16_t height, const Eigen::Matrix<float,3,Eigen::Dynamic>& normals);
/*
 * Implement Lambertian colouring as per https://www.cs.unc.edu/~rademach/xroads-RT/RTarticle.html
 * @param width The with of the image
 * @param height The height of the imahe
 * @param vertices A width x height array of Vertex coordinates in global space
 * @param normals A width x height array of surface normals in global space
 * @param camera The Camera for which to render
 * @param light_source THe global position of the light source
 */
PngWrapper * scene_as_png(uint16_t width, uint16_t height,
                          const Eigen::Matrix<float,3,Eigen::Dynamic>& vertices,
                          const Eigen::Matrix<float,3,Eigen::Dynamic>& normals,
                          const phd::Camera & camera,
                          const Eigen::Vector3f & light_source);

void save_normals_as_colour_png( std::string filename, uint16_t width, uint16_t height, const Eigen::Matrix<float,3,Eigen::Dynamic>& normals );
void save_rendered_scene_as_png( std::string filename, uint16_t width, uint16_t height, const Eigen::Matrix<float,3,Eigen::Dynamic>& vertices, const Eigen::Matrix<float,3,Eigen::Dynamic>& normals, const phd::Camera & camera, const Eigen::Vector3f & light_source);

#endif // RenderUtilities_h
