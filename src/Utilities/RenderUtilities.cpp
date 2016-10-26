#include "../include/RenderUtilities.hpp"

#include <iostream>

void save_normals_as_colour_png( std::string filename, uint16_t width, uint16_t height, const Eigen::Matrix<float, 3, Eigen::Dynamic>& normals ) {

    PngWrapper * p = normals_as_png( width, height, normals );
    p->save_to( filename );
    delete p;
}

/*
 * Implement Lambertian colouring as per https://www.cs.unc.edu/~rademach/xroads-RT/RTarticle.html
 * @param filename Where to save file
 * @param width The with of the image
 * @param height The height of the imahe
 * @param vertices A width x height array of Vertex coordinates in global space
 * @param normals A width x height array of surface normals in global space
 * @param camera The Camera for which to render
 * @param light_source THe global position of the light source
 */
void save_rendered_scene_as_png(std::string filename, uint16_t width, uint16_t height,
const Eigen::Matrix<float, 3, Eigen::Dynamic>& vertices, const Eigen::Matrix<float, 3, Eigen::Dynamic>& normals, const Camera & camera, const Eigen::Vector3f & light_source) {

    PngWrapper * p = scene_as_png(width, height, vertices, normals, camera, light_source);
    p->save_to( filename );
    delete p;
}

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
                          const Camera & camera,
                          const Eigen::Vector3f & light_source) {
    PngWrapper * pw = NULL;
    // allocate image data
    uint64_t num_pixels = width * height;
    uint8_t * image = new uint8_t[num_pixels];
    if( image ) {

        // Ensure that there's always ambient light
        float ambient_coefficient = 0.2;
        float diffuse_coefficient = 1.0 - ambient_coefficient;

        // For each vertex/normal
        for (uint64_t idx = 0; idx < num_pixels; idx ++ ) {
            // Vertex in camera space
            Eigen::Vector3f vertex {vertices(0,idx), vertices(1,idx), vertices(2,idx)};

            // Compute vector from vertex to light source
            Eigen::Vector3f r = (light_source - vertex).normalized();
            Eigen::Vector3f n {normals(0,idx), normals(1,idx), normals(2,idx)};

            // Compute shade
            float shade = std::fmax( 0.0, n.dot( r ) );
            shade = ambient_coefficient + ( diffuse_coefficient * shade );

            image[idx] = std::floor( shade * 255  );
        }

        pw = new PngWrapper( width, height, image, PngWrapper::PNG_TYPE::GREYSCALE_8 );

        delete [] image;
    } else {
        std::cout << "couldn't allocate storage for image" << std::endl;
    }

    return pw;
}

PngWrapper * normals_as_png(uint16_t width, uint16_t height, const Eigen::Matrix<float,3,Eigen::Dynamic>& normals) {

    PngWrapper * pw = NULL;

    // Allocate grey scale
    uint64_t num_pixels = width * height;
    uint64_t data_size = num_pixels * 3;
    uint8_t * image = new uint8_t[data_size];
    if( image ) {
        uint32_t write_idx = 0;
        for (uint64_t idx = 0; idx < num_pixels; idx ++ ) {

            float n[3];
            n[0] = normals( 0,idx);
            n[1] = normals(1,idx);
            n[2] = normals(2,idx);

            if( n[2] < 0 ) n[2] = -n[2];
            n[0] = ((n[0] / 2.0f) + 0.5) * 255;
            n[1] = ((n[1] / 2.0f) + 0.5) * 255;
            n[2] = ((n[2] / 2.0f) + 0.5) * 255;
            image[write_idx++] = floor( n[0] );
            image[write_idx++] = floor( n[1] );
            image[write_idx++] = floor( n[2] );
        }
        pw = new PngWrapper( width, height, image, PngWrapper::PNG_TYPE::COLOUR );

        delete [] image;
    } else {
        std::cout << "couldn't allocate storage for image" << std::endl;
    }
    return pw;
}

