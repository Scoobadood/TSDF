#include <iostream>
#include <cmath>


#include "TSDFVolume.hpp"
#include "Utilities/PngWrapper.hpp"


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
            float shade = std::fmax( 0.0, r.dot( n ) );
            shade = ambient_coefficient + ( diffuse_coefficient * shade );

            image[idx] = std::floor( shade * 255  );
        }

        pw = new PngWrapper( width, height, image, PngWrapper::PNG_TYPE::GREYSCALE );
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
    } else {
        std::cout << "couldn't allocate storage for image" << std::endl;
    }
    return pw;
}

int main( int argc, const char * argv[] ) {
    int retval = 0;
    if( argc == 2 ) {

        // Load a TSDF
        phd::TSDFVolume *volume = phd::TSDFVolume::make_volume(phd::TSDFVolume::CPU);


        std::cout << "Loading TSDF" << std::endl;
        if (volume->load_from_file( argv[1] ) ) {
            std::cout << "Loaded volume " << volume->size().x() << "x" << volume->size().y() << "x" << volume->size().z() << "  (" << volume->physical_size().x() << "mm x "<< volume->physical_size().y() << "mm x " << volume->physical_size().z() << "mm)"  << std::endl;

            phd::Camera camera { 585.6f, 585.6f, 316.0f, 247.6f  };

            Eigen::Matrix< float, 3, Eigen::Dynamic> vertices;
            Eigen::Matrix< float, 3, Eigen::Dynamic> normals;

            std::cout << "Raycasting" << std::endl;
            camera.move_to( 1344.38, 627.206, 1661.75 );
            camera.look_at( 1332.94, 1416.95, 2275.08 );

            volume->raycast( 640, 480, camera, vertices, normals );

            // Render to image
            Eigen::Vector3f light_source { 7100, 2500, -2100 };

            std::cout << "Saving PNG" << std::endl;
            PngWrapper *p = scene_as_png( 640, 480, vertices, normals, camera, light_source );
            p->save_to("/home/dave/Desktop/scene.png");
            delete p;

            p = normals_as_png( 640, 480, normals );
            p->save_to("/home/dave/Desktop/normals.png");
            delete p;

            delete volume;

        } else {
            std::cout << "Failed to read TSDF" << std::endl;
            retval = 1;
        }
    } else {
        std::cerr << "Must specify a volume file" << std::endl;
        retval = 2;
    }
    return retval;
}

