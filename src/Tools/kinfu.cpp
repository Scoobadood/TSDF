#include <iostream>
#include <cmath>

#include "../include/TSDFVolume.hpp"
#include "../include/PngWrapper.hpp"
#include "../include/DepthMapUtilities.hpp"
#include "../include/RenderUtilities.hpp"
#include "../include/ply.hpp"
#include "../include/BlockTSDFLoader.hpp"
#include "../include/TUMDataLoader.hpp"
#include "../include/GPUMarchingCubes.hpp"



Eigen::Matrix4f g_campose;

Camera make_camera( ) {
    return Camera{ 591.1f, 590.1f, 331.0f, 234.6f };
}

/**
 * Make the TSDF from input files
 */
TSDFVolume * make_tsdf(int num_images ) {
    using namespace Eigen;

    // Make volume
    TSDFVolume * volume = new TSDFVolume( TSDFVolume::UInt3{ 450, 450, 450}, TSDFVolume::UInt3{8000, 8000, 8000});
    volume->offset(-4000,-4000,-4000);
    
    // And camera (from FREI 1 IR calibration data at TUM)
    Camera camera = make_camera();

    // Create TUMDataLoader
    TUMDataLoader tdl{ "/mnt/hgfs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg2_xyz" };

    // Construct TSDF Volume
    for ( int i = 0; i < num_images; i++ ) {
        std::cout << "Integrating frame " << i << std::endl;

        // Read it
        Matrix4f pose;
        DepthImage * depthmap = tdl.next( pose );

        if( i == 0 ) {
            g_campose = pose ;
        }

        if ( depthmap ) {
            // Set location
            camera.set_pose( pose );
            volume->integrate(depthmap->data() , depthmap->width(), depthmap->height(), camera);

            delete depthmap;
        }
    }

    return volume;
}





/**
 * Load the TSDF
 */
TSDFVolume * load_tsdf( const std::string& file_name) {
    TSDFVolume * volume = nullptr;

    BlockTSDFLoader loader;
    if ( loader.load_from_file( file_name ) ) {
        volume = loader.to_tsdf();
        if ( volume ) {
            std::cout << "Loaded volume " << volume->size().x << "x" << volume->size().y << "x" << volume->size().z << "  (" << volume->physical_size().x << "mm x " << volume->physical_size().y << "mm x " << volume->physical_size().z << "mm)"  << std::endl;
        } else {
            std::cout << "Couldn't load volume"  << std::endl;
        }
    } else {
        std::cout << "Failed to read TSDF" << std::endl;
    }
    return volume;
}






int main( int argc, const char * argv[] ) {
    int retval = 0;
    TSDFVolume *volume = nullptr;

    // Format is 
    // go -m=nn : make with nn frames or
    // go filename


    if ( argc == 2 ) {
        if( argv[1][0] != '-' ) {
            std::cout << "Loading TSDF" << std::endl;
            volume = load_tsdf( argv[1]);
        } else {
            int arglen = strlen( argv[1] );
            if( ( arglen >=4 ) && (argv[1][1] == 'm') && (argv[1][2] == '=' ) ) {
                int n=0;
                int idx = 3;
                while( idx < arglen ) {
                    n = n * 10;
                    n = n + argv[1][idx] - '0';
                    std::cout << n << std::endl;
                    idx++;
                }

                if( n > 0 ) {
                    std::cout << "Make TSDF" << std::endl;
                    volume = make_tsdf( n);
                    volume->save_to_file( "/home/dave/Desktop/volume.bin");

                } else {
                    std::cout << "Invalid frame count " << n << std::endl;
                }
            } else {
                std::cout << "Usage : " << argv[0] << " -m=<nn> | file" << std::endl;
            }
        }

    } else {
        std::cout << "Usage : " << argv[0] << " -m=<nn> | file" << std::endl;
    }


    // Save norm and verts
    if ( volume ) {
        Camera camera = make_camera();

        Eigen::Matrix< float, 3, Eigen::Dynamic> vertices;
        Eigen::Matrix< float, 3, Eigen::Dynamic> normals;

        std::cout << "Raycasting" << std::endl;

        camera.set_pose( g_campose);

        volume->raycast( 640, 480, camera, vertices, normals );

        std::cout << "Rendering to image " << std::endl;
        Eigen::Vector3f light_source { g_campose(0,3), g_campose(1,3), g_campose(2,3) };

        PngWrapper *p = scene_as_png( 640, 480, vertices, normals, camera, light_source );

        std::cout << "Saving PNG" << std::endl;
        p->save_to("/home/dave/Desktop/scene.png");
        delete p;

        std::cout << "Rendering normals to image " << std::endl;
        p = normals_as_png( 640, 480, normals );
        std::cout << "Saving PNG" << std::endl;
        p->save_to("/home/dave/Desktop/normals.png");
        delete p;

    } else {
        std::cout << "Couldn't make or load volume" << std::endl;
    }


    // Extract Mesh
    if( volume ) {
        std::cout << "Extracting ISO surface" << std::endl;
        std::vector<int3> triangles;
        std::vector<float3> verts;
        extract_surface( volume, verts, triangles);

        // Save to PLY file
        std::cout << "Writing "<< verts.size() << " vertices and " << triangles.size() << " triangles to PLY" << std::endl;
        write_to_ply( "/home/dave/Desktop/mesh.ply", verts, triangles);
        delete volume;
    } else {
        std::cout << "Couldn't extract mesh because couldn't make or load volume" << std::endl;
    }


    return retval;
}

