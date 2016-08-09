#include <iostream>
#include <cmath>

#include "TSDFVolume.hpp"
#include "Utilities/PngWrapper.hpp"
#include "Utilities/DepthMapUtilities.hpp"
#include "Utilities/RenderUtilities.hpp"
#include "Utilities/ply.hpp"
#include "GPU/BlockTSDFLoader.hpp"
#include "DataLoader/TUMDataLoader.hpp"
#include "GPU/MarchingCubes.hpp"



Eigen::Matrix4f g_campose;

/**
 * Make the TSDF from input files
 */
phd::TSDFVolume * make_tsdf(phd::TSDFVolume::volume_type type, int num_images ) {
    using namespace phd;
    using namespace Eigen;

    // Make volume
    TSDFVolume * volume = TSDFVolume::make_volume(type, Vector3i{ 512, 512, 512}, Vector3f{ 3000, 3000, 3000});
    volume->offset( -1500, -1500, -1500);

    // And camera (from FREI 1 IR calibration data at TUM)
    Camera camera{ 591.1f, 590.1f, 331.0f, 234.6f };

    // Create TUMDataLoader
    TUMDataLoader tdl{ "/mnt/hgfs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_rpy" };

    // Load depth image
    uint32_t width{0};
    uint32_t height{0};
    Vector3f camera_location;
    Vector3f camera_focus;


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
phd::TSDFVolume * load_tsdf( phd::TSDFVolume::volume_type type, const std::string& file_name) {
    phd::TSDFVolume * volume = nullptr;

    phd::BlockTSDFLoader loader;
    if ( loader.load_from_file( file_name ) ) {
        volume = loader.to_tsdf(type);
        if ( volume ) {
            std::cout << "Loaded volume " << volume->size().x() << "x" << volume->size().y() << "x" << volume->size().z() << "  (" << volume->physical_size().x() << "mm x " << volume->physical_size().y() << "mm x " << volume->physical_size().z() << "mm)"  << std::endl;
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
    phd::TSDFVolume::volume_type type = phd::TSDFVolume::GPU;
    phd::TSDFVolume *volume = nullptr;

    // Format is 
    // go -m=nn : make with nn frames or
    // go filename


    if ( argc == 2 ) {
        if( argv[1][0] != '-' ) {
            std::cout << "Loading TSDF" << std::endl;
            volume = load_tsdf( type, argv[1]);
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
                    volume = make_tsdf(type, n);
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
        phd::Camera camera { 585.6f, 585.6f, 316.0f, 247.6f  };

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

        p = normals_as_png( 640, 480, normals );
        p->save_to("/home/dave/Desktop/normals.png");
        delete p;

    } else {
        std::cout << "Couldn't make or load volume" << std::endl;
    }


    // Extract Mesh
    if( volume ) {
        std::vector<int3> triangles;
        std::vector<float3> verts;
        extract_surface( volume, verts, triangles);

        // Save to PLY file
        write_to_ply( "/home/dave/Desktop/mesh.ply", verts, triangles);
        delete volume;
    } else {
        std::cout << "Couldn't extract mesh because couldn't make or load volume" << std::endl;
    }


    return retval;
}

