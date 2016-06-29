#include <iostream>
#include <cmath>


#include "TSDFVolume.hpp"
#include "Utilities/PngWrapper.hpp"
#include "Utilities/DepthMapUtilities.hpp"
#include "Utilities/RenderUtilities.hpp"
#include "../TestTSDF/TestHelpers.hpp"




struct {
    std::string     file_name;
    float           ground_truth[7];
}  g_data[] =
{
// Make these relative paths

    { "/mnt/hgfs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg2_xyz/depth/1311867170.450076.png", {0.1554, -1.1425, 1.3593, -0.5691, 0.6454, -0.3662, 0.3541}},
    { "/mnt/hgfs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.160407.png", {1.344379, 0.627206, 1.661754, 0.658249, 0.611043, -0.294444, -0.326553}},
    { "/mnt/hgfs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.194330.png", {1.343641, 0.626458, 1.652408, 0.657327, 0.613265, -0.295150, -0.323593}},
    { "/mnt/hgfs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.226738.png", {1.338382, 0.625665, 1.641460, 0.657713, 0.615255, -0.294626, -0.319485}},
    { "/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.262886.png", {1.325627, 0.624485, 1.632561, 0.659141, 0.617445, -0.292536, -0.314195}},
    { "/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.295279.png", {1.312190, 0.625418, 1.625809, 0.660869, 0.619147, -0.290608, -0.308959}},
    { "/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.329195.png", {1.301563, 0.623031, 1.616491, 0.662153, 0.619222, -0.290126, -0.306504}},
    { "/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.363013.png", {1.293270, 0.626161, 1.607816, 0.662227, 0.620410, -0.290893, -0.303198}},
    { "/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.394772.png", {1.284946, 0.625813, 1.599284, 0.661801, 0.622191, -0.291109, -0.300256}},
    { "/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.427815.png", {1.284070, 0.623464, 1.589476, 0.661726, 0.624201, -0.290800, -0.296526}},
    { "/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.462395.png", {1.280648, 0.627129, 1.578073, 0.662090, 0.625917, -0.290794, -0.292069}},
    { "/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.494271.png", {1.254294, 0.627271, 1.558543, 0.663700, 0.629278, -0.284166, -0.287683} },
    { "/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.526330.png", {1.238252, 0.632818, 1.555590, 0.664967, 0.632747, -0.277169, -0.283951} },
    { "/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.562224.png", {1.223685, 0.628649, 1.548521, 0.666343, 0.630651, -0.274921, -0.287549} },
    { "/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.594158.png", {1.220404, 0.625367, 1.539421, 0.666832, 0.628663, -0.275898, -0.289822} },
    { "/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.626818.png", {1.217731, 0.623185, 1.528533, 0.666335, 0.628441, -0.276938, -0.290457} },
    { "/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.663273.png", {1.205526, 0.623843, 1.519896, 0.667511, 0.628343, -0.276176, -0.288688} },
    { "/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.695165.png", {1.202711, 0.624060, 1.508244, 0.668394, 0.626923, -0.277292, -0.288663} },
    { "/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.728423.png", {1.193503, 0.630835, 1.497742, 0.668160, 0.628311, -0.276110, -0.287316} },
    { "/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.763549.png", {1.185448, 0.631389, 1.487746, 0.669018, 0.628258, -0.273096, -0.288315} },
    { "/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.794978.png", {1.176852, 0.634599, 1.478039, 0.669657, 0.628210, -0.270747, -0.289150} },
    { "/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.828537.png", {1.165553, 0.632181, 1.469138, 0.669721, 0.628905, -0.266464, -0.291460} },
    { "/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.862808.png", {1.160138, 0.630265, 1.458932, 0.668689, 0.628908, -0.265691, -0.294513} },
    { "/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.894167.png", {1.153575, 0.625835, 1.449607, 0.668317, 0.629068, -0.264450, -0.296129} },
    { "/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.926851.png", {1.147986, 0.612716, 1.440107, 0.666125, 0.631064, -0.264441, -0.296828} },
    { "/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.962137.png", {1.135779, 0.612160, 1.419704, 0.666560, 0.631448, -0.273130, -0.287005} },
};


/**
 * Make the TSDF from input files
 */
phd::TSDFVolume * make_tsdf( ) {
    using namespace phd;
    using namespace Eigen;
    // Make volume
    TSDFVolume * volume = TSDFVolume::make_volume(phd::TSDFVolume::CPU, Vector3i{ 64, 64, 64}, Vector3f{ 3000, 3000, 3000});

    // And camera
    Camera camera{ 585.6f, 585.6f, 316.0f, 247.6f };

    // Load depth image
    uint32_t width{0};
    uint32_t height{0};
    Vector3f camera_location;
    Vector3f camera_focus;
    int num_images = 2;

    for( int i=0; i < num_images; i++ ) {
        std::cout << "Integrating " << i << std::endl;

// TODO (dave#4#): Make sure that the file name is fully qualified and if not , make it relative to this path ...
//
//OR
//
//investigate use of fopen() with a path to file.


        // Read it
        uint16_t * depthmap = read_tum_depth_map( g_data[i].file_name, width, height);

        // Set location
        camera.set_pose( g_data[i].ground_truth );

        volume->integrate(depthmap, width, height, camera);
        delete [] depthmap;
    }

    return volume;
}

/**
 * Load the TSDF
 */
phd::TSDFVolume * load_tsdf( const std::string& file_name) {
    phd::TSDFVolume * volume = phd::TSDFVolume::make_volume(phd::TSDFVolume::CPU);

    if (volume->load_from_file( file_name) ) {
        std::cout << "Loaded volume " << volume->size().x() << "x" << volume->size().y() << "x" << volume->size().z() << "  (" << volume->physical_size().x() << "mm x "<< volume->physical_size().y() << "mm x " << volume->physical_size().z() << "mm)"  << std::endl;
    } else {
        std::cout << "Failed to read TSDF" << std::endl;
    }
    return volume;
}

int main( int argc, const char * argv[] ) {
    int retval = 0;
    phd::TSDFVolume *volume = nullptr;

    if( argc == 2 ) {
        std::cout << "Loading TSDF" << std::endl;

        volume = load_tsdf( argv[1]);

    } else if ( argc == 1 ) {
        std::cout << "Make TSDF" << std::endl;
        volume = make_tsdf();
    } else {
        std::cout << "Usage : " << argv[0] << " [file]" << std::endl;
    }

    if( volume ) {
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
        std::cout << "Couldn't make or load volume" << std::endl;
    }


    return retval;
}

