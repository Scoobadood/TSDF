//
//  TestTSDF_Integration.cpp
//  KinFu
//
//  Created by Dave on 21/05/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include <gtest/gtest.h>
#include "TSDFVolume.hpp"
#include "Camera.hpp"
#include "TestHelpers.hpp"
#include "PngUtilities.hpp"
#include "BilateralFilter.hpp"
#pragma mark - Construction




struct {
    std::string     file_name;
    float           ground_truth[7];
}  g_data[] =
{
//        { "/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg2_xyz/depth/1311867170.450076.png", {0.1554, -1.1425, 1.3593, -0.5691, 0.6454, -0.3662, 0.3541}},
    { "/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.160407.png", {1.344379, 0.627206, 1.661754, 0.658249, 0.611043, -0.294444, -0.326553}},
    { "/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.194330.png", {1.343641, 0.626458, 1.652408, 0.657327, 0.613265, -0.295150, -0.323593}},
    { "/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.226738.png", {1.338382, 0.625665, 1.641460, 0.657713, 0.615255, -0.294626, -0.319485}},
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



void helper_move_look( float vars[7], Eigen::Vector3f & move_to, Eigen::Vector3f & look_at ) {
    move_to.x() = vars[0];
    move_to.y() = vars[1];
    move_to.z() = vars[2];
    move_to = move_to * 1000;
    
    Eigen::Quaternionf qq{ vars[6], vars[3], vars[4], vars[5] };
    
    Eigen::Matrix3f r = qq.toRotationMatrix();
    Eigen::Matrix<float,1,3> c{ 0.0, 0.0, -1.0 };
    
    c = c * r;
    
    look_at = move_to + ( 1000 * c.transpose() );
    
    std::cout << "Move To : (" << move_to.x() << ", " << move_to.y() << ", " << move_to.z() << ")" << std::endl;
    std::cout << "Look At : (" << look_at.x() << ", " << look_at.y() << ", " << look_at.z() << ")" << std::endl;
    
    
}

TEST( TSDF_Integration, givenManyImages ) {
    // Set up
    using namespace phd;
    using namespace Eigen;
    
    /*** SET PARAMETERS HERE ***/
    
    uint16_t voxels = 512;
    uint16_t num_images = 20;
    bool     save = true;
    bool     raycast = true;
    bool     filter = false;
    
    
    // Make volume
    float vw, vh, vd;
    TSDFVolume volume = construct_volume(voxels, voxels, voxels, 5120, 5120, 5120, vw, vh, vd);
    
    // And camera
    Camera camera = make_kinect();
    
    // Load depth image
    uint32_t width{0};
    uint32_t height{0};
    Vector3f camera_location;
    Vector3f camera_focus;
    
    
    BilateralFilter bf{ 2, 2};
    
    for( int i=0; i < num_images; i++ ) {
        std::cout << "Integrating " << i << std::endl;
        
        // Read it
        uint16_t * depthmap = read_tum_depth_map( g_data[i].file_name, width, height);
        
        
        // Filter it (optionally)
        if( filter ) {
            std::cout << "Filtering" << std::endl;
            bf.filter(depthmap, width, height);
        }
        
        
        // Set location
        helper_move_look(g_data[i].ground_truth, camera_location, camera_focus);
        
        camera.move_to( camera_location.x(), camera_location.y(), camera_location.z() );
        camera.look_at( camera_focus.x(), camera_focus.y(), camera_focus.z() );
        
        volume.integrate(depthmap, width, height, camera);
        delete [] depthmap;
    }
    
    
    // Now save ...
    if( save ) {
        std::cout << "Saving" << std::endl;
        std::ostringstream file_name;
        file_name << "/Users/Dave/Desktop/TSDF_" << voxels << ".txt";
        volume.save_to_file( file_name.str());
    }
    
    
    // ... and render ...
    if( raycast ) {
        if( (width > 0) && (height > 0 ) ){
            Vector3f light_source{ 10000, 6600, -10000 };
            Vector3f * vertices = new Vector3f[ width * height ];
            Vector3f * normals  = new Vector3f[ width * height ];
            
            
            std::cout << "Rendering" << std::endl;
            
            // Set location
            helper_move_look(g_data[0].ground_truth, camera_location, camera_focus);
            camera.move_to( camera_location.x(), camera_location.y(), camera_location.z() );
            camera.look_at( camera_focus.x(), camera_focus.y(), camera_focus.z() );
            
            // Raycast volume
            volume.raycast(camera, width, height, vertices, normals);
            
            save_normals_as_colour_png("/Users/Dave/Desktop/normals.png", width, height, normals);
            save_rendered_scene_as_png("/Users/Dave/Desktop/vertices.png", width, height, vertices, normals, camera, light_source);
            
            delete [] vertices;
            delete[] normals;
        } else {
            std::cerr << "Width or height not set. Can't render" << std::endl;
        }
    }
}