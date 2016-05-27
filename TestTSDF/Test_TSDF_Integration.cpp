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
#pragma mark - Construction


//TEST( TSDF_Integration, given ) {
//    using namespace phd;
//    using namespace Eigen;
//    
//    // Make volume
//    float vw, vh, vd;
//    TSDFVolume volume = construct_volume(64, 64, 64, 6400, 6400, 6400, vw, vh, vd);
//    
//    // And camera
//    Camera camera = make_kinect();
//    
//    // Load depth image
//    uint32_t width = 640;
//    uint32_t height = 480;
////    uint16_t * depthmap = make_sphere_depth_map(width, height, 50, 2000, 100);
//    uint16_t * depthmap = make_wall_depth_map(width, height, 3500, 400, 1500 );
//    save_png_to_file("/Users/Dave/Desktop/sphere_1_depth_map.png", width, height, depthmap);
//    
//    camera.move_to( 3200, 3200, 6400 );
//    camera.look_at(3200, 3200, 0);
//    volume.integrate(depthmap, width, height, camera);
//    
//    
//    // Go again from right hand side
//    camera.move_to( 6400, 3200, 3200 );
//    camera.look_at(0, 3200, 3200);
//    volume.integrate(depthmap, width, height, camera);
//    
//    
//    // Set up to render
//    Vector3f light_source{ 10000, 10000, 10000 };
//    Vector3f * vertices = new Vector3f[ width * height ];
//    Vector3f * normals  = new Vector3f[ width * height ];
//    
//    // Raycast volume
//    volume.raycast(camera, width, height, vertices, normals);
//    save_normals_as_colour_png("/Users/Dave/Desktop/sphere_1_normals.png", width, height, normals);
//    save_rendered_scene_as_png("/Users/Dave/Desktop/sphere_1_render.png", width, height, vertices, normals, camera.position(), light_source);
//    
//    delete [] vertices;
//    delete [] normals;
//}

TEST( TSDF_Integration, given ) {
    using namespace phd;
    using namespace Eigen;
    
    // Make volume
    float vw, vh, vd;
    TSDFVolume volume = construct_volume(64, 64, 64, 6400, 6400, 6400, vw, vh, vd);

    // And camera
    Camera camera = make_kinect();
    
    // Load depth image
    uint32_t width;
    uint32_t height;
    
    
    std::cout << "Integrating 1" << std::endl;
    
    uint16_t * depthmap = read_tum_depth_map("/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.160407.png", width, height);
    
    camera.move_to( 1343.4, 627.1, 1660.6 );
    camera.look_at( 1331.0, 1416.3, 2274.6 );
    
    volume.integrate(depthmap, width, height, camera);
    delete [] depthmap;
    
    
    
    // Go again
//    std::cout << "Integrating 2" << std::endl;
//    depthmap = read_tum_depth_map("/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.194330.png", width, height);
//    camera.move_to( 1343.6, 626.5, 1652.4 );
//    camera.look_at( 1334.8, 1413.9, 2268.8 );
//    volume.integrate(depthmap, width, height, camera);
//    delete [] depthmap;
    

    // And again
//    depthmap = read_tum_depth_map("/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031104.959750.png", width, height);
//    camera.move_to( 1407.5, 648.8, 1743.8 );
//    camera.look_at( 1385.4, 1538.9, 2199.1 );
//    volume.integrate(depthmap, width, height, camera);
//    delete [] depthmap;
//
//    
//    depthmap = read_tum_depth_map("/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031108.935116.png", width, height);
//    camera.move_to( 1290.4, 960.4, 1619.6 );
//    camera.look_at( 1255.9, 1730.2, 2256.9 );
//        volume.integrate(depthmap, width, height, camera);
//    delete [] depthmap;
//
//    
//    depthmap = read_tum_depth_map("/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031123.015462.png", width, height);
//    camera.move_to( 1292.0, 551.7, 1493.8 );
//    camera.look_at( 1324.6, 1244.3, 2214.3 );
//        volume.integrate(depthmap, width, height, camera);
//    delete [] depthmap;
//    

 
    
    std::cout << "Saving" << std::endl;
    volume.save_to_file( "/Users/Dave/Desktop/TSDF.dat");
    
    // Set up camera
    std::cout << "Rendering" << std::endl;
    camera.move_to( 1343.4, 627.1, 1660.6 );
    camera.look_at( 1331.0, 1416.3, 2274.6 );
    Vector3f lookat_vec{ 13331 - 1343.4, 1416.3-627.1, 2274.6 - 1660.6};
    camera.move_to( 1343.4 + lookat_vec.x() * 3, 627.1+ lookat_vec.y() * 3, 1660.6 + lookat_vec.z() * 3);
    camera.look_at( 1331.0, 1416.3, 2274.6 );
    
    
    Vector3f light_source{ 1500, 1000, 1600 };
    Vector3f * vertices = new Vector3f[ width * height ];
    Vector3f * normals  = new Vector3f[ width * height ];

    // Raycast volume
    volume.raycast(camera, width, height, vertices, normals);
    save_normals_as_colour_png("/Users/Dave/Desktop/normals.png", width, height, normals);
    save_rendered_scene_as_png("/Users/Dave/Desktop/render.png", width, height, vertices, normals, camera, light_source);

    delete [] vertices;
    delete [] normals;
}
