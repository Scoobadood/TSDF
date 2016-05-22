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

#pragma mark - Construction

TEST( TSDF_Integration, given ) {
    using namespace phd;
    using namespace Eigen;
    
    // Make volume
    float vw, vh, vd;
    TSDFVolume volume = construct_volume(32, 32, 32, 3200, 3200, 3200, vw, vh, vd);

    // And camera
    Camera camera{ 500, 500, 320, 240 };
    


    // Load depth image
    uint32_t width;
    uint32_t height;
    uint16_t * depthmap = read_tum_depth_map("/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.160407.png", width, height);
    
    // Pose camera
    Quaternion<float> q{0.6583, 0.6112, -0.2938, -0.3266};
    Matrix3f r = q.toRotationMatrix();
    Matrix4f pose = Matrix4f::Zero();
    pose.block(0,0,3,3) = r;
    pose(0,3) = 1343.4f;
    pose(1,3) = 627.1f;
    pose(2,3) = 1660.6f;
    pose(3,3) = 1.0f;
    camera.set_pose(pose);
    
    volume.integrate(depthmap, width, height, camera);
    
    // Set up camera
    Vector3f light_source{ 1600, 1600, -4000 };
    Vector3f * vertices = new Vector3f[ width * height ];
    Vector3f * normals  = new Vector3f[ width * height ];

    // Raycast volume
    volume.raycast(camera, 640, 480, vertices, normals);
    save_normals_as_colour_png("/Users/Dave/Desktop/tum_1_normals.png", width, height, normals);
    save_rendered_scene_as_png("/Users/Dave/Desktop/tum_1_render.png", width, height, vertices, normals, camera.position(), light_source);


    // Integrate second image
    depthmap = read_tum_depth_map("/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.194330.png", width, height);
    
    q= Quaternion<float>{0.6564f, 0.6139f, -0.2963f, -0.3231f};
    r = q.toRotationMatrix();
    pose = Matrix4f::Zero();
    pose.block(0,0,3,3) = r;
    pose(0,3) = 1335.2;
    pose(1,3) = 626.1 ;
    pose(2,3) = 1651.9;
    pose(3,3) = 1.0f;
    camera.set_pose(pose);
    
    
    volume.integrate(depthmap, width, height, camera);

    // Raycast again
    volume.raycast(camera, 640, 480, vertices, normals);
    save_normals_as_colour_png("/Users/Dave/Desktop/tum_2_normals.png", width, height, normals);
    save_rendered_scene_as_png("/Users/Dave/Desktop/tum_2_render.png", width, height, vertices, normals, camera.position(), light_source);
    
    
    delete [] vertices;
    delete [] normals;
}
