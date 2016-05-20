//
//  TestCamera.cpp
//  KinFu
//
//  Created by Dave on 19/05/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include <gtest/gtest.h>
#include "Camera.hpp"
#include "TestHelpers.hpp"

#pragma mark - Construction

const float EPS = 1e-6;

TEST( Camera, givenPointWhenInCentreOfImageThenCamPointIsOrigin ) {
    using namespace phd;
    using namespace Eigen;

    Camera cam{ 500, 500, 320, 240 };
    
    Vector2i point{ 320, 240 };
    Vector2f cam_point;
    cam.image_to_camera(point, cam_point);
    
    EXPECT_NEAR( cam_point.x(), 0.0f, EPS );
    EXPECT_NEAR( cam_point.y(), 0.0f, EPS );
}


TEST( Camera, givenPointWhenAtOriginOfImageThenCamPointIs_MM ) {
    using namespace phd;
    using namespace Eigen;
    
    Camera cam{ 500, 500, 320, 240 };
    
    Vector2i point{ 0, 0 };
    Vector2f cam_point;
    cam.image_to_camera(point, cam_point);
    
    EXPECT_NEAR( cam_point.x(), -500.0f, EPS );
    EXPECT_NEAR( cam_point.y(), -500.0f, EPS );
}

TEST( Camera, givenDepthMapThenGenerateNormalMap ) {
    using namespace phd;
    using namespace Eigen;
    
    Camera cam{ 500, 500, 320, 240 };
    
    // Load depth image
    uint32_t width;
    uint32_t height;
    uint16_t * depthmap = read_tum_depth_map("/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.160407.png", width, height);
    

    std::deque<Vector3f> vertices;
    std::deque<Vector3f> normals;
    
    cam.depth_image_to_vertices_and_normals(depthmap, width, height, vertices, normals);
    
    save_normals_as_colour_png("/Users/Dave/Desktop/normals_tum.png", width, height, normals);
    save_rendered_scene_as_png("/Users/Dave/Desktop/render_tum.png", width, height, vertices, normals, Vector3f{1343, 627, 1660}, Vector3f{0,0,100});
    
}

