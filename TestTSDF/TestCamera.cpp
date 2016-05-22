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

phd::Camera make_kinect( ) {
    return phd::Camera{ 585, 585, 316, 247 };
}

TEST( Camera, givenPointWhenInCentreOfImageThenCamPointIsOrigin ) {
    using namespace phd;
    using namespace Eigen;

    Camera cam = make_kinect();
    
    Vector2i point{ 320, 240 };
    Vector2f cam_point;
    cam.image_to_camera(point, cam_point);
    
    EXPECT_NEAR( cam_point.x(), 0.0f, EPS );
    EXPECT_NEAR( cam_point.y(), 0.0f, EPS );
}


TEST( Camera, givenPointWhenAtOriginOfImageThenCamPointIs_MM ) {
    using namespace phd;
    using namespace Eigen;
    
    Camera cam = make_kinect();
    
    Vector2i point{ 0, 0 };
    Vector2f cam_point;
    cam.image_to_camera(point, cam_point);
    
    EXPECT_NEAR( cam_point.x(), 320.0f/500.0f, EPS );
    EXPECT_NEAR( cam_point.y(), 240.0f/500.0f, EPS );
}

TEST( Camera, givenPositionThenPoseIsUpdatedCorrectly ) {
    using namespace phd;
    using namespace Eigen;
    
    Camera cam = make_kinect();
    
    Vector2i point{ 0, 0 };
    Vector2f cam_point;
    cam.image_to_camera(point, cam_point);
    
    EXPECT_NEAR( cam_point.x(), 320.0f/500.0f, EPS );
    EXPECT_NEAR( cam_point.y(), 240.0f/500.0f, EPS );
}

TEST( Camera, givenDefaultConstructorThenPoseIsIdentity ) {
    using namespace phd;
    using namespace Eigen;
    
    Camera cam = make_kinect();
    EXPECT_EQ( cam.pose(), Matrix4f::Identity() );
}

TEST( Camera, givenMoveToThenPoseUpdatesCorrcetly ) {
    using namespace phd;
    using namespace Eigen;
    
    Camera cam = make_kinect();
    EXPECT_EQ( cam.pose(), Matrix4f::Identity() );
    
    cam.move_to(Vector3f { 100.0, 200.0, 300.0 } );
    
    Matrix4f pose = cam.pose();
    EXPECT_EQ( pose(0,3), 100 );
    EXPECT_EQ( pose(1,3), 200 );
    EXPECT_EQ( pose(2,3), 300 );
}



TEST( Camera, givenLookAtOriginWhenInFrontThenPoseLooksBack ) {
    using namespace phd;
    using namespace Eigen;
    
    Camera cam = make_kinect();
    Vector3f camera_postion{ 0, 0, 100 };
    
    cam.move_to(camera_postion );
    cam.look_at(Vector3f::Zero() );
    
    Matrix4f pose = cam.pose();
    
    Matrix4f expected = make_y_axis_rotation(0, camera_postion);
    for( int i=0; i<16; i++ ) {
        int r = i%4;
        int c = i/4;
        EXPECT_NEAR(pose(i), expected(i), EPS ) << "Failed at (" << r << ", " << c << ")" << std::endl;
    }
}


TEST( Camera, givenLookAtOriginWhenRightOfOriginThenPoseLooksLeft ) {
    using namespace phd;
    using namespace Eigen;
    
    Camera cam = make_kinect();
    Vector3f camera_postion{ 100, 0, 0 };
    
    cam.move_to(camera_postion );
    cam.look_at(Vector3f::Zero() );
    Matrix4f pose = cam.pose();
    

    Matrix4f expected = make_y_axis_rotation(M_PI_2, camera_postion);
    for( int i=0; i<16; i++ ) {
        int r = i%4;
        int c = i/4;
        EXPECT_NEAR(pose(i), expected(i), EPS ) << "Failed at (" << r << ", " << c << ")" << std::endl;
    }
}

TEST( Camera, givenLookAtOriginWhenLeftOfOriginThenPoseLooksRight ) {
    using namespace phd;
    using namespace Eigen;
    
    Camera cam = make_kinect();
    Vector3f camera_postion{ -100, 0, 0 };
    
    cam.move_to(camera_postion );
    cam.look_at(Vector3f::Zero() );
    Matrix4f pose = cam.pose();
    
    Matrix4f expected = make_y_axis_rotation(-M_PI_2, camera_postion);
    for( int i=0; i<16; i++ ) {
        int r = i%4;
        int c = i/4;
        EXPECT_NEAR(pose(i), expected(i), EPS ) << "Failed at (" << r << ", " << c << ")" << std::endl;
    }
}

TEST( Camera, givenLookAtOriginWhenBehindOriginThenPoseLooksFront ) {
    using namespace phd;
    using namespace Eigen;
    
    Camera cam = make_kinect();
    Vector3f camera_postion{ 0, 0, -100 };
    
    cam.move_to(camera_postion );
    cam.look_at(Vector3f::Zero() );
    Matrix4f pose = cam.pose();
    
    Matrix4f expected = make_y_axis_rotation(M_PI, camera_postion);
    for( int i=0; i<16; i++ ) {
        int r = i%4;
        int c = i/4;
        EXPECT_NEAR(pose(i), expected(i), EPS ) << "Failed at (" << r << ", " << c << ")" << std::endl;
    }
}

TEST( Camera, givenLookAtOriginWhenAboveThenPoseLooksDown ) {
    using namespace phd;
    using namespace Eigen;
    
    Camera cam = make_kinect();
    Vector3f camera_postion{ 0, 100, 0 };
    
    cam.move_to(camera_postion );
    cam.look_at(Vector3f::Zero() );
    Matrix4f pose = cam.pose();
    
    Matrix4f expected = make_x_axis_rotation(-M_PI_2, camera_postion);
    for( int i=0; i<16; i++ ) {
        int r = i / 4;
        int c = i % 4;
        EXPECT_NEAR( pose(i), expected(i), EPS ) << "Incorrect value for index (" << r << ", " << c << ")";
    }
}

TEST( Camera, givenLookAtOriginWhenBelowThenPoseLooksUp ) {
    using namespace phd;
    using namespace Eigen;
    
    Camera cam = make_kinect();
    Vector3f camera_postion{ 0, -100, 0 };
    
    cam.move_to(camera_postion );
    cam.look_at(Vector3f::Zero() );
    Matrix4f pose = cam.pose();
    
    Matrix4f expected = make_x_axis_rotation(M_PI_2, camera_postion);
    for( int i=0; i<16; i++ ) {
        int r = i / 4;
        int c = i % 4;
        EXPECT_NEAR( pose(i), expected(i), EPS ) << "Incorrect value for index (" << r << ", " << c << ")";
    }
}



TEST( Camera, givenCameraThenSettingPoseWorks ) {
    using namespace phd;
    using namespace Eigen;
    
    Camera cam = make_kinect();
    
    Matrix4f new_pose;
    new_pose << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16;
    
    cam.set_pose(new_pose);
    
    EXPECT_EQ( cam.pose(), new_pose );
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
    save_rendered_scene_as_png("/Users/Dave/Desktop/render_tum.png", width, height, vertices, normals, Vector3f{0, 0, 0}, Vector3f{1000, 0, 1500});
    
}


TEST( Camera, givenWorldCoordinateWhenCameraAtOriginFacingNegativeZThenCameraCoordinateIsCorrect ) {
    using namespace phd;
    using namespace Eigen;
    
    Camera cam{ 500, 500, 320, 240 };
    
    cam.move_to( 0, 0, 0);
    cam.look_at( 0, 0, -1 );
    
    Vector3f x{ 1, 0, 0 };
    Vector3f world;
    cam.camera_to_world(x, world);
    
    EXPECT_NEAR( world.x(), x.x(), EPS );
    EXPECT_NEAR( world.y(), x.y(), EPS );
    EXPECT_NEAR( world.z(), x.z(), EPS );
}


