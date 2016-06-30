//
//  TestCamera.cpp
//  KinFu
//
//  Created by Dave on 19/05/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include <gtest/gtest.h>
#include "../KinFu/Camera.hpp"
#include "../KinFu/Utilities/DepthMapUtilities.hpp"
#include "../KinFu/Utilities/RenderUtilities.hpp"

#include "TestHelpers.hpp"

#pragma mark - Construction

const float EPS = 1e-6;

#pragma mark - Coordinate Transforms
#pragma mark World to Camera
static Eigen::Vector3f world_coordinates[] = {
    Eigen::Vector3f{ 0, 0, 0 },
    Eigen::Vector3f{ 100, 0, 0 },
    Eigen::Vector3f{ 100, 100, 0 },
    Eigen::Vector3f{ 0, 100, 0 },
    Eigen::Vector3f{ 0, 100, 100 },
    Eigen::Vector3f{ 0, 0, 100 },
    Eigen::Vector3f{ 100, 0, 100 },
    Eigen::Vector3f{ 100, 100, 100 } };

TEST( Camera, givenWorldCoordinateWhenCameraAtOriginFacingNegativeZThenCameraCoordinateIsCorrect ) {
    using namespace phd;
    using namespace Eigen;

    Camera cam = make_kinect();

    Vector3f camera_coordinate;
    for( int i=0; i< 8; i++ ) {
        camera_coordinate = cam.world_to_camera(world_coordinates[i]);

        EXPECT_NEAR( world_coordinates[i].x(), camera_coordinate.x(), EPS );
        EXPECT_NEAR( world_coordinates[i].y(), camera_coordinate.y(), EPS );
        EXPECT_NEAR( world_coordinates[i].z(), camera_coordinate.z(), EPS );
    }
}

TEST( Camera, givenWorldCoordinateWhenCameraAtOriginFacingNegativeXThenCameraCoordinateIsCorrect ) {
    using namespace phd;
    using namespace Eigen;

    Camera cam = make_kinect();
    cam.look_at(-1.0f, 0.0f, 0.0f );

    Vector3f camera_coordinate;
    for( int i=0; i< 8; i++ ) {
        camera_coordinate = cam.world_to_camera(world_coordinates[i]);

        EXPECT_NEAR( camera_coordinate.x(), -world_coordinates[i].z(), EPS );
        EXPECT_NEAR( camera_coordinate.y(), world_coordinates[i].y(), EPS );
        EXPECT_NEAR( camera_coordinate.z(), world_coordinates[i].x(), EPS );
    }
}

TEST( Camera, givenWorldCoordinateWhenCameraAtOriginFacingNegativeYThenCameraCoordinateIsCorrect ) {
    using namespace phd;
    using namespace Eigen;

    Camera cam = make_kinect();
    cam.look_at(0.0f, -1.0f, 0.0f );

    Vector3f camera_coordinate;
    for( int i=0; i< 8; i++ ) {
        camera_coordinate = cam.world_to_camera(world_coordinates[i]);

        EXPECT_NEAR( camera_coordinate.x(), world_coordinates[i].x(), EPS );
        EXPECT_NEAR( camera_coordinate.y(), -world_coordinates[i].z(), EPS );
        EXPECT_NEAR( camera_coordinate.z(), world_coordinates[i].y(), EPS );
    }
}

TEST( Camera, givenWorldCoordinateWhenCameraAtOriginFacingPositiveYThenCameraCoordinateIsCorrect ) {
    using namespace phd;
    using namespace Eigen;

    Camera cam = make_kinect();
    cam.look_at(0.0f, 1.0f, 0.0f );

    Vector3f camera_coordinate;
    for( int i=0; i< 8; i++ ) {
        camera_coordinate = cam.world_to_camera(world_coordinates[i]);

        EXPECT_NEAR( camera_coordinate.x(), world_coordinates[i].x(), EPS );
        EXPECT_NEAR( camera_coordinate.y(), world_coordinates[i].z(), EPS );
        EXPECT_NEAR( camera_coordinate.z(), -world_coordinates[i].y(), EPS );
    }
}

TEST( Camera, givenWorldCoordinateWhenCameraAtOriginFacingPositiveXThenCameraCoordinateIsCorrect ) {
    using namespace phd;
    using namespace Eigen;

    Camera cam = make_kinect();
    cam.look_at(1.0f, 0.0f, 0.0f );

    Vector3f camera_coordinate;
    for( int i=0; i< 8; i++ ) {
        camera_coordinate = cam.world_to_camera(world_coordinates[i]);

        EXPECT_NEAR( camera_coordinate.x(), world_coordinates[i].z(), EPS );
        EXPECT_NEAR( camera_coordinate.y(), world_coordinates[i].y(), EPS );
        EXPECT_NEAR( camera_coordinate.z(), -world_coordinates[i].x(), EPS );
    }
}

TEST( Camera, givenWorldCoordinateWhenCameraAtOriginFacingPositiveZThenCameraCoordinateIsCorrect ) {
    using namespace phd;
    using namespace Eigen;

    Camera cam = make_kinect();
    cam.look_at(0.0f, 0.0f, 1.0f );

    Vector3f camera_coordinate;
    for( int i=0; i< 8; i++ ) {
        camera_coordinate = cam.world_to_camera(world_coordinates[i]);

        EXPECT_NEAR( camera_coordinate.x(), -world_coordinates[i].x(), EPS );
        EXPECT_NEAR( camera_coordinate.y(), world_coordinates[i].y(), EPS );
        EXPECT_NEAR( camera_coordinate.z(), -world_coordinates[i].z(), EPS );
    }
}

#pragma Translation Only
TEST( Camera, givenWorldCoordinateWhenCameraOnXAxisFacingNegativeZThenCameraCoordinateIsCorrect ) {
    using namespace phd;
    using namespace Eigen;

    Camera cam = make_kinect();
    cam.move_to(100, 0, 0);

    Vector3f camera_coordinate;
    for( int i=0; i< 8; i++ ) {
        camera_coordinate = cam.world_to_camera(world_coordinates[i]);

        EXPECT_NEAR( camera_coordinate.x(), world_coordinates[i].x()-100, EPS );
        EXPECT_NEAR( camera_coordinate.y(), world_coordinates[i].y(), EPS );
        EXPECT_NEAR( camera_coordinate.z(), world_coordinates[i].z(), EPS );
    }
}

TEST( Camera, givenWorldCoordinateWhenCameraOnYAxisFacingNegativeZThenCameraCoordinateIsCorrect ) {
    using namespace phd;
    using namespace Eigen;

    Camera cam = make_kinect();
    cam.move_to(0, 100, 0);

    Vector3f camera_coordinate;
    for( int i=0; i< 8; i++ ) {
        camera_coordinate = cam.world_to_camera(world_coordinates[i]);

        EXPECT_NEAR( camera_coordinate.x(), world_coordinates[i].x(), EPS );
        EXPECT_NEAR( camera_coordinate.y(), world_coordinates[i].y()-100, EPS );
        EXPECT_NEAR( camera_coordinate.z(), world_coordinates[i].z(), EPS );
    }
}
TEST( Camera, givenWorldCoordinateWhenCameraOnZAxisFacingNegativeZThenCameraCoordinateIsCorrect ) {
    using namespace phd;
    using namespace Eigen;

    Camera cam = make_kinect();
    cam.move_to(0, 0, 100);

    Vector3f camera_coordinate;
    for( int i=0; i< 8; i++ ) {
        camera_coordinate = cam.world_to_camera(world_coordinates[i]);

        EXPECT_NEAR( camera_coordinate.x(), world_coordinates[i].x(), EPS );
        EXPECT_NEAR( camera_coordinate.y(), world_coordinates[i].y(), EPS );
        EXPECT_NEAR( camera_coordinate.z(), world_coordinates[i].z()-100, EPS );
    }
}

#pragma Translation and Rotation through unit cube vertices
TEST( Camera, givenWorldCoordinateWhenCameraOn_m1_m1_m1_ThenCameraCoordinateCorrect ) {
    using namespace phd;
    using namespace Eigen;

    Camera cam = make_kinect();
    cam.move_to(-1, -1, -1);
    cam.look_at(0,0,0);

    Vector3f camera_coordinate;
    // Only do origin
    for( int i=0; i< 0; i++ ) {
        camera_coordinate = cam.world_to_camera(world_coordinates[i]);

        EXPECT_NEAR( camera_coordinate.x(), 0, EPS );
        EXPECT_NEAR( camera_coordinate.y(), 0, EPS );
        EXPECT_NEAR( camera_coordinate.z(), -M_SQRT2, EPS );
    }
}





#pragma mark - pixel to image plane

TEST( Camera, givenPointWhenInCentreOfImageThenCamPointIsOrigin ) {
    using namespace phd;
    using namespace Eigen;

    Camera cam{ 500.0f, 500.0f, 320.0f, 240.0f };

    Vector2i point{ 320, 240 };
    Vector2f cam_point;
    cam_point = cam.pixel_to_image_plane(point);

    EXPECT_NEAR( cam_point.x(), 0.0f, EPS );
    EXPECT_NEAR( cam_point.y(), 0.0f, EPS );
}


TEST( Camera, givenPixelAt_0_0_ConvertToCamAndBackIsOK ) {
    using namespace phd;
    using namespace Eigen;

    Camera cam = make_kinect();

    Vector2i initial_pixel_coordinate{0,0};
    Vector2f image_plane_coord;
    Vector2i final_pixel_coordinate;

    image_plane_coord = cam.pixel_to_image_plane(initial_pixel_coordinate);
    EXPECT_NEAR(image_plane_coord.x(), -0.5, 0.1 );
    EXPECT_NEAR(image_plane_coord.y(), -0.5, 0.11 );

    final_pixel_coordinate = cam.image_plane_to_pixel( image_plane_coord);

    EXPECT_EQ(initial_pixel_coordinate.x(), final_pixel_coordinate.x());
    EXPECT_EQ(initial_pixel_coordinate.y(), final_pixel_coordinate.y());
}

TEST( Camera, givenPixelAt_MaxX_0_ConvertToCamAndBackIsOK ) {
    using namespace phd;
    using namespace Eigen;

    Camera cam = make_kinect();

    Vector2i initial_pixel_coordinate{640,0};
    Vector2f image_plane_coord;
    Vector2i final_pixel_coordinate;

    image_plane_coord = cam.pixel_to_image_plane(initial_pixel_coordinate);
    EXPECT_NEAR(image_plane_coord.x(), 0.5, 0.1 );
    EXPECT_NEAR(image_plane_coord.y(), -0.5, 0.11 );

    final_pixel_coordinate = cam.image_plane_to_pixel( image_plane_coord);
    EXPECT_EQ(initial_pixel_coordinate.x(), final_pixel_coordinate.x());
    EXPECT_EQ(initial_pixel_coordinate.y(), final_pixel_coordinate.y());
}

TEST( Camera, givenPixelAt_0_MaxY_ConvertToCamAndBackIsOK ) {
    using namespace phd;
    using namespace Eigen;

    Camera cam = make_kinect();

    Vector2i initial_pixel_coordinate{0, 480};
    Vector2f image_plane_coord;
    Vector2i final_pixel_coordinate;

    image_plane_coord = cam.pixel_to_image_plane(initial_pixel_coordinate);
    EXPECT_NEAR(image_plane_coord.x(), -0.5, 0.1 );
    EXPECT_NEAR(image_plane_coord.y(), 0.5, 0.11 );

    final_pixel_coordinate = cam.image_plane_to_pixel( image_plane_coord);
    EXPECT_EQ(initial_pixel_coordinate.x(), final_pixel_coordinate.x());
    EXPECT_EQ(initial_pixel_coordinate.y(), final_pixel_coordinate.y());
}

TEST( Camera, givenPixelAt_MaxX_MaxY_ConvertToCamAndBackIsOK ) {
    using namespace phd;
    using namespace Eigen;

    Camera cam = make_kinect();

    Vector2i initial_pixel_coordinate{640,480};
    Vector2f image_plane_coord;
    Vector2i final_pixel_coordinate;

    image_plane_coord = cam.pixel_to_image_plane(initial_pixel_coordinate);
    EXPECT_NEAR(image_plane_coord.x(), 0.5, 0.1 );
    EXPECT_NEAR(image_plane_coord.y(), 0.5, 0.11 );

    final_pixel_coordinate = cam.image_plane_to_pixel( image_plane_coord);
    EXPECT_EQ(initial_pixel_coordinate.x(), final_pixel_coordinate.x());
    EXPECT_EQ(initial_pixel_coordinate.y(), final_pixel_coordinate.y());
}


#pragma mark - Pose

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

    Camera cam = make_kinect();

    // Load depth image
    uint32_t width;
    uint32_t height;
    uint16_t * depthmap = read_tum_depth_map("/mnt/hgfs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.160407.png", width, height);


    Matrix<float, 3, Dynamic> vertices;
    Matrix<float, 3, Dynamic> normals;

    cam.depth_image_to_vertices_and_normals(depthmap, width, height, vertices, normals);

    save_normals_as_colour_png("/Users/Dave/Desktop/normals_tum.png", width, height, normals);
    save_rendered_scene_as_png("/Users/Dave/Desktop/render_tum.png", width, height, vertices, normals, cam, Vector3f{10000, 10000, 1000});

}
