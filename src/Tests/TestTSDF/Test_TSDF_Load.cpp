//
//  Test_TSDF_Load.cpp
//  KinFu
//
//  Created by Dave on 26/05/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include <fstream>
#include <gtest/gtest.h>
#include <Eigen/Core>

#include "../KinFu/TSDFVolume.hpp"
#include "../KinFu/Utilities/PngWrapper.hpp"
#include "../KinFu/Utilities/DepthMapUtilities.hpp"
#include "../KinFu/Utilities/RenderUtilities.hpp"

#include "TestHelpers.hpp"

#pragma mark - Construction

TEST( TSDF_Load, givenFileLoadData ) {
    using namespace phd;
    using namespace Eigen;

    TSDFVolume * volume = TSDFVolume::make_volume( TSDFVolume::CPU);

    volume->load_from_file( "/Users/Dave/Desktop/512_512_TSDF@3200_3200_6400.txt" );

    EXPECT_EQ( volume->size().x(), 512);
    EXPECT_EQ( volume->size().y(), 512);
    EXPECT_EQ( volume->size().z(), 512);

    EXPECT_EQ( volume->physical_size().x(), 6400.0f);
    EXPECT_EQ( volume->physical_size().y(), 6400.0f);
    EXPECT_EQ( volume->physical_size().z(), 6400.0f);

    uint16_t width = 640;
    uint16_t height = 480;

    Matrix<float, 3, Dynamic> vertices;
    Matrix<float, 3, Dynamic> normals;

    Vector3f light_source{ 0, 0, 0 };

    Camera cam = make_kinect();
    cam.move_to( 0, 3200, 6400 );
    cam.look_at( 3200,3200,3200);

    volume->raycast( width, height, cam, vertices, normals);

    PngWrapper * p;
    p = normals_as_png(width, height, normals);
    p->save_to( "/Users/Dave/Desktop/load_front_left_normals.png");
    delete p;

    p = scene_as_png(width, height, vertices, normals,cam, light_source);
    p->save_to("/Users/Dave/Desktop/load_front_left_render.png");
    delete p;
}



