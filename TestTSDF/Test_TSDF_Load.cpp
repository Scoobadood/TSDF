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
#include "TSDFVolume.hpp"
#include "TestHelpers.hpp"

#pragma mark - Construction

TEST( TSDF_Load, givenFileLoadData ) {
    using namespace phd;
    using namespace Eigen;
    
    TSDFVolume volume;
    
    volume.load_from_file( "/Users/Dave/Desktop/512_512_TSDF@3200_3200_6400.txt" );
    
    EXPECT_EQ( volume.size().x(), 512);
    EXPECT_EQ( volume.size().y(), 512);
    EXPECT_EQ( volume.size().z(), 512);
    
    EXPECT_EQ( volume.physical_size().x(), 6400.0f);
    EXPECT_EQ( volume.physical_size().y(), 6400.0f);
    EXPECT_EQ( volume.physical_size().z(), 6400.0f);
    
    uint16_t width = 640;
    uint16_t height = 480;
    
    Vector3f * vertices = new Vector3f[ width * height ];
    Vector3f * normals = new Vector3f[ width * height ];
    
    Vector3f light_source{ 0, 0, 0 };
    
    Camera cam = make_kinect();
    cam.move_to( 0, 3200, 6400 );
    cam.look_at( 3200,3200,3200);
    
    
    volume.raycast(cam, 640, 480, vertices, normals);
    save_normals_as_colour_png("/Users/Dave/Desktop/load_front_left_normals.png", width, height, normals);
    save_rendered_scene_as_png("/Users/Dave/Desktop/load_front_left_render.png", width, height, vertices, normals, cam, light_source);
}



