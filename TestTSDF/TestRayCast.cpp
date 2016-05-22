//
//  TestRayCast.cpp
//  KinFu
//
//  Created by Dave on 14/05/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include <stdio.h>
#include <gtest/gtest.h>
#include "TSDFVolume.hpp"
#include "PngUtilities.hpp"
#include "TestHelpers.hpp"

#define EPS 1e-6
#pragma mark - Grid Entry Points

#pragma mark Misses
TEST( TSDF_Raycasting, givenRayToLeftOfVolumeWhenIntersectingThenReturnsFalse ) {
    using namespace phd;
    using namespace Eigen;
    
    TSDFVolume volume{ Vector3i{ 10, 10, 10 },Vector3f{ 300, 300, 300 } };
    
    // Ray to left of volume
    Vector3f origin{ -10, 150, -150 };
    Vector3f direction{ 0.0, 0.0, 1.0 };
    Vector3f actual_entry_point;
    float actual_t;
    bool is = volume.is_intersected_by_ray_2( origin , direction, actual_entry_point, actual_t );

    EXPECT_FALSE( is );
}

TEST( TSDF_Raycasting, givenRayToRightOfVolumeWhenIntersectingThenReturnsFalse ) {
    using namespace phd;
    using namespace Eigen;
    
    TSDFVolume volume{ Vector3i{ 10, 10, 10 },Vector3f{ 300, 300, 300 } };
    
    // Ray to right of volume
    Vector3f origin{ 310, 150, -150 };
    Vector3f direction{ 0.0, 0.0, 1.0 };
    Vector3f actual_entry_point;
    float actual_t;
    bool is = volume.is_intersected_by_ray_2( origin , direction, actual_entry_point, actual_t );
    
    EXPECT_FALSE( is );
}

TEST( TSDF_Raycasting, givenRayToTopOfVolumeWhenIntersectingThenReturnsFalse ) {
    using namespace phd;
    using namespace Eigen;
    
    TSDFVolume volume{ Vector3i{ 10, 10, 10 },Vector3f{ 300, 300, 300 } };
    
    // Ray to left of volume
    Vector3f origin{ 150, 310, -150 };
    Vector3f direction{ 0.0, 0.0, 1.0 };
    Vector3f actual_entry_point;
    float actual_t;
    bool is = volume.is_intersected_by_ray_2( origin , direction, actual_entry_point, actual_t );
    
    EXPECT_FALSE( is );
}

TEST( TSDF_Raycasting, givenRayToBottomOfVolumeWhenIntersectingThenReturnsFalse ) {
    using namespace phd;
    using namespace Eigen;
    
    TSDFVolume volume{ Vector3i{ 10, 10, 10 },Vector3f{ 300, 300, 300 } };
    
    // Ray to left of volume
    Vector3f origin{ 150, -10, -150 };
    Vector3f direction{ 0.0, 0.0, 1.0 };
    Vector3f actual_entry_point;
    float actual_t;
    bool is = volume.is_intersected_by_ray_2( origin , direction, actual_entry_point, actual_t );
    
    EXPECT_FALSE( is );
}

TEST( TSDF_Raycasting, givenRayToFrontOfVolumeWhenIntersectingThenReturnsFalse ) {
    using namespace phd;
    using namespace Eigen;
    
    TSDFVolume volume{ Vector3i{ 10, 10, 10 },Vector3f{ 300, 300, 300 } };
    
    // Ray to left of volume
    Vector3f origin{ 150, 150, 310 };
    Vector3f direction{ 0.0, 1.0, 0.0 };
    Vector3f actual_entry_point;
    float actual_t;
    bool is = volume.is_intersected_by_ray_2( origin , direction, actual_entry_point, actual_t );
    
    EXPECT_FALSE( is );
}

TEST( TSDF_Raycasting, givenRayToBackOfVolumeWhenIntersectingThenReturnsFalse ) {
    using namespace phd;
    using namespace Eigen;
    
    TSDFVolume volume{ Vector3i{ 10, 10, 10 },Vector3f{ 300, 300, 300 } };
    
    // Ray to left of volume
    Vector3f origin{ 150, 150, -10 };
    Vector3f direction{ 0.0, 1.0, 0.0 };
    Vector3f actual_entry_point;
    float actual_t;
    bool is = volume.is_intersected_by_ray_2( origin , direction, actual_entry_point, actual_t );
    
    EXPECT_FALSE( is );
}

#pragma mark Front hits
TEST( TSDF_Raycasting, givenRayFrontToBackOfVolumeWhenIntersectingThenReturnsTrue ) {
    using namespace phd;
    using namespace Eigen;
    
    TSDFVolume volume{ Vector3i{ 10, 10, 10 },Vector3f{ 300, 300, 300 } };
    
    // Ray to left of volume
    for( int x=0; x < 10; x++ ) {
        for( int y=0; y< 10; y++ ) {
            float gx = x * 30 + 15;
            float gy = y * 30 + 15;
            
            Vector3f origin{ gx, gy, 310 };
            Vector3f direction{ 0.0, 0.0, -1.0 };
        
            Vector3f actual_entry_point;
            float actual_t;
            bool is = volume.is_intersected_by_ray_2( origin , direction, actual_entry_point, actual_t );
        
    
            EXPECT_TRUE( is );
            EXPECT_NEAR(actual_t, 10, FLT_EPSILON);
            EXPECT_NEAR(actual_entry_point.x(), gx, FLT_EPSILON);
            EXPECT_NEAR(actual_entry_point.y(), gy, FLT_EPSILON);
            EXPECT_NEAR(actual_entry_point.z(), 300.0, FLT_EPSILON);
        }
    }
}
TEST( TSDF_Raycasting, givenRayBackToFrontOfVolumeWhenIntersectingThenReturnsTrue ) {
    using namespace phd;
    using namespace Eigen;
    
    TSDFVolume volume{ Vector3i{ 10, 10, 10 },Vector3f{ 300, 300, 300 } };
    
    // Ray to left of volume
    for( int x=0; x < 10; x++ ) {
        for( int y=0; y< 10; y++ ) {
            float gx = x * 30 + 15;
            float gy = y * 30 + 15;
            
            Vector3f origin{ gx, gy, -10 };
            Vector3f direction{ 0.0, 0.0, 1.0 };
            
            Vector3f actual_entry_point;
            float actual_t;
            bool is = volume.is_intersected_by_ray_2( origin , direction, actual_entry_point, actual_t );
            
            
            EXPECT_TRUE( is );
            EXPECT_NEAR(actual_t, 10, FLT_EPSILON);
            EXPECT_NEAR(actual_entry_point.x(), gx, FLT_EPSILON);
            EXPECT_NEAR(actual_entry_point.y(), gy, FLT_EPSILON);
            EXPECT_NEAR(actual_entry_point.z(), 0.0, FLT_EPSILON);
        }
    }
}

TEST( TSDF_Raycasting, givenRayLeftToRightOfVolumeWhenIntersectingThenReturnsTrue ) {
    using namespace phd;
    using namespace Eigen;
    
    TSDFVolume volume{ Vector3i{ 10, 10, 10 },Vector3f{ 300, 300, 300 } };
    
    // Ray to left of volume
    for( int z=0; z < 10; z++ ) {
        for( int y=0; y< 10; y++ ) {
            float gz = z * 30 + 15;
            float gy = y * 30 + 15;
            
            Vector3f origin{ -10, gy, gz };
            Vector3f direction{ 1.0, 0.0, 0.0 };
            
            Vector3f actual_entry_point;
            float actual_t;
            bool is = volume.is_intersected_by_ray_2( origin , direction, actual_entry_point, actual_t );
            
            
            EXPECT_TRUE( is );
            EXPECT_NEAR(actual_t, 10, FLT_EPSILON);
            EXPECT_NEAR(actual_entry_point.x(), 0.0, FLT_EPSILON);
            EXPECT_NEAR(actual_entry_point.y(), gy, FLT_EPSILON);
            EXPECT_NEAR(actual_entry_point.z(), gz, FLT_EPSILON);
        }
    }
}
TEST( TSDF_Raycasting, givenRayRightToLeftOfVolumeWhenIntersectingThenReturnsTrue ) {
    using namespace phd;
    using namespace Eigen;
    
    TSDFVolume volume{ Vector3i{ 10, 10, 10 },Vector3f{ 300, 300, 300 } };
    
    // Ray to left of volume
    for( int z=0; z < 10; z++ ) {
        for( int y=0; y< 10; y++ ) {
            float gz = z * 30 + 15;
            float gy = y * 30 + 15;
            
            Vector3f origin{ 310, gy, gz };
            Vector3f direction{ -1.0, 0.0, 0.0 };
            
            Vector3f actual_entry_point;
            float actual_t;
            bool is = volume.is_intersected_by_ray_2( origin , direction, actual_entry_point, actual_t );
            
            
            EXPECT_TRUE( is );
            EXPECT_NEAR(actual_t, 10, FLT_EPSILON);
            EXPECT_NEAR(actual_entry_point.x(), 300.0, FLT_EPSILON);
            EXPECT_NEAR(actual_entry_point.y(), gy, FLT_EPSILON);
            EXPECT_NEAR(actual_entry_point.z(), gz, FLT_EPSILON);
        }
    }
}

TEST( TSDF_Raycasting, givenRayTopToBottomOfVolumeWhenIntersectingThenReturnsTrue ) {
    using namespace phd;
    using namespace Eigen;
    
    TSDFVolume volume{ Vector3i{ 10, 10, 10 },Vector3f{ 300, 300, 300 } };
    
    // Ray to left of volume
    for( int z=0; z < 10; z++ ) {
        for( int x=0; x< 10; x++ ) {
            float gz = z * 30 + 15;
            float gx = x * 30 + 15;
            
            Vector3f origin{ gx, 310, gz };
            Vector3f direction{ 0.0, -1.0, 0.0 };
            
            Vector3f actual_entry_point;
            float actual_t;
            bool is = volume.is_intersected_by_ray_2( origin , direction, actual_entry_point, actual_t );
            
            
            EXPECT_TRUE( is );
            EXPECT_NEAR(actual_t, 10, FLT_EPSILON);
            EXPECT_NEAR(actual_entry_point.x(), gx, FLT_EPSILON);
            EXPECT_NEAR(actual_entry_point.y(), 300, FLT_EPSILON);
            EXPECT_NEAR(actual_entry_point.z(), gz, FLT_EPSILON);
        }
    }
}
TEST( TSDF_Raycasting, givenRayBottomToTopOfVolumeWhenIntersectingThenReturnsTrue ) {
    using namespace phd;
    using namespace Eigen;
    
    TSDFVolume volume{ Vector3i{ 10, 10, 10 },Vector3f{ 300, 300, 300 } };
    
    // Ray to left of volume
    for( int z=0; z < 10; z++ ) {
        for( int x=0; x< 10; x++ ) {
            float gz = z * 30 + 15;
            float gx = x * 30 + 15;
            
            Vector3f origin{ gx, -10, gz };
            Vector3f direction{ 0.0, 1.0, 0.0 };
            
            Vector3f actual_entry_point;
            float actual_t;
            bool is = volume.is_intersected_by_ray_2( origin , direction, actual_entry_point, actual_t );
            
            
            EXPECT_TRUE( is );
            EXPECT_NEAR(actual_t, 10, FLT_EPSILON);
            EXPECT_NEAR(actual_entry_point.x(), gx, FLT_EPSILON);
            EXPECT_NEAR(actual_entry_point.y(), 0, FLT_EPSILON);
            EXPECT_NEAR(actual_entry_point.z(), gz, FLT_EPSILON);
        }
    }
}

#pragma Walk Ray


TEST( TSDF_Raycasting, givenRaysFromFrontToBackWhenRenderingWallThenNoValues ) {
    using namespace phd;
    using namespace Eigen;
    
    float vw, vh, vd;
    int num_voxels = 64;
    TSDFVolume volume = construct_volume(num_voxels, num_voxels, num_voxels, 300, 300, 300, vw, vh, vd);
    create_wall_in_TSDF(volume, 40);
    
    Vector3f direction{ 0.0, 0.0, -1.0 };
    
    for( int x=0; x<num_voxels; x++ ) {
        for( int y=0; y<num_voxels; y++ ) {
            Vector3f origin{ x * vw, y * vh, 450.0 };
            
            Eigen::Vector3f vertex;
            Eigen::Vector3f normal;
            bool values = volume.walk_ray( origin, direction, vertex, normal);
            
            EXPECT_FALSE( values );
        }
    }
}

TEST( TSDF_Raycasting, givenRaysFromBackToFrontWhenRenderingWallThenZVertexIs40NormalZIsMinus1 ) {
    using namespace phd;
    using namespace Eigen;
    
    float vw, vh, vd;
    int num_voxels = 64;
    TSDFVolume volume = construct_volume(num_voxels, num_voxels, num_voxels, 300, 300, 300, vw, vh, vd);
    create_wall_in_TSDF(volume, 40);
    
    float trunc_dist = volume.truncation_distance();
    
    Vector3f direction{ 0.0, 0.0, 1.0 };

    for( int x=0; x<num_voxels; x++ ) {
        for( int y=0; y<num_voxels; y++ ) {
            Vector3f origin{ x * vw, y * vh, -150.0 };
            
            Eigen::Vector3f vertex;
            Eigen::Vector3f normal;
            bool values = volume.walk_ray( origin, direction, vertex, normal);
            
            EXPECT_TRUE( values );

            // Check vertex
            EXPECT_NEAR(vertex.x(), x*vw, EPS);
            EXPECT_NEAR(vertex.y(), y*vh, EPS);
            EXPECT_NEAR(vertex.z(), 40, trunc_dist);

            // Check normal
            EXPECT_NEAR(normal.x(), 0.0, EPS ) << "Invalid normal at (" << x << ", " << y << ")";
            EXPECT_NEAR(normal.y(), 0.0, EPS) << "Invalid normal at (" << x << ", " << y << ")";
            EXPECT_NEAR(normal.z(), -1.0, EPS) << "Invalid normal at (" << x << ", " << y << ")";
        }
    }
}




#pragma mark - raycast generating images
// Front on positive Z axis
TEST( TSDF_Raycasting, testRayCast_150_150_450 ) {
    using namespace phd;
    using namespace Eigen;
    
    float vw, vh, vd;
    TSDFVolume volume = construct_volume(64, 64, 64, 300, 300, 300, vw, vh, vd);
    create_wall_in_TSDF(volume, 100);
    
    
    uint16_t width = 640;
    uint16_t height = 480;
    
    Vector3f * vertices = new Vector3f[ width * height ];
    Vector3f * normals = new Vector3f[ width * height ];
    
    Vector3f light_source{ 0, 0, 0 };
    
    Camera cam = make_kinect();
    cam.move_to( 150, 150, 450 );
    cam.look_at( 150, 150, 150);
    
    volume.raycast(cam, 640, 480, vertices, normals);
    save_normals_as_colour_png("/Users/Dave/Desktop/normals_150_150_450.png", width, height, normals);
    save_rendered_scene_as_png("/Users/Dave/Desktop/render_150_150_450.png", width, height, vertices, normals, cam.position(), light_source);
}

// Front right positive Z axis
TEST( TSDF_Raycasting, testRayCast_450_150_450 ) {
    using namespace phd;
    using namespace Eigen;
    
    float vw, vh, vd;
    TSDFVolume volume = construct_volume(64, 64, 64, 300, 300, 300, vw, vh, vd);
    create_wall_in_TSDF(volume, 40);
    
    uint16_t width = 640;
    uint16_t height = 480;
    
    Vector3f * vertices = new Vector3f[ width * height ];
    Vector3f * normals = new Vector3f[ width * height ];
    
    Vector3f light_source{ 0, 0, 0 };
    
    Camera cam = make_kinect();
    cam.move_to( 450, 150, 450 );
    cam.look_at( 150,150,150);
    
    
    volume.raycast(cam, 640, 480, vertices, normals);
    save_normals_as_colour_png("/Users/Dave/Desktop/normals_450_150_450.png", width, height, normals);
    save_rendered_scene_as_png("/Users/Dave/Desktop/render__450_150_450.png", width, height, vertices, normals, cam.position(), light_source);
}
// Right on positive X axis
TEST( TSDF_Raycasting, testRayCast_450_150_150 ) {
    using namespace phd;
    using namespace Eigen;
    
    float vw, vh, vd;
    TSDFVolume volume = construct_volume(64, 64, 64, 300, 300, 300, vw, vh, vd);
    create_wall_in_TSDF(volume, 100);
    //    create_sphere_in_TSDF(volume, 80);
    
    uint16_t width = 640;
    uint16_t height = 480;
    
    Vector3f * vertices = new Vector3f[ width * height ];
    Vector3f * normals = new Vector3f[ width * height ];

    Vector3f light_source{ 0, 0, 0 };
    
    Camera cam = make_kinect();
    cam.move_to( 450, 150, 150 );
    cam.look_at( 150,150,150);
    
    volume.raycast(cam, 640, 480, vertices, normals);
    save_normals_as_colour_png("/Users/Dave/Desktop/normals_450_150_150.png", width, height, normals);
    save_rendered_scene_as_png("/Users/Dave/Desktop/render_450_150_150.png", width, height, vertices, normals, cam.position(), light_source);
}

// Right rear on positive X, negative Z axis
TEST( TSDF_Raycasting, testRayCast_450_150_m150 ) {
    using namespace phd;
    using namespace Eigen;
    
    float vw, vh, vd;
    TSDFVolume volume = construct_volume(64, 64, 64, 300, 300, 300, vw, vh, vd);
    create_wall_in_TSDF(volume, 100);
    //    create_sphere_in_TSDF(volume, 80);
    
    uint16_t width = 640;
    uint16_t height = 480;
    
    Vector3f * vertices = new Vector3f[ width * height ];
    Vector3f * normals = new Vector3f[ width * height ];
    
    Vector3f light_source{ 0, 0, 0 };
    
    Camera cam = make_kinect( );
    cam.move_to( 450, 150, -150 );
    cam.look_at( 150,150,150);
    
    volume.raycast(cam, 640, 480, vertices, normals);
    save_normals_as_colour_png("/Users/Dave/Desktop/normalstestRayCast_450_150_m150.png", width, height, normals);
    save_rendered_scene_as_png("/Users/Dave/Desktop/rendertestRayCast_450_150_m150.png", width, height, vertices, normals, cam.position(), light_source);
}

// Rear on -ve Z axis
TEST( TSDF_Raycasting, testRayCast_150_150_m150 ) {
    using namespace phd;
    using namespace Eigen;
    
    float vw, vh, vd;
    TSDFVolume volume = construct_volume(64, 64, 64, 300, 300, 300, vw, vh, vd);
    create_wall_in_TSDF(volume, 40);
    
    uint16_t width = 640;
    uint16_t height = 480;
    
    Vector3f * vertices = new Vector3f[ width * height ];
    Vector3f * normals = new Vector3f[ width * height ];

    Vector3f light_source{ 0, 0, 0 };
    
    Camera cam = make_kinect();
    cam.move_to( 150, 150, -150 );
    cam.look_at( 150,150,150);
    
    
    volume.raycast(cam, 640, 480, vertices, normals);
    save_normals_as_colour_png("/Users/Dave/Desktop/normals_150_150_m150.png", width, height, normals);
    save_rendered_scene_as_png("/Users/Dave/Desktop/render_150_150_m150.png", width, height, vertices, normals, cam.position(), light_source);
}

// Left rear on -ve Z, -ve X axis
TEST( TSDF_Raycasting, testRayCast_m150_150_m150 ) {
    using namespace phd;
    using namespace Eigen;
    
    float vw, vh, vd;
    TSDFVolume volume = construct_volume(64, 64, 64, 300, 300, 300, vw, vh, vd);
    create_wall_in_TSDF(volume, 40);
    
    uint16_t width = 640;
    uint16_t height = 480;
    
    Vector3f * vertices = new Vector3f[ width * height ];
    Vector3f * normals = new Vector3f[ width * height ];
    
    Vector3f light_source{ 0, 0, 0 };
    
    Camera cam = make_kinect();
    cam.move_to( -150, 150, -150 );
    cam.look_at( 150,150,150);
    
    
    volume.raycast(cam, 640, 480, vertices, normals);
    save_normals_as_colour_png("/Users/Dave/Desktop/normals_m150_150_m150.png", width, height, normals);
    save_rendered_scene_as_png("/Users/Dave/Desktop/render_m150_150_m150.png", width, height, vertices, normals, cam.position(), light_source);
}
// Left on -ve X axis
TEST( TSDF_Raycasting, testRayCast_m150_150_150 ) {
    using namespace phd;
    using namespace Eigen;
    
    float vw, vh, vd;
    TSDFVolume volume = construct_volume(64, 64, 64, 300, 300, 300, vw, vh, vd);
    create_sphere_in_TSDF(volume, 80);
    
    uint16_t width = 640;
    uint16_t height = 480;
    
    Vector3f * vertices = new Vector3f[ width * height ];
    Vector3f * normals = new Vector3f[ width * height ];
    
    Vector3f light_source{ 0, 0, 0 };
    
    Camera cam = make_kinect();
    cam.move_to( -150, 150, 150 );
    cam.look_at( 150,150,150);
    
    
    volume.raycast(cam, 640, 480, vertices, normals);
    save_normals_as_colour_png("/Users/Dave/Desktop/normals_m150_150_150.png", width, height, normals);
    save_rendered_scene_as_png("/Users/Dave/Desktop/render_m150_150_150.png", width, height, vertices, normals, cam.position(), light_source);
}
// Left front on -ve x axis, +ve z axis
TEST( TSDF_Raycasting, testRayCast_m150_150_450 ) {
    using namespace phd;
    using namespace Eigen;
    
    float vw, vh, vd;
    TSDFVolume volume = construct_volume(64, 64, 64, 300, 300, 300, vw, vh, vd);
    create_sphere_in_TSDF(volume, 80);
    
    uint16_t width = 640;
    uint16_t height = 480;
    
    Vector3f * vertices = new Vector3f[ width * height ];
    Vector3f * normals = new Vector3f[ width * height ];
    
    Vector3f light_source{ 0, 0, 0 };
    
    Camera cam = make_kinect();
    cam.move_to( -150, 150, 450 );
    cam.look_at( 150,150,150);
    
    
    volume.raycast(cam, 640, 480, vertices, normals);
    save_normals_as_colour_png("/Users/Dave/Desktop/normals_m150_150_450.png", width, height, normals);
    save_rendered_scene_as_png("/Users/Dave/Desktop/render_m150_150_450.png", width, height, vertices, normals, cam.position(), light_source);
}
