//
//  TestRayCast.cpp
//  KinFu
//
//  Created by Dave on 14/05/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include <stdio.h>
#include <cfloat>
#include <gtest/gtest.h>

#include "../../include/TSDFVolume.hpp"
#include "../../include/RenderUtilities.hpp"
#include "../../include/GPURaycaster.hpp"

#include "TestHelpers.hpp"

#pragma mark - Grid Entry Points

#pragma mark Misses

// TEST( TSDF_Raycasting, givenRayToLeftOfVolumeWhenIntersectingThenReturnsFalse ) {
//     using namespace Eigen;

//     TSDFVolume volume{ 10, 10, 10, 300, 300, 300 };

//     // Ray to left of volume
//     Vector3f origin{ -10, 150, -150 };
//     Vector3f direction{ 0.0, 0.0, 1.0 };
//     Vector3f actual_entry_point;
//     float actual_t;
//     bool is = volume.is_intersected_by_ray( origin , direction, actual_entry_point, actual_t );

//     EXPECT_FALSE( is );
// }

// TEST( TSDF_Raycasting, givenRayToRightOfVolumeWhenIntersectingThenReturnsFalse ) {
//     using namespace Eigen;

//     CPUTSDFVolume volume{ Vector3i{10, 10, 10}, Vector3f{ 300, 300, 300 } };

//     // Ray to right of volume
//     Vector3f origin{ 310, 150, -150 };
//     Vector3f direction{ 0.0, 0.0, 1.0 };
//     Vector3f actual_entry_point;
//     float actual_t;
//     bool is = volume.is_intersected_by_ray( origin , direction, actual_entry_point, actual_t );

//     EXPECT_FALSE( is );
// }

// TEST( TSDF_Raycasting, givenRayToTopOfVolumeWhenIntersectingThenReturnsFalse ) {
//     using namespace Eigen;

//     CPUTSDFVolume volume{ Vector3i{10, 10, 10}, Vector3f{ 300, 300, 300 } };

//     // Ray to left of volume
//     Vector3f origin{ 150, 310, -150 };
//     Vector3f direction{ 0.0, 0.0, 1.0 };
//     Vector3f actual_entry_point;
//     float actual_t;
//     bool is = volume.is_intersected_by_ray( origin , direction, actual_entry_point, actual_t );

//     EXPECT_FALSE( is );
// }

// TEST( TSDF_Raycasting, givenRayToBottomOfVolumeWhenIntersectingThenReturnsFalse ) {
//     using namespace Eigen;

//     CPUTSDFVolume volume{ Vector3i{10, 10, 10}, Vector3f{ 300, 300, 300 } };

//     // Ray to left of volume
//     Vector3f origin{ 150, -10, -150 };
//     Vector3f direction{ 0.0, 0.0, 1.0 };
//     Vector3f actual_entry_point;
//     float actual_t;
//     bool is = volume.is_intersected_by_ray( origin , direction, actual_entry_point, actual_t );

//     EXPECT_FALSE( is );
// }

// TEST( TSDF_Raycasting, givenRayToFrontOfVolumeWhenIntersectingThenReturnsFalse ) {
//     using namespace Eigen;

//     CPUTSDFVolume volume{ Vector3i{10, 10, 10}, Vector3f{ 300, 300, 300 } };

//     // Ray to left of volume
//     Vector3f origin{ 150, 150, 310 };
//     Vector3f direction{ 0.0, 1.0, 0.0 };
//     Vector3f actual_entry_point;
//     float actual_t;
//     bool is = volume.is_intersected_by_ray( origin , direction, actual_entry_point, actual_t );

//     EXPECT_FALSE( is );
// }

// TEST( TSDF_Raycasting, givenRayToBackOfVolumeWhenIntersectingThenReturnsFalse ) {
//     using namespace Eigen;

//     CPUTSDFVolume volume{ Vector3i{ 10, 10, 10 },Vector3f{ 300, 300, 300 } };

//     // Ray to left of volume
//     Vector3f origin{ 150, 150, -10 };
//     Vector3f direction{ 0.0, 1.0, 0.0 };
//     Vector3f actual_entry_point;
//     float actual_t;
//     bool is = volume.is_intersected_by_ray( origin , direction, actual_entry_point, actual_t );

//     EXPECT_FALSE( is );
// }

// #pragma mark Front hits
// TEST( TSDF_Raycasting, givenRayFrontToBackOfVolumeWhenIntersectingThenReturnsTrue ) {
//     using namespace Eigen;

//     CPUTSDFVolume volume{ Vector3i{ 10, 10, 10 },Vector3f{ 300, 300, 300 } };

//     // Ray to left of volume
//     for( int x=0; x < 10; x++ ) {
//         for( int y=0; y< 10; y++ ) {
//             float gx = x * 30 + 15;
//             float gy = y * 30 + 15;

//             Vector3f origin{ gx, gy, 310 };
//             Vector3f direction{ 0.0, 0.0, -1.0 };

//             Vector3f actual_entry_point;
//             float actual_t;
//             bool is = volume.is_intersected_by_ray( origin , direction, actual_entry_point, actual_t );


//             EXPECT_TRUE( is );
//             EXPECT_NEAR(actual_t, 10, FLT_EPSILON);
//             EXPECT_NEAR(actual_entry_point.x(), gx, FLT_EPSILON);
//             EXPECT_NEAR(actual_entry_point.y(), gy, FLT_EPSILON);
//             EXPECT_NEAR(actual_entry_point.z(), 300.0, FLT_EPSILON);
//         }
//     }
// }
// TEST( TSDF_Raycasting, givenRayBackToFrontOfVolumeWhenIntersectingThenReturnsTrue ) {
//     using namespace Eigen;

//     CPUTSDFVolume volume{ Vector3i{ 10, 10, 10 },Vector3f{ 300, 300, 300 } };

//     // Ray to left of volume
//     for( int x=0; x < 10; x++ ) {
//         for( int y=0; y< 10; y++ ) {
//             float gx = x * 30 + 15;
//             float gy = y * 30 + 15;

//             Vector3f origin{ gx, gy, -10 };
//             Vector3f direction{ 0.0, 0.0, 1.0 };

//             Vector3f actual_entry_point;
//             float actual_t;
//             bool is = volume.is_intersected_by_ray( origin , direction, actual_entry_point, actual_t );


//             EXPECT_TRUE( is );
//             EXPECT_NEAR(actual_t, 10, FLT_EPSILON);
//             EXPECT_NEAR(actual_entry_point.x(), gx, FLT_EPSILON);
//             EXPECT_NEAR(actual_entry_point.y(), gy, FLT_EPSILON);
//             EXPECT_NEAR(actual_entry_point.z(), 0.0, FLT_EPSILON);
//         }
//     }
// }

// TEST( TSDF_Raycasting, givenRayLeftToRightOfVolumeWhenIntersectingThenReturnsTrue ) {
//     using namespace Eigen;

//     CPUTSDFVolume volume{ Vector3i{ 10, 10, 10 },Vector3f{ 300, 300, 300 } };

//     // Ray to left of volume
//     for( int z=0; z < 10; z++ ) {
//         for( int y=0; y< 10; y++ ) {
//             float gz = z * 30 + 15;
//             float gy = y * 30 + 15;

//             Vector3f origin{ -10, gy, gz };
//             Vector3f direction{ 1.0, 0.0, 0.0 };

//             Vector3f actual_entry_point;
//             float actual_t;
//             bool is = volume.is_intersected_by_ray( origin , direction, actual_entry_point, actual_t );


//             EXPECT_TRUE( is );
//             EXPECT_NEAR(actual_t, 10, FLT_EPSILON);
//             EXPECT_NEAR(actual_entry_point.x(), 0.0, FLT_EPSILON);
//             EXPECT_NEAR(actual_entry_point.y(), gy, FLT_EPSILON);
//             EXPECT_NEAR(actual_entry_point.z(), gz, FLT_EPSILON);
//         }
//     }
// }
// TEST( TSDF_Raycasting, givenRayRightToLeftOfVolumeWhenIntersectingThenReturnsTrue ) {
//     using namespace Eigen;

//     CPUTSDFVolume volume{ Vector3i{ 10, 10, 10 },Vector3f{ 300, 300, 300 } };

//     // Ray to left of volume
//     for( int z=0; z < 10; z++ ) {
//         for( int y=0; y< 10; y++ ) {
//             float gz = z * 30 + 15;
//             float gy = y * 30 + 15;

//             Vector3f origin{ 310, gy, gz };
//             Vector3f direction{ -1.0, 0.0, 0.0 };

//             Vector3f actual_entry_point;
//             float actual_t;
//             bool is = volume.is_intersected_by_ray( origin , direction, actual_entry_point, actual_t );


//             EXPECT_TRUE( is );
//             EXPECT_NEAR(actual_t, 10, FLT_EPSILON);
//             EXPECT_NEAR(actual_entry_point.x(), 300.0, FLT_EPSILON);
//             EXPECT_NEAR(actual_entry_point.y(), gy, FLT_EPSILON);
//             EXPECT_NEAR(actual_entry_point.z(), gz, FLT_EPSILON);
//         }
//     }
// }

// TEST( TSDF_Raycasting, givenRayTopToBottomOfVolumeWhenIntersectingThenReturnsTrue ) {
//     using namespace Eigen;

//     CPUTSDFVolume volume{ Vector3i{ 10, 10, 10 },Vector3f{ 300, 300, 300 } };

//     // Ray to left of volume
//     for( int z=0; z < 10; z++ ) {
//         for( int x=0; x< 10; x++ ) {
//             float gz = z * 30 + 15;
//             float gx = x * 30 + 15;

//             Vector3f origin{ gx, 310, gz };
//             Vector3f direction{ 0.0, -1.0, 0.0 };

//             Vector3f actual_entry_point;
//             float actual_t;
//             bool is = volume.is_intersected_by_ray( origin , direction, actual_entry_point, actual_t );


//             EXPECT_TRUE( is );
//             EXPECT_NEAR(actual_t, 10, FLT_EPSILON);
//             EXPECT_NEAR(actual_entry_point.x(), gx, FLT_EPSILON);
//             EXPECT_NEAR(actual_entry_point.y(), 300, FLT_EPSILON);
//             EXPECT_NEAR(actual_entry_point.z(), gz, FLT_EPSILON);
//         }
//     }
// }
// TEST( TSDF_Raycasting, givenRayBottomToTopOfVolumeWhenIntersectingThenReturnsTrue ) {
//     using namespace Eigen;

//     CPUTSDFVolume volume{ Vector3i{ 10, 10, 10 },Vector3f{ 300, 300, 300 } };

//     // Ray to left of volume
//     for( int z=0; z < 10; z++ ) {
//         for( int x=0; x< 10; x++ ) {
//             float gz = z * 30 + 15;
//             float gx = x * 30 + 15;

//             Vector3f origin{ gx, -10, gz };
//             Vector3f direction{ 0.0, 1.0, 0.0 };

//             Vector3f actual_entry_point;
//             float actual_t;
//             bool is = volume.is_intersected_by_ray( origin , direction, actual_entry_point, actual_t );


//             EXPECT_TRUE( is );
//             EXPECT_NEAR(actual_t, 10, FLT_EPSILON);
//             EXPECT_NEAR(actual_entry_point.x(), gx, FLT_EPSILON);
//             EXPECT_NEAR(actual_entry_point.y(), 0, FLT_EPSILON);
//             EXPECT_NEAR(actual_entry_point.z(), gz, FLT_EPSILON);
//         }
//     }
// }

// #pragma Walk Ray


// TEST( TSDF_Raycasting, givenRaysFromFrontToBackWhenRenderingWallThenNoValues ) {
//     using namespace Eigen;

//     int num_voxels = 64;
//     CPUTSDFVolume volume{ Vector3i{num_voxels, num_voxels, num_voxels}, Vector3f{300, 300, 300} };
//     Vector3f vsz = volume.voxel_size();
//     create_wall_in_TSDF(volume, 40);

//     Vector3f direction{ 0.0, 0.0, -1.0 };

//     CPURaycaster raycaster{ 640, 480 };

//     for( int x=0; x<num_voxels; x++ ) {
//         for( int y=0; y<num_voxels; y++ ) {
//             Vector3f origin{ x * vsz[0], y * vsz[1], 450.0 };

//             Eigen::Vector3f vertex;
//             Eigen::Vector3f normal;
//             bool values = raycaster.walk_ray( volume, origin, direction, vertex, normal);

//             EXPECT_FALSE( values );
//         }
//     }
// }

// TEST( TSDF_Raycasting, givenRaysFromBackToFrontWhenRenderingWallThenZVertexIs40NormalZIsMinus1 ) {
//     using namespace Eigen;

//     int num_voxels = 64;
//     CPUTSDFVolume volume( Vector3i{num_voxels, num_voxels, num_voxels}, Vector3f{300, 300, 300});
//     Vector3f vsz = volume.voxel_size();
//     create_wall_in_TSDF(volume, 40);

//     float trunc_dist = volume.truncation_distance();

//     Vector3f direction{ 0.0, 0.0, 1.0 };

//     CPURaycaster CPURaycaster{640, 480};

//     for( int x=0; x<num_voxels; x++ ) {
//         for( int y=0; y<num_voxels; y++ ) {
//             Vector3f origin{ x * vsz.x(), y * vsz.y(), -150.0 };

//             Eigen::Vector3f vertex;
//             Eigen::Vector3f normal;
//             bool values = CPURaycaster.walk_ray( volume, origin, direction, vertex, normal);

//             EXPECT_TRUE( values );

//             // Check vertex
//             EXPECT_NEAR(vertex.x(), x*vsz[0], FLT_EPSILON);
//             EXPECT_NEAR(vertex.y(), y*vsz[1], FLT_EPSILON);
//             EXPECT_NEAR(vertex.z(), 40, trunc_dist);

//             // Check normal
//             EXPECT_NEAR(normal.x(), 0.0, FLT_EPSILON ) << "Invalid normal at (" << x << ", " << y << ")";
//             EXPECT_NEAR(normal.y(), 0.0, FLT_EPSILON) << "Invalid normal at (" << x << ", " << y << ")";
//             EXPECT_NEAR(normal.z(), -1.0, FLT_EPSILON) << "Invalid normal at (" << x << ", " << y << ")";
//         }
//     }
// }




// #pragma mark - raycast generating images
// // Front on positive Z axis
// TEST( TSDF_Raycasting, testRayCast_150_150_450 ) {
//     using namespace Eigen;

//     CPUTSDFVolume volume{Vector3i{64, 64, 64}, Vector3f{300, 300, 300}};
//     create_sphere_in_TSDF(volume, 80);


//     uint16_t width = 640;
//     uint16_t height = 480;

//     Matrix<float, 3, Dynamic> vertices;
//     Matrix<float, 3, Dynamic> normals;

//     Vector3f light_source{ 150, 300, 150 };

//     Camera cam = make_kinect();
//     cam.move_to( 150, 150, 450 );
//     cam.look_at( 150, 150, 150);

//     CPURaycaster raycaster{640, 480};

//     raycaster.raycast(volume, cam, vertices, normals);

//     PngWrapper * p = normals_as_png(width, height, normals );
//     p->save_to("/Users/Dave/Desktop/normals_rear.png");
//     delete p;

//     p = scene_as_png(width, height, vertices, normals, cam, light_source);
//     p->save_to("/Users/Dave/Desktop/render_rear.png");
//     delete p;
// }

// // Front right positive Z axis
// TEST( TSDF_Raycasting, testRayCast_450_150_450 ) {
//     using namespace Eigen;

//     CPUTSDFVolume volume{Vector3i{64, 64, 64}, Vector3f{300, 300, 300}};
//     create_sphere_in_TSDF(volume, 80);

//     uint16_t width = 640;
//     uint16_t height = 480;

//     Matrix<float, 3, Dynamic> vertices;
//     Matrix<float, 3, Dynamic> normals;

//     Vector3f light_source{ 150, 300, 150 };


//     Camera cam = make_kinect();
//     cam.move_to( 450, 150, 450 );
//     cam.look_at( 150,150,150);

//     CPURaycaster raycaster{640, 480};

//     raycaster.raycast(volume, cam, vertices, normals);
//     PngWrapper * p = normals_as_png(width, height, normals );
//     p->save_to("/Users/Dave/Desktop/normals_rear_right.png");
//     delete p;
//     p = scene_as_png(width, height, vertices, normals, cam, light_source);
//     p->save_to("/Users/Dave/Desktop/render_rear_right.png");
//     delete p;
// }

// Right on positive X axis
TEST( TSDF_Raycasting, testRayCast_450_150_150 ) {
    using namespace Eigen;

    TSDFVolume volume{64, 64, 64, 300, 300, 300};
    create_sphere_in_TSDF(volume, 80);

    uint16_t width = 640;
    uint16_t height = 480;

    Matrix<float, 3, Dynamic> vertices;
    Matrix<float, 3, Dynamic> normals;

    Vector3f light_source{ 150, 300, 150 };


    Camera cam = make_kinect();
    cam.move_to( 450, 150, 150 );
    cam.look_at( 150,150,150);

    GPURaycaster raycaster{640, 480};

    raycaster.raycast(volume, cam, vertices, normals);
    PngWrapper * p = normals_as_png(width, height, normals );
    p->save_to("/home/dave/Desktop/normals_right.png");
    delete p;
    p = scene_as_png(width, height, vertices, normals, cam, light_source);
    p->save_to("/home/dave/Desktop/render_right.png");
    delete p;
}

// // Right rear on positive X, negative Z axis
// TEST( TSDF_Raycasting, testRayCast_450_150_m150 ) {
//     using namespace Eigen;

//     CPUTSDFVolume volume{Vector3i{64, 64, 64}, Vector3f{300, 300, 300}};
//     create_sphere_in_TSDF(volume, 80);

//     uint16_t width = 640;
//     uint16_t height = 480;

//     Matrix<float, 3, Dynamic> vertices;
//     Matrix<float, 3, Dynamic> normals;

//     Vector3f light_source{ 150, 300, 150 };

//     Camera cam = make_kinect( );
//     cam.move_to( 450, 150, -150 );
//     cam.look_at( 150,150,150);

//     CPURaycaster CPURaycaster{640, 480};

//     CPURaycaster.raycast(volume, cam, vertices, normals);

//     PngWrapper * p = normals_as_png(width, height, normals );
//     p->save_to("/Users/Dave/Desktop/normals_front_right.png");
//     delete p;
//     p = scene_as_png(width, height, vertices, normals, cam, light_source);
//     p->save_to("/Users/Dave/Desktop/render_front_right.png");
//     delete p;
// }

// // Rear on -ve Z axis
// TEST( TSDF_Raycasting, testRayCast_150_150_m150 ) {
//     using namespace Eigen;

//     CPUTSDFVolume volume{Vector3i{64, 64, 64}, Vector3f{300, 300, 300}};
//     create_sphere_in_TSDF(volume, 80);

//     uint16_t width = 640;
//     uint16_t height = 480;

//     Matrix<float, 3, Dynamic> vertices;
//     Matrix<float, 3, Dynamic> normals;

//     Vector3f light_source{ 150, 300, 150 };

//     Camera cam = make_kinect();
//     cam.move_to( 150, 150, -150 );
//     cam.look_at( 150,150,150);

//     CPURaycaster CPURaycaster{640, 480};

//     CPURaycaster.raycast(volume, cam, vertices, normals);
//     PngWrapper * p = normals_as_png(width, height, normals );
//     p->save_to("/Users/Dave/Desktop/normals_front.png");
//     delete p;
//     p = scene_as_png(width, height, vertices, normals, cam, light_source);
//     p->save_to("/Users/Dave/Desktop/render_front.png");
//     delete p;
// }

// // Left rear on -ve Z, -ve X axis
// TEST( TSDF_Raycasting, testRayCast_m150_150_m150 ) {
//     using namespace Eigen;

//     CPUTSDFVolume volume{Vector3i{64, 64, 64}, Vector3f{300, 300, 300}};
//     create_sphere_in_TSDF(volume, 80);

//     uint16_t width = 640;
//     uint16_t height = 480;

//     Matrix<float, 3, Dynamic> vertices;
//     Matrix<float, 3, Dynamic> normals;

//     Vector3f light_source{ 150, 300, 150 };

//     Camera cam = make_kinect();
//     cam.move_to( -150, 150, -150 );
//     cam.look_at( 150,150,150);

//     CPURaycaster CPURaycaster{640, 480};

//     CPURaycaster.raycast(volume, cam, vertices, normals);
//     PngWrapper * p = normals_as_png(width, height, normals );
//     p->save_to("/Users/Dave/Desktop/normals_front_left.png");
//     delete p;
//     p = scene_as_png(width, height, vertices, normals, cam, light_source);
//     p->save_to("/Users/Dave/Desktop/render_front_left.png");
//     delete p;
// }
// // Left on -ve X axis
// TEST( TSDF_Raycasting, testRayCast_m150_150_150 ) {
//     using namespace Eigen;

//     CPUTSDFVolume volume{Vector3i{64, 64, 64}, Vector3f{300, 300, 300}};
//     create_sphere_in_TSDF(volume, 80);

//     uint16_t width = 640;
//     uint16_t height = 480;

//     Matrix<float, 3, Dynamic> vertices;
//     Matrix<float, 3, Dynamic> normals;

//     Vector3f light_source{ 150, 300, 150 };

//     Camera cam = make_kinect();
//     cam.move_to( -150, 150, 150 );
//     cam.look_at( 150,150,150);

//     CPURaycaster CPURaycaster{640, 480};

//     CPURaycaster.raycast(volume, cam, vertices, normals);
//     PngWrapper * p = normals_as_png(width, height, normals );
//     p->save_to("/Users/Dave/Desktop/normals_left.png");
//     delete p;
//     p = scene_as_png(width, height, vertices, normals, cam, light_source);
//     p->save_to("/Users/Dave/Desktop/render_leftt.png");
//     delete p;
// }
// Left front on -ve x axis, +ve z axis
TEST( TSDF_Raycasting, testRayCast_m150_150_450 ) {
    using namespace Eigen;

    TSDFVolume volume{64, 64, 64, 300, 300, 300};
    create_sphere_in_TSDF(volume, 80);

    uint16_t width = 640;
    uint16_t height = 480;

    Matrix<float, 3, Dynamic> vertices;
    Matrix<float, 3, Dynamic> normals;

    Vector3f light_source{ 150, 300, 150 };

    Camera cam = make_kinect();
    cam.move_to( -150, 150, 450 );
    cam.look_at( 150,150,150);

    GPURaycaster raycaster{640, 480};

    raycaster.raycast(volume, cam, vertices, normals);
    PngWrapper * p = normals_as_png(width, height, normals );
    p->save_to("/home/dave/Desktop/normals_rear_left.png");
    delete p;
    p = scene_as_png(width, height, vertices, normals, cam, light_source);
    p->save_to("/home/dave/Desktop/render_rear_left.png");
    delete p;
}

int main( int argc, char *argv[] ) {
    testing::InitGoogleTest(&argc, argv );
    return RUN_ALL_TESTS();
}