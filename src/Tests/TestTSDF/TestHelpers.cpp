//
//  TestHelpers.cpp
//  KinFu
//
//  Created by Dave on 16/05/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include <fstream>
#include "TestHelpers.hpp"
#include "../KinFu/Utilities/PgmUtilities.hpp"
#include "../KinFu/Utilities/PngUtilities.hpp"

#pragma mark - helpers


void create_sphere_in_TSDF( phd::TSDFVolume & volume, float radius ) {
    using namespace phd;
    using namespace Eigen;

    Vector3i size = volume.size();
    Vector3f volume_centre = (volume.physical_size() / 2.0f) + volume.offset();

    float trunc_dist = volume.truncation_distance();

    for( int i=0; i<size.x(); i++ ) {
        for( int j=0; j<size.y(); j++ ) {
            for( int k=0; k<size.z(); k++ ) {
                Vector3f voxel_centre = volume.offset().array() + (volume.voxel_size().array() * Array3f{i+0.5f, j+0.5f, k+0.5f} );
                Vector3f vector = volume_centre - voxel_centre;
                float dist = std::sqrt( vector.dot( vector ) );
                dist = dist - radius;

                dist = std::fminf( std::fmaxf( dist, -trunc_dist ), trunc_dist );

                volume.set_distance(i, j, k, dist);
            }
        }
    }
}

void create_wall_in_TSDF( phd::TSDFVolume & volume, float depth ) {
    using namespace phd;
    using namespace Eigen;

    float trunc_dist = volume.truncation_distance();

    Vector3i size = volume.size();

    for( int i=0; i<size.x(); i++ ) {
        for( int j=0; j<size.y(); j++ ) {
            for( int k=0; k<size.z(); k++ ) {
                Vector3f voxel_centre = volume.offset().array() + (volume.voxel_size().array() * Array3f{i+0.5f, j+0.5f, k+0.5f} );

                float dist = depth - voxel_centre.z();

                dist = std::fminf( std::fmaxf( dist, -trunc_dist ), trunc_dist );
                volume.set_distance(i, j, k, dist);
            }
        }
    }
}

void create_cube_in_TSDF( phd::TSDFVolume & volume, float depth ) {
    using namespace phd;
    using namespace Eigen;

    float trunc_dist = volume.truncation_distance();

    Vector3i size = volume.size();

    for( int i=0; i<size.x(); i++ ) {
        for( int j=0; j<size.y(); j++ ) {
            for( int k=0; k<size.z(); k++ ) {
                Vector3f voxel_centre = volume.offset().array() + (volume.voxel_size().array() * Array3f{i+0.5f, j+0.5f, k+0.5f} );

                float dist = depth - voxel_centre.z();

                dist = std::fminf( std::fmaxf( dist, -trunc_dist ), trunc_dist );
                volume.set_distance(i, j, k, dist);
                volume.set_weight( i, j, k, 1 );
            }
        }
    }
}




Eigen::Matrix4f make_y_axis_rotation( float theta, Eigen::Vector3f pos ) {

    float cos_theta = cosf( theta );
    float sin_theta = sinf( theta );
    Eigen::Matrix4f rot;

    rot <<  cos_theta,  0,  sin_theta,  pos.x(),
            0,          1,  0,          pos.y(),
            -sin_theta, 0,  cos_theta,  pos.z(),
            0,          0,  0,          1;

    return rot;
}
Eigen::Matrix4f make_x_axis_rotation( float theta, Eigen::Vector3f pos ) {

    float cos_theta = cosf( theta );
    float sin_theta = sinf( theta );
    Eigen::Matrix4f rot;

    rot <<  1, 0,           0,          pos.x(),
            0, cos_theta,   -sin_theta, pos.y(),
            0, sin_theta,   cos_theta,  pos.z(),
            0,  0,          0,          1;

    return rot;
}
Eigen::Matrix4f make_z_axis_rotation( float theta, Eigen::Vector3f pos ) {

    float cos_theta = cosf( theta );
    float sin_theta = sinf( theta );
    Eigen::Matrix4f rot;

    rot <<  cos_theta, -sin_theta, 0, pos.x(),
            sin_theta,  cos_theta, 0, pos.y(),
            0,          0,         1, pos.z(),
            0,          0,         0,  1;

    return rot;
}

phd::Camera make_kinect( ) {
    return phd::Camera{ 585.6f, 585.6f, 316.0f, 247.6f };
}

uint16_t * make_sphere_depth_map( uint16_t width, uint16_t height, uint16_t radius, uint16_t min_depth, uint16_t max_depth ) {
    uint16_t * depths = new uint16_t[width*height];

    if( max_depth < min_depth) {
        float t = min_depth;
        min_depth = max_depth;
        max_depth = t;
    }

    uint32_t idx = 0;
    float cx = width/2.0f;
    float cy = height/2.0f;
    float r2 = radius * radius;
    float half_depth =(max_depth - min_depth) / 2.0;
    float depth_centre = min_depth + half_depth;

    for( uint16_t y=0; y<height; y++ ) {
        for( uint16_t x=0; x<width; x++ ) {

            uint16_t depth = 0;

            float dx = (cx - x);
            float dy = (cy - y);

            float dx2 = dx*dx;
            float dy2 = dy*dy;
            if( dx2+dy2 < r2 ) {
                // dx^2 + dy^2 + dz^2 = radius^2
                float dz = std::sqrt( r2 - (dx2+dy2) );
                depth = depth_centre - dz;
                depth = std::max(min_depth, std::min( max_depth, depth ) );
            }

            depths[idx] = depth;
            idx++;
        }
    }

    return depths;
}

uint16_t * make_wall_depth_map( uint16_t width, uint16_t height, uint16_t max_depth, uint16_t min_depth, uint16_t wall_depth ) {
    uint16_t * depths = new uint16_t[width*height];

    int wall_min_x = (width / 5) * 2;
    int wall_max_x = (width / 5) * 3;
    int wall_min_y = (height / 5) * 2;
    int wall_max_y = (height / 5) * 3;

    uint32_t idx = 0;
    for( uint16_t y=0; y<height; y++ ) {
        for( uint16_t x=0; x<width; x++ ) {
            uint16_t depth = max_depth;


            if( x > wall_min_x && x < wall_max_x && y > wall_min_y && y < wall_max_y ) {
                depth = wall_depth;
            }
            depths[idx] = std::max(min_depth, std::min( max_depth, depth ) );
            idx++;
        }
    }

    return depths;
}

/**
 * Save a depth map as
 * width [uint16_t]
 * height[uint16_t]
 * data  [uint16_t * wdith x height]
 */
void save_depth_map( std::string file_name, uint16_t width, uint16_t height, uint16_t * pixels) {
    std::ofstream of{ file_name, std::ios::out | std::ios::binary };
    if( of.good() ) {
        of.write ( reinterpret_cast <char*> ( &width ), sizeof( width ) );
        of.write ( reinterpret_cast <char*> ( &height), sizeof( height ) );
        of.write( reinterpret_cast <char*> ( pixels ), width * height * sizeof( uint16_t ) );
    } else {
        std::cerr << "Couldn't create file " << file_name << std::endl;
    }
}

/**
 * Load a depth map as
 * width [uint16_t]
 * height[uint16_t]
 * data  [uint16_t * wdith x height]
 */
uint16_t * load_depth_map( std::string file_name, uint16_t & width, uint16_t & height) {
    uint16_t * pixels = NULL;

    std::ifstream in_f{ file_name, std::ios::in | std::ios::binary };
    if( in_f.good() ) {
        in_f.read ( reinterpret_cast <char*> ( &width ), sizeof( width )  );
        in_f.read ( reinterpret_cast <char*> ( &height), sizeof( height ) );

        pixels = new uint16_t[ width * height] ;
        if( pixels ) {
            in_f.read( reinterpret_cast <char*> ( pixels ), width * height * sizeof( uint16_t ) );
        } else {
            std::cerr << "out of memory reading file  " << file_name << std::endl;
        }
    } else {
        std::cerr << "Couldn't create file " << file_name << std::endl;
    }

    return pixels;
}



