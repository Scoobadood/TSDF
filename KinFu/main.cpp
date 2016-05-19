//
//  main.cpp
//  KinFu
//
//  Created by Dave on 1/05/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include <iostream>
#include <Eigen/Core>
#include  <pcl/PolygonMesh.h>
#include  <pcl/io/ply_io.h>

#include "TSDFVolume.hpp"
#include "PngUtilities.hpp"
#include "PgmUtilities.hpp"


uint16_t * read_my_depth_map( const std::string & file_name, uint32_t & width, uint32_t & height ) {
    uint16_t * range_map = load_png_from_file( file_name, width, height );
    
    size_t map_size = width * height;
    for( size_t i=0; i<map_size; i++ ) {
        uint16_t v = range_map[i];
        
        range_map[i] = v >> 3;
    }

    return range_map;
}

uint16_t * read_tum_depth_map( const std::string & file_name, uint32_t & width, uint32_t & height ) {
    uint16_t * range_map = load_png_from_file( file_name, width, height );
    
    size_t map_size = width * height;
    for( size_t i=0; i<map_size; i++ ) {
        uint16_t v = range_map[i];
        
        // Convert to metres by dividing by 5000, then to millimetres by multiplying by 1000
        range_map[i] = v / 5;
    }
    
    return range_map;
}

// NYU Maps are in mm already but do need to be byte swapped

uint16_t * read_nyu_depth_map( const std::string & file_name, uint32_t & width, uint32_t & height ) {
    uint16_t * range_map = read_pgm( file_name, width, height );
    
    size_t map_size = width * height;
    for( size_t i=0; i<map_size; i++ ) {
        uint16_t v = range_map[i];
        
        v = (v >> 8) + ( ( v & 0xFF ) * 256 );
        
        // Convert to metres by dividing by 5000, then to millimetres by multiplying by 1000
        range_map[i] = v;
    }
    
    return range_map;
}



/*
 * Construct a sphere in the middle of the TSDF
 */
void make_sphere_in_tsdf( phd::TSDFVolume & tsdf ) {
    using namespace Eigen;
    using namespace phd;
    
    Vector3i size = tsdf.size();
    
    Vector3f voxel_size = tsdf.voxel_size();
    
    float width = size[0] * voxel_size[0];
    float height = size[1] * voxel_size[1];
    float depth =  size[2] * voxel_size[2];
    
    float radius = std::min( width, std::min( depth, height) ) / 3.0f;
    
    Vector3f centre{width / 2.0f, height / 2.0f, depth / 2.0f };
    
    for( uint16_t vx = 0; vx < size[0]; vx ++ ) {
        for( uint16_t vy = 0; vy < size[1]; vy ++ ) {
            for( uint16_t vz = 0; vz < size[0]; vz ++ ) {
                
                Vector3f vg = tsdf.centre_of_voxel_at(vx, vy, vz);
                Vector3f ray = vg - centre;
                float ray_length = ray.norm();
                
                tsdf.distance(vx, vy, vz) = ray_length - radius;
                tsdf.weight(vx, vy, vz) = 1;
            }
        }
    }
    
}

/*
 * Construct a cube in the middle of the TSDF
 */
void make_cube_in_tsdf( phd::TSDFVolume & tsdf ) {
    using namespace Eigen;
    using namespace phd;
    
    Vector3i size = tsdf.size();
    
    Vector3f voxel_size = tsdf.voxel_size();
    
    float width = size[0] * voxel_size[0];
    float height = size[1] * voxel_size[1];
    float depth =  size[2] * voxel_size[2];
    
    float radius = std::min( width, std::min( depth, height) ) / 3.0f;
    
    Vector3f centre{width / 2.0f, height / 2.0f, depth / 2.0f };
    
    for( uint16_t vx = 0; vx < size[0]; vx ++ ) {
        for( uint16_t vy = 0; vy < size[1]; vy ++ ) {
            for( uint16_t vz = 0; vz < size[0]; vz ++ ) {
                
                Vector3f vg = tsdf.centre_of_voxel_at(vx, vy, vz);
                Vector3f ray = vg - centre;
                float ray_length = ray.norm();
                
                tsdf.distance(vx, vy, vz) = ray_length - radius;
                tsdf.weight(vx, vy, vz) = 1;
            }
        }
    }
    
}



int main(int argc, const char * argv[]) {
    uint32_t width, height;
    
//        uint16_t * range_map = read_my_depth_map( "/Users/Dave/Desktop/xbox.png", width, height );
//    uint16_t * range_map = read_my_depth_map( "/Users/Dave/Desktop/SphereKinect.png", width, height );
    uint16_t * range_map = read_nyu_depth_map( "/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/NYU/Raw/cafe/cafe_0001c/d-1295527309.165552-1788500182.pgm", width, height );
//    uint16_t * range_map = read_tum_depth_map( "/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/Washington/rgbd-dataset/cell_phone/cell_phone_1/cell_phone_1_1_1_depth.png", width, height );
//    uint16_t * range_map = read_tum_depth_map( "/Users/Dave/Library/Mobile Documents/com~apple~CloudDocs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_xyz/depth/1305031102.160407.png", width, height );

    
    float dimension = 2000;
    Eigen::Vector3i size{ 512, 512, 16 };
    Eigen::Vector3f phys_size{ dimension, dimension, dimension };
    phd::TSDFVolume tsdf{ size, phys_size };
//    tsdf.offset(-dimension/2.0, -dimension/2.0, 200);
    
    Eigen::Matrix4f pose;
    pose << 1.0, 0.0, 0.0, dimension / 2.0,
            0.0, 1.0, 0.0, dimension / 2.0,
            0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 1.0;
    
    tsdf.integrate(range_map, width, height, pose);

//    pose << 0.0, 0.0, -1.0, dimension,
//            0.0, 1.0, 0.0,  dimension/2.0,
//            1.0, 0.0, 0.0, dimension/2,
//            0.0, 0.0, 0.0, 1.0;
//    tsdf.integrate(range_map, width, height, pose);
//
//    pose << 0.0, 0.0, 1.0, 0,
//            0.0, 1.0, 0.0,  dimension/2.0,
//            -1.0, 0.0, 0.0, dimension/2,
//            0.0, 0.0, 0.0, 1.0;
//    tsdf.integrate(range_map, width, height, pose);
    
//    pose << -1.0, 0.0, 1.0, dimension/2.0,
//            0.0, 1.0, 0.0,  dimension/2.0,
//            0.0, 0.0, -1.0, dimension,
//            0.0, 0.0, 0.0, 1.0;
//    tsdf.integrate(range_map, width, height, pose);

//    make_sphere_in_tsdf(tsdf);
    
    pcl::PolygonMesh pm = tsdf.extractSISOSurface();
    
    std::string ply_name = "/Users/Dave/Desktop/xbox.ply";
    pcl::io::savePLYFile(ply_name, pm);
    
    return 0;
}







