//
//  main.cpp
//  VoxelSpaceViewer
//
//  Created by Dave on 26/05/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include <Eigen/Core>
#include "TSDFVolume.hpp"
#include "TestHelpers.hpp"



int main(int argc, const char * argv[]) {
    using namespace phd;
    using namespace Eigen;
    
    TSDFVolume volume;
    
    std::cout << "Loading voxel grid..." << std::endl;
    
    volume.load_from_file( "/Users/Dave/Desktop/512_512_TSDF@3200_3200_6400.txt" );
    
    std::cout << "Read file. Rendering." << std::endl;
    
    uint16_t width = 640;
    uint16_t height = 480;
    
    Vector3f * vertices = new Vector3f[ width * height ];
    Vector3f * normals = new Vector3f[ width * height ];
    
    Vector3f light_source{ 0, 0, 0 };
    
    Camera cam = make_kinect();
    
    cam.move_to( 3200, 3200, 6400 );
    cam.look_at( 3200,3200,3200);
    
    
    volume.raycast(cam, 640, 480, vertices, normals);
    save_normals_as_colour_png("/Users/Dave/Desktop/load_front_left_normals.png", width, height, normals);
    save_rendered_scene_as_png("/Users/Dave/Desktop/load_front_left_render.png", width, height, vertices, normals, cam, light_source);

    return 0;
}
