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
#include "Raycaster.hpp"



int main(int argc, const char * argv[]) {
    using namespace phd;
    using namespace Eigen;
    
    TSDFVolume volume;
    
    std::cout << "Loading voxel grid..." << std::endl;
    
    volume.load_from_file( path_to_file_on_desktop( argv[1] ) );
    
    std::cout << "Read file. Rendering." << std::endl;
    
    uint16_t width = 640;
    uint16_t height = 480;
    
    Vector3f * vertices = new Vector3f[ width * height ];
    Vector3f * normals = new Vector3f[ width * height ];
    
    Vector3f light_source{ 8000, 8000, -8000 };
    
    Camera cam = make_kinect();
    
    cam.move_to( 1300, 650, 1300 );
    cam.look_at( 1300, 1300, 1500);
    

    Raycaster r{ 640, 480};
    r.raycast(volume, cam, vertices, normals);
    save_normals_as_colour_png(path_to_file_on_desktop( "nnn.png" ), width, height, normals);
    save_rendered_scene_as_png(path_to_file_on_desktop( "vvv.png" ), width, height, vertices, normals, cam, light_source);

    return 0;
}
