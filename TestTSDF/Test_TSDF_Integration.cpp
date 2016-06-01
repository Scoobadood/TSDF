//
//  TestTSDF_Integration.cpp
//  KinFu
//
//  Created by Dave on 21/05/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include <gtest/gtest.h>
#include "TSDFVolume.hpp"
#include "Camera.hpp"
#include "TestHelpers.hpp"
#include "PngUtilities.hpp"
#include "BilateralFilter.hpp"
#pragma mark - Construction


TEST( TSDF_Integration, givenSphere ) {
    // Set up
    using namespace phd;
    using namespace Eigen;
    
    /*** SET PARAMETERS HERE ***/
    
    uint16_t voxels = 64;
    uint16_t num_images = 2;
    bool     save = true;
    bool     raycast = true;

    float dimension = 3200.0f;
    
    
    // Make volume
    float vw, vh, vd;
    TSDFVolume volume = construct_volume(voxels, voxels, voxels, dimension, dimension, dimension, vw, vh, vd);
    volume.offset( -(dimension/2.0f), -(dimension/2.0f), -(dimension/2.0f) );
    
    // And camera
    Camera camera = make_kinect();
    
    // Make spherical depth map
    uint32_t width{640};
    uint32_t height{480};
    uint16_t * depth_map = make_sphere_depth_map(width, height, 180, (dimension * 0.25f), (dimension * 0.75f));
    

    // Project two spheres
    float dist = dimension/2.0f;
    
    float th = 45;
    float cth = dist * cos( th * M_PI / 180.0f );
    float sth = dist * sin( th * M_PI / 180.0f );
    
    
    camera.move_to( -dimension/2, 0, 0);
    camera.look_at( 0, 0, 0 );
    volume.integrate(depth_map, width, height, camera);


    camera.move_to( dimension/2, 0, 0);
    camera.look_at( 0, 0, 0 );
    volume.integrate(depth_map, width, height, camera);

    delete [] depth_map;
    
    
    // Now save ...
    if( save ) {
        std::cout << "Saving" << std::endl;
        std::ostringstream file_name;
        file_name << "/Users/Dave/Desktop/TSDF_" << voxels << ".txt";
        volume.save_to_file( file_name.str());
    }
    
    
    // ... and render ...
    if( raycast ) {
        if( (width > 0) && (height > 0 ) ){
            Vector3f light_source{ 0, (dimension * 0.25f), -(dimension/2.0f) };
            Vector3f * vertices = new Vector3f[ width * height ];
            Vector3f * normals  = new Vector3f[ width * height ];
            
            
            std::cout << "Rendering" << std::endl;
            
            // Set location
            camera.move_to( 0, 0, -dist );
            camera.look_at( 0, 0, 0);
            
            // Raycast volume
            volume.raycast(camera, width, height, vertices, normals);
            
            save_normals_as_colour_png("/Users/Dave/Desktop/normals.png", width, height, normals);
            save_rendered_scene_as_png("/Users/Dave/Desktop/vertices.png", width, height, vertices, normals, camera, light_source);
            
            delete [] vertices;
            delete[] normals;
        } else {
            std::cerr << "Width or height not set. Can't render" << std::endl;
        }
    }
}