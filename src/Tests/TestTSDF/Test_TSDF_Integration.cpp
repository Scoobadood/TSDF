//
//  TestTSDF_Integration.cpp
//  KinFu
//
//  Created by Dave on 21/05/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include <gtest/gtest.h>

#include "../../include/TSDFVolume.hpp"
#include "../../include/Camera.hpp"
#include "../../include/PngUtilities.hpp"
#include "../../include/BilateralFilter.hpp"
#include "../../include/RenderUtilities.hpp"
#include "../../include/DepthImage.hpp"
#include "../../include/TUMDataLoader.hpp"
#include "../../include/GPUMarchingCubes.hpp"
#include "../../include/ply.hpp"

#include "TestHelpers.hpp"

Eigen::Matrix4f g_cam_pose;

TEST(  , givenManyImages ) {
    // Set up
    using namespace Eigen;

    /*** SET PARAMETERS HERE ***/

    uint16_t voxels = 400;
    uint16_t num_images = 25;
    bool     save = true;
    bool     raycast = true;
    bool     filter = false;
	bool	mesh = true;

    // Make volume
    TSDFVolume * volume = new TSDFVolume( voxels, voxels, voxels, 3000, 3000, 3000);

    // And camera
    Camera camera = make_kinect();

    // Load depth image
    Vector3f camera_location;
    Vector3f camera_focus;


    BilateralFilter bf{ 2, 2};

    TUMDataLoader data_loader{ "/mnt/hgfs/PhD/Kinect Raw Data/TUM/rgbd_dataset_freiburg1_rpy" };
    DepthImage *di = nullptr;
    Eigen::Matrix4f pose;
    int i = 0;
	
    while( ( di = data_loader.next(pose ) ) != nullptr && ( i<10)) {
    
        std::cout << "Integrating " << i << std::endl;

        // Set location
        camera.set_pose( pose );

	if( i == 0 ) g_cam_pose = pose;
        
	volume->integrate(di->data(), di->width(), di->height(), camera);
        delete di;

	i++;
    }


    // Now save ...
    if( save ) {
        std::cout << "Saving" << std::endl;
        std::ostringstream file_name;
        file_name << "/home/dave/Desktop/TSDF_" << voxels << ".txt";
        volume->save_to_file( file_name.str());
    }


    // ... and render ...
int width{640}, height{480};
    if( raycast ) {
        if( (width > 0) && (height > 0 ) ){
            Vector3f light_source{ 10000, 6600, -10000 };
            Eigen::Matrix<float, 3, Eigen::Dynamic> vertices;
            Eigen::Matrix<float, 3, Eigen::Dynamic> normals;

            std::cout << "Rendering" << std::endl;

            // Set location
            camera.set_pose( g_cam_pose );

            // Raycast volume
            volume->raycast( width, height, camera, vertices, normals);


            PngWrapper * p = normals_as_png(width, height, normals);
            p->save_to( "/home/dave/Desktop/normals.png");
            delete p;

            p = scene_as_png(width, height, vertices, normals, camera, light_source);
            p->save_to("/home/dave/Desktop/vertices.png");
            delete p;

        } else {
            std::cerr << "Width or height not set. Can't render" << std::endl;
        }
    }

	// ... and extract mesh ...
	if( mesh ) {
		std::cout << "Extracting mesh" << std::endl;

		std::vector<int3> triangles;
		std::vector<float3> vertices;
		extract_surface( volume, vertices, triangles);
		std::cout << "Vertices: " << vertices.size() << ", Triangles: " << triangles.size() << std::endl;

		// Save to PLY file
		write_to_ply( "/home/dave/Desktop/sphere.ply", vertices, triangles);
	}

}

int main( int argc, char *argv[] ) {
    testing::InitGoogleTest(&argc, argv );
    return RUN_ALL_TESTS();
}
