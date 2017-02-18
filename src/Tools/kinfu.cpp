#include <iostream>
#include <cmath>

#include "../include/TSDFVolume.hpp"
#include "../include/PngWrapper.hpp"
#include "../include/DepthMapUtilities.hpp"
#include "../include/RenderUtilities.hpp"
#include "../include/ply.hpp"
#include "../include/BlockTSDFLoader.hpp"
#include "../include/TUMDataLoader.hpp"
#include "../include/GPUMarchingCubes.hpp"



Eigen::Matrix4f g_campose;

/**
 * Make the TSDF from input files
 */
TSDFVolume * make_tsdf(int num_images, const char * directory ) {
    using namespace Eigen;

    // Make volume
    TSDFVolume * volume = new TSDFVolume( TSDFVolume::UInt3{ 200, 200, 200}, TSDFVolume::Float3{3000.0f, 3000.0f, 3000.0f});
//    volume->offset(2000,2000,2000);
    
    // And camera (from FREI 1 IR calibration data at TUM)
    Camera * camera = Camera::default_depth_camera();

    // Create TUMDataLoader
    TUMDataLoader tdl{ directory };

    // Construct TSDF Volume
    for ( int i = 0; i < num_images; i++ ) {
        std::cout << "Integrating frame " << i << std::endl;

        // Read it
        Matrix4f pose;
        DepthImage * depthmap = tdl.next( pose );

        uint16_t dmin = 0;
        uint16_t dmax = 0;
        depthmap->min_max( dmin, dmax );
        std::cout << " -- depth " << dmin << "mm to " << dmax <<"mm" << std::endl;

        if( i == 0 ) {
            g_campose = pose ;
        }

        if ( depthmap ) {
            // Set location
            camera->set_pose( pose );
            volume->integrate(depthmap->data() , depthmap->width(), depthmap->height(), *camera);

            delete depthmap;
        }
    }

    delete camera;

    return volume;
}





/**
 * Load the TSDF
 */
TSDFVolume * load_tsdf( const std::string& file_name) {
    TSDFVolume * volume = new TSDFVolume( file_name );


        if ( volume ) {
            std::cout << "Loaded volume " << volume->size().x << "x" << volume->size().y << "x" << volume->size().z << "  (" << volume->physical_size().x << "mm x " << volume->physical_size().y << "mm x " << volume->physical_size().z << "mm)"  << std::endl;
        } else {
            std::cout << "Couldn't load volume"  << std::endl;
        }
        g_campose << 1, 0, 0, 0,      0, 1, 0, 0,     0, 0, 1, 0,   0, 0, 0, 1;
    return volume;
}


typedef struct {
   	int count;
	const char * directory;
    const char * filename;
} Args;



bool parse_args( int argc, const char * argv[], Args& args ) {
	args.count = 0;
	args.directory = 0;
	args.filename = 0;

	if( argc != 3 && argc != 5 ) {
		std::cerr << "Usage: " << argv[0] << "-f filename (to load from file) | -m N -d directory (to create from images )" << std::endl;
		return false;
	}

	int arg_idx = 1;
	while( arg_idx < argc ) {
		if( argv[arg_idx][0] == '-' && strlen( argv[arg_idx] ) == 2 ) {
			switch( argv[arg_idx][1] ) {
				case 'm':
					args.count = atoi( argv[arg_idx + 1] );
					arg_idx += 2;
					break;

				case 'd':
					args.directory = argv[arg_idx + 1];
					arg_idx += 2;
					break;


				case 'f':
					args.filename = argv[arg_idx + 1];
					arg_idx += 2;
					break;

				default:
					std::cerr << "Unexpected argument " << argv[arg_idx] << std::endl;
					return false;
			}
		} else {
			std::cerr << " Expected '-arg'" << std::endl;
			return false;
		}
	}

	if( args.directory != 0 && args.filename != 0 ) {
		std::cerr << "Must specify file OR directory" << std::endl;
		return false;
	}

	if( args.directory != 0 && args.count == 0 ) {
		std::cerr << "Count must be > 0 if directory is specified" << std::endl;
		return false;
	}

	if( args.filename != 0 && args.count > 0 ) {
		std::cerr << "Count does not apply when loading TSDF from file" << std::endl;
		return false;
	}

	return true;
} 

int main( int argc, const char * argv[] ) {
    int retval = 0;
    TSDFVolume *volume = nullptr;

    // Format is 
    // go -m=nn : make with nn frames or
    // go filename

	Args args;
	if( !parse_args( argc, argv, args ) ) {
		exit( -1 );
	}

	if( args.filename != 0 ) {
		volume = load_tsdf( args.filename );
	} else {
		volume = make_tsdf( args.count, args.directory );
	}


    // Save norm and verts
    if ( volume ) {
        Camera * camera = Camera::default_depth_camera();

        Eigen::Matrix< float, 3, Eigen::Dynamic> vertices;
        Eigen::Matrix< float, 3, Eigen::Dynamic> normals;

        std::cout << "Raycasting" << std::endl;

        camera->set_pose( g_campose);

        volume->raycast( 640, 480, *camera, vertices, normals );

        std::cout << "Rendering to image " << std::endl;
        Eigen::Vector3f light_source { g_campose(0,3), g_campose(1,3), g_campose(2,3) };

        PngWrapper *p = scene_as_png( 640, 480, vertices, normals, *camera, light_source );

        std::cout << "Saving PNG" << std::endl;
        p->save_to("/home/dave/Desktop/scene.png");
        delete p;

        std::cout << "Rendering normals to image " << std::endl;
        p = normals_as_png( 640, 480, normals );
        std::cout << "Saving PNG" << std::endl;
        p->save_to("/home/dave/Desktop/normals.png");
        delete p;

        delete camera;

    } else {
        std::cout << "Couldn't make or load volume" << std::endl;
    }

    // Extract Mesh
    if( volume ) {
        std::cout << "Extracting ISO surface" << std::endl;
        std::vector<int3> triangles;
        std::vector<float3> verts;
        extract_surface( volume, verts, triangles);

        // Save to PLY file
        std::cout << "Writing "<< verts.size() << " vertices and " << triangles.size() << " triangles to PLY" << std::endl;
        write_to_ply( "/home/dave/Desktop/mesh.ply", verts, triangles);
        delete volume;
    } else {
        std::cout << "Couldn't extract mesh because couldn't make or load volume" << std::endl;
    }


    return retval;
}

