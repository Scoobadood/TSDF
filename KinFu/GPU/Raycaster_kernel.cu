#include "Raycaster_kernel.hpp"
#include "math_constants.h"

#include "cu_common.hpp"
#include "TSDF_utilities.hpp"

#include <iomanip>




/**
 * Convert a pixel position in the image to a 3D camera coordinate
 * @param origin The camera position in world coordinates
 * @param pix_x The x pixel coordinate
 * @param pix_y The y pixel coordinate
 * @param rot the camera pose rotation matrix
 * @param kinv THe camera intrinsics inverse matrix
 * @return The unit direction vector for the ray in world coordinate space
 */__device__
float3 compute_ray_direction_at_pixel( const float3& origin, uint16_t pix_x, uint16_t pix_y, const Mat33& rot, const Mat33& kinv ) {
    // Get the pixel coordinate in camera plane
    float3 pix_h { static_cast<float>(pix_x), static_cast<float>(pix_y), 1.0f };
    float3 im_plane_coord = m3_f3_mul( kinv, pix_h );

    // Make this point 1mm in front of the camera (in camera coords)
    im_plane_coord.z = -1;

    // Convert this vector to world coordinate frame
    float3 world_vector = m3_f3_mul( rot, im_plane_coord);

    f3_normalise( world_vector);

    return world_vector;
}

/**
 * Determine the voxel in which a point lies
 * @param point The point in voxel space coordinates (0,0,0) -> (max_x, max_y, max_z)
 * @return The voxel in which the point lies.
 */
__device__
int3 voxel_for_point( const float3 point, const float3 voxel_size ) {
    int3 voxel {
        int(floor( point.x / voxel_size.x )),
        int(floor( point.y / voxel_size.y )),
        int(floor( point.z / voxel_size.z ))
    };

    return voxel;
}

/**
 * Perform trilinear interpolation of the TSDF value at a given point in volume space
 * @param point The point (0,0,0) -> (max_x, max_y, max_z)
 * @param voxel_grid_size The size of the space in voxels
 * @param tsdf_values An array of max_x*max_y*max_z floats being the values in the space
 * @return The interpolated TSDF value
 */
__device__
float trilinearly_interpolate( const float3 point,
                               const dim3 voxel_grid_size,
                               const float3 voxel_size,
                               const float *tsdf_values ) {
    // Get the voxel containing this point
    int3 voxel = voxel_for_point( point, voxel_size );

    // Handle voxel out of bounds
    if ( voxel.x < 0 || voxel.y < 0 || voxel.z < 0  || voxel.x >= voxel_grid_size.x || voxel.y >= voxel_grid_size.y || voxel.z >= voxel_grid_size.z) {
        return CUDART_NAN_F;
    }

    // Get the centre of the voxel
    float3 v_centre = centre_of_voxel_at( voxel.x, voxel.y, voxel.z, voxel_size );

    // Set up the lower bound for trilinear interpolation
    int3 lower;
    lower.x = (point.x < v_centre.x) ? voxel.x - 1 : voxel.x;
    lower.y = (point.y < v_centre.y) ? voxel.y - 1 : voxel.y;
    lower.z = (point.z < v_centre.z) ? voxel.z - 1 : voxel.z;

    // Compute u,v,w
    float3 lower_centre = centre_of_voxel_at( lower.x, lower.y, lower.z, voxel_size );
    float3 uvw = f3_sub( point, lower_centre );
    uvw = f3_div_elem( uvw, voxel_size );
    float u = uvw.x;
    float v = uvw.y;
    float w = uvw.z;

    // Lookup c000 - c111
    float c000 = tsdf_value_at( lower.x + 0, lower.y + 0, lower.z + 0, tsdf_values, voxel_grid_size );
    float c001 = tsdf_value_at( lower.x + 0, lower.y + 0, lower.z + 1, tsdf_values, voxel_grid_size );
    float c010 = tsdf_value_at( lower.x + 0, lower.y + 1, lower.z + 0, tsdf_values, voxel_grid_size );
    float c011 = tsdf_value_at( lower.x + 0, lower.y + 1, lower.z + 1, tsdf_values, voxel_grid_size );
    float c100 = tsdf_value_at( lower.x + 1, lower.y + 0, lower.z + 0, tsdf_values, voxel_grid_size );
    float c101 = tsdf_value_at( lower.x + 1, lower.y + 0, lower.z + 1, tsdf_values, voxel_grid_size );
    float c110 = tsdf_value_at( lower.x + 1, lower.y + 1, lower.z + 0, tsdf_values, voxel_grid_size );
    float c111 = tsdf_value_at( lower.x + 1, lower.y + 1, lower.z + 1, tsdf_values, voxel_grid_size );

    float interpolated = c000 * (1 - u) * (1 - v) * (1 - w) +
                         c001 * (1 - u) * (1 - v) *    w  +
                         c010 * (1 - u) *    v  * (1 - w) +
                         c011 * (1 - u) *    v  *    w  +
                         c100 *    u  * (1 - v) * (1 - w) +
                         c101 *    u  * (1 - v) *    w  +
                         c110 *    u  *    v  * (1 - w) +
                         c111 *    u  *    v  *    w;

    return interpolated;
}


/**
 * For a given dimension check whether the ray can intersect the volume
 * Surviving this method doesn't mean it does intersect, just that it could
 * @param space_min The minimum bound of the voxel space in the current dimension
 * @param space_max The maximum bound of the voxel space in the current dimension
 * @param origin The starting point of the ray in the current dimension
 * @param direction The direction of the ray in the current dimension
 * @param near_t The nearest point of intersection of the voxel space which may be updated in this call
 * @param far_t The furthest point of intersection of the voxel space which may be updated in this call
 * @return true if an intersection is still possible otherwise false
 */
__device__ __forceinline__
bool can_intersect_in_dimension( float space_min, float space_max, float origin, float direction, float& near_t, float& far_t) {
    bool can_intersect = true;

    if( direction == 0 ) {
        // Not moving in this direction so we must be within  bounds in order to have any intersection at all
        if( origin < space_min || origin > space_max ) {
            can_intersect = false;
        }
    } else {

        // compute intersection distance of the planes
        float t1 = ( space_min - origin ) / direction;
        float t2 = ( space_max - origin ) / direction;

        // If t1 > t2 swap (t1, t2) since t1 intersection with near plane
        if( t1 > t2 ) {
            float temp_t = t1;
            t1 = t2;
            t2 = temp_t;
        }

        // if t1 > t_near set t_near = t1 : We want largest t_near
        if( t1 > near_t ) {
            near_t = t1;
        }

        //If t2 < t_far set t_far="t2"  want smallest t_far
        if( t2 < far_t ) {
            far_t = t2;
        }

        // If Tnear > Tfar box is missed so return false
        if( near_t > far_t ) {
            can_intersect =  false;
        } else {
            // If Tfar < 0 box is behind ray return false end
            if( far_t < 0 ) {
                can_intersect = false;
            }
        }
    }
    return can_intersect;
}

/**
 * Compute the intersections of the ray from origin in direction through the cube specified by space_min and space_max
 * There are three cases:
 * origin is inside the cube near_t =0, far_t is distance to exit and we return true
 * origin is outside of cube and ray misses it near_t and far_t are undefined, return value is false
 * origin is outside of cube and ray penetrates. near_t and far_t are defined (should have the same sign) and return value is true
 * @param origin The source of the ray in world coordinates
 * @param direction a Vector representing the direction of the ray in world coordinates
 * @param space_min The lower, leftmost, frontmost vertex of the voxel space in world coordinates
 * @param space_mac The upper, rightmost, rearmost vertex of the voxel space in world coordinates
 * @param near_t Populated with the nearpoint if return is true
 * @param far_t populated with the far point if return is true
 * @return true if the ray pierces the volme
 */
__device__
bool  compute_near_and_far_t( const float3 & origin, const float3 & direction, const float3 & space_min, const float3 & space_max, float & near_t, float & far_t ) {
    bool intersects = false;

    // Handle start_point in space
    if ( origin.x >= space_min.x && origin.x <= space_max.x && origin.y >= space_min.y && origin.y <= space_max.y && origin.z >= space_min.z && origin.z <= space_max.z ) {
        // Near_t is zero as origin is inside the space
        near_t = 0;

        float x_t = CUDART_NAN_F;
        float y_t = CUDART_NAN_F;
        float z_t = CUDART_NAN_F;
        if ( direction.x > 0 ) {
            x_t = (space_max.x - origin.x) / direction.x;
        } else if ( direction.x < 0 ) {
            x_t = ( space_min.x - origin.x) / direction.x;
        }

        if ( direction.y > 0 ) {
            y_t = (space_max.y - origin.y) / direction.y;
        } else if ( direction.y < 0 ) {
            y_t = ( space_min.y - origin.y) / direction.y;
        }

        if ( direction.z > 0 ) {
            z_t = (space_max.z - origin.z) / direction.z;
        } else if ( direction.z < 0 ) {
            z_t = ( space_min.z - origin.z) / direction.z;
        }

        if ( x_t < y_t ) {
            if ( x_t < z_t ) far_t = x_t;
            else far_t = z_t;
        } else {
            if ( y_t < z_t ) far_t = y_t;
            else far_t = z_t;
        }
        intersects = true;
    } else {
        // Outside of the voxel space just now
        near_t = -CUDART_INF_F;
        far_t  =  CUDART_INF_F;

        // Consider X
        if ( (can_intersect_in_dimension( space_min.x, space_max.x, origin.x, direction.x, near_t, far_t ) ) &&
             (can_intersect_in_dimension( space_min.y, space_max.y, origin.y, direction.y, near_t, far_t ) ) &&
             (can_intersect_in_dimension( space_min.z, space_max.z, origin.z, direction.z, near_t, far_t ) ) ) {
             intersects = true;
        }
    }

    // If we got here then

    return intersects;
}
/**
 * Walk a ray from the camera onto the TSDF determining where (if at all) it
 * intersects the ISO surface defcined by the TSDF
 * @param origin The position of the camera in world coordinates
 * @param rot The camera pose rotation matrix in world terms
 * @param kinv The camera's intrinsic matrix inverse
 * @param space_min The lowest X,Y,Z in world coordinates occupied by the voxel space
 * @param space_max The highest  X,Y,Z in world coordinates occupied by the voxel space
 * @param voxel_grid_size Dimension of the voxel space in each direction in voxels
 * @param voxel_size Dimensions of each voxel
 * @param tsdf_values A pointer to the TSDF distance values with Z,Y,X coords major
 * @param vertex The 3D world coordinate of the vertex intersected by this ray if it exists or {NaN, NaN, NaN}
 */
__global__
void process_ray(const float3 origin, const Mat33 rot, const Mat33 kinv,
                 uint16_t max_x, uint16_t max_y,
                 const float3 space_min, const float3 space_max,
                 const dim3 voxel_grid_size, const float3 voxel_size,
                 const float *tsdf_values,
                 float3 *vertices) {

    int imx = threadIdx.x + blockIdx.x * blockDim.x;
    int imy = threadIdx.y + blockIdx.y * blockDim.y;

    // Terminat eearly if the index is out of bounds
    if ( imx >= max_x || imy >= max_y ) {
        return;
    }

    size_t idx = imy * max_x + imx;

    // Compute the ray direction for this pixel
    // which is R K-1 (x,y,1)T
    float3 direction      = compute_ray_direction_at_pixel( origin, imx, imy, rot, kinv );

    // Compute near and far intersections of the TSDF volume by this ray
    float near_t, far_t;
    bool intersects = compute_near_and_far_t( origin, direction, space_min, space_max, near_t, far_t );

    // Only do this if the ray intersects space
    float3 current_point { CUDART_NAN_F, CUDART_NAN_F, CUDART_NAN_F};
    if( intersects ) {
        // Start pooint in world coords
        float3 start_point = f3_add( origin, f3_mul_scalar( near_t, direction ) );

        // Adjust to be in voxel grid coords
        start_point = f3_sub( start_point, space_min);

        bool done = false;

        float tsdf = 0, previous_tsdf = 0;


        // Set up current point to iterate
        float t = 0;
        float max_t = far_t - near_t;

        // Iterate until
        //   We leave the voxel space (fail)
        //   We transit from +ve to -ve tsdf (intersect)
        //   We transit from -ve to +ve (fail)
        while ( !done ) {
            current_point = f3_add( start_point, f3_mul_scalar( t, direction ) );

            previous_tsdf = tsdf;

            // Compute voxel for current_point
            int3 voxel = voxel_for_point( current_point, voxel_size );

            // Extract the tsdf
            float tsdf = trilinearly_interpolate( current_point, voxel_grid_size, voxel_size, tsdf_values );

            // If tsdf is negative then we're behind the surface
            if ( tsdf < 0 ) {
                t = t - 1 + ( previous_tsdf / (previous_tsdf - tsdf));
                current_point = {
                    start_point.x + t * direction.x,
                    start_point.y + t * direction.y,
                    start_point.z + t * direction.z
                };
                done = true;
            } else {
                t = t + 1;
                if( t >= max_t ) {
                    current_point = float3 { CUDART_NAN_F, CUDART_NAN_F, CUDART_NAN_F};
                    done = true;
                }
            }
        }
    }
    vertices[idx] = current_point;
}


__host__
float3 float3_from_eigen_vector( const Eigen::Vector3f & vector ) {
    float3 f { vector[0], vector[1], vector[2]};
    return f;
}






/**
 * Compute normals to the vertex data provided
 * @param width The with of the output matrix
 * @param height The height of the output matrix
 * @param vertices The input vertex array - unchanged by ths
 * @param normals the oputput normals array to be populated
 */
__global__
void compute_normals( uint16_t width, uint16_t height, const float3 * vertices, float3 * normals ) {
    int imx = threadIdx.x + blockIdx.x * blockDim.x;
    int imy = threadIdx.y + blockIdx.y * blockDim.y;

    // Terminate eearly if the index is out of bounds
    if ( imx >= width || imy >= height ) {
        return;
    }

    size_t idx = (imy * width + imx);

    if( imy == height - 1 ) {
        normals[idx].x = 0;
        normals[idx].y = 0;
        normals[idx].z = 0;
    } else {
        if( imx == width-1) {
            normals[idx].x = 0;
            normals[idx].y = 0;
            normals[idx].z = 0;
        } else {
            float3 v1 { vertices[idx+1].x - vertices[idx].x,vertices[idx+1].y - vertices[idx].y,vertices[idx+1].z - vertices[idx].z};
            float3 v2 { vertices[idx+width].x - vertices[idx].x,vertices[idx+width].y - vertices[idx].y,vertices[idx+width].z - vertices[idx].z};

            float nx = v1.y*v2.z - v1.z*v2.y;
            float ny = v1.z*v2.x - v1.x*v2.z;
            float nz = v1.x*v2.y - v1.y*v2.x;
            float l = sqrt( nx*nx+ny*ny+nz*nz);
            normals[idx].x = nx/l;
            normals[idx].y = ny/l;
            normals[idx].z = nz/l;
        }
    }
}

/**
 * POC CUDA kernel to generate vertices as for a sphere in the
 * provided vertex matrix
 * @param width The width of the image
 * @param height The height of the image
 * @param vertices A pointer to an array of width*height*3 floats
 */
__global__
void make_sphere( uint16_t width, uint16_t height, float3 * vertices ) {
    int imx = threadIdx.x + blockIdx.x * blockDim.x;
    int imy = threadIdx.y + blockIdx.y * blockDim.y;

    // Terminat eearly if the index is out of bounds
    if ( imx >= width || imy > height ) {
        return;
    }

    size_t idx = (imy * width + imx);

    // radius is 3/8 of smallest of height and width
    float r = min( 0.75f * width, 0.75f * height) / 2.0f;
    float dx = abs((width/2.0f) - imx);
    float dy = abs((height/2.0f) - imy);
    float r2 = r*r;
    float dx2 = dx*dx;
    float dy2 = dy*dy;

    if( dx2+dy2 < r2) {
        vertices[idx].x = width/2.0f - imx;
        vertices[idx].y = height/2.0f - imy;
        vertices[idx].z = sqrt(r2 - dx2 - dy2);
    } else {
        vertices[idx].x = CUDART_NAN_F;
        vertices[idx].y = CUDART_NAN_F;
        vertices[idx].z = CUDART_NAN_F;
    }
}


__host__
void dump_vector( const std::string& title, const float3& vec ) {
    std::ios oldState(nullptr);
    oldState.copyfmt(std::cout);

    std::cout << title << " (" << std::fixed << std::setw(6) << std::setprecision(2) << vec.x << ", " << vec.y << ", " << vec.z << ")" << std::endl;

    std::cout.copyfmt(oldState);
}

__host__
void dump_matrix( const std::string& title, const Mat33& mat ) {
    std::cout << title << std::endl;

    std::ios oldState(nullptr);
    oldState.copyfmt(std::cout);

    std::cout << std::fixed << std::setw(7) << std::setprecision(4) << std::setfill(' ');
    std::cout << "| " << mat.m11 << " " << mat.m12 << " " << mat.m13 << "|" << std::endl;
    std::cout << "| " << mat.m21 << " " << mat.m22 << " " << mat.m23 << "|" << std::endl;
    std::cout << "| " << mat.m31 << " " << mat.m32 << " " << mat.m33 << "|" << std::endl << std::endl;

    std::cout.copyfmt(oldState);
}


/**
 * Compute the vertex map
 */
__host__
float3 * get_vertices(  const phd::TSDFVolume&  volume,
                        const phd::Camera&      camera,
                        uint16_t                width,
                        uint16_t                height ) {


    // Setup camera origin
    float3 origin = float3_from_eigen_vector( camera.position() );
    dump_vector( "Camera at", origin );

    // Set up voxel grid size
    Eigen::Vector3i volume_size = volume.size();
    dim3 voxel_grid_size {
        static_cast<uint16_t>( volume_size[0] ),
        static_cast<uint16_t>( volume_size[1] ),
        static_cast<uint16_t>( volume_size[2] )
    };

    float3 voxel_size = float3_from_eigen_vector( volume.voxel_size());

    // Set up rotation matrices for pose and Kinv
    const float *pose = camera.pose().data();
    Mat33 rot {
        pose[0], pose[1], pose[2],
        pose[4], pose[5], pose[6],
        pose[8], pose[9], pose[10]
    };
    dump_matrix( "Pose", rot );

    const float *cam_kinv = camera.kinv().data();
    Mat33 kinv {
        cam_kinv[0], cam_kinv[1], cam_kinv[2],
        cam_kinv[3], cam_kinv[4], cam_kinv[5],
        cam_kinv[6], cam_kinv[7], cam_kinv[8]
    };
    dump_matrix( "Kinv", kinv );

    // Set up world coords for min and max extremes of the voxel space
    float3 space_min = float3_from_eigen_vector( volume.offset() );
    dump_vector( "Space Min", space_min );

    float3 space_max = float3_from_eigen_vector( volume.offset() + volume.physical_size( ) );
    dump_vector( "Space Max", space_max );




    cudaError_t err;

    // Allocate storage on device for TSDF data
    float * d_tsdf_values;
    size_t tsdf_data_size = voxel_grid_size.x * voxel_grid_size.y * voxel_grid_size.z * sizeof( float );
    err = cudaMalloc( &d_tsdf_values, tsdf_data_size);
    check_cuda_error( "TSDF alloc failed " , err );

    // Copy TSDF data to device
    err = cudaMemcpy( d_tsdf_values, volume.distance_data(), tsdf_data_size, cudaMemcpyHostToDevice );
    check_cuda_error( "TSDF Memcpy failed ", err);

    // Allocate storage for vertcies on device
    size_t data_size = width * height * sizeof( float3 );
    float3 * d_vertices;
    err = cudaMalloc( &d_vertices, data_size );
    check_cuda_error( "Vertices alloc failed ", err);


    // Execute the kernel
    dim3 block( 32, 32 );
    dim3 grid ( divUp( width, block.x ), divUp( height, block.y ) );
    process_ray<<<grid, block>>>( origin, rot, kinv, width, height, space_min, space_max, voxel_grid_size, voxel_size, d_tsdf_values, d_vertices );
    err = cudaDeviceSynchronize( );
    check_cuda_error( "process_ray failed " , err);


    // Remove TSDF data from device
    err = cudaFree( d_tsdf_values );
    check_cuda_error( "Failed to free TSDF values from device " , err);

    return d_vertices;
}

float3 * compute_normals( uint16_t width, uint16_t height, float3 * d_vertices ) {
    float3 * d_normals;

    cudaError_t err;

    err = cudaMalloc( &d_normals,  width*height*sizeof( float3 ) );
    check_cuda_error( "Normals alloc failed ", err);

    dim3 block( 32, 32 );
    dim3 grid ( divUp( width, block.x ), divUp( height, block.y ) );
    compute_normals<<<grid, block>>>(width, height, d_vertices, d_normals);
    check_cuda_error( "compute_normals failed ", err);

    return d_normals;
}



/**
 * Perform ray casting
 *
 */
__host__
void cast(const phd::TSDFVolume & volume, const phd::Camera & camera,
          uint16_t width, uint16_t height,
          Eigen::Matrix<float, 3, Eigen::Dynamic> & vertices,
          Eigen::Matrix<float, 3, Eigen::Dynamic> & normals ) {

    using namespace Eigen;

    // Compute vertices
    float3 * d_vertices = get_vertices( volume, camera, width, height );

    // Compute normals
    float3 * d_normals = compute_normals( width, height, d_vertices );


    // Copy vertex data back
    cudaError_t err;

    vertices.resize( 3, width*height);
    float  *h_vertices = vertices.data();
    err = cudaMemcpy( h_vertices, d_vertices, width*height*3*sizeof( float ), cudaMemcpyDeviceToHost);
    check_cuda_error( "Vertices Memcpy failed ", err);
    cudaFree( d_vertices );

    // Copy normal data back
    normals.resize( 3, width*height);
    float  *h_normals = normals.data();
    err = cudaMemcpy( h_normals, d_normals, width*height*3*sizeof( float ), cudaMemcpyDeviceToHost);
    check_cuda_error( "Normals Memcpy failed ", err);
    cudaFree( d_normals );
}


__host__
void sphere_cast(const phd::TSDFVolume & volume,
                 const phd::Camera & camera,
                 uint16_t width, uint16_t height,
                 Eigen::Matrix<float, 3, Eigen::Dynamic> & vertices,
                 Eigen::Matrix<float, 3, Eigen::Dynamic> & normals ) {

    using namespace Eigen;


    // Allocate storage on device for verts and normals
    size_t data_size = width * height * sizeof( float3 );
    cudaError_t err;
    dim3 block( 32, 32 );
    dim3 grid ( divUp( width, block.x ), divUp( height, block.y ) );


    // Call a simple CUDA kernel to fill the vertex array with a sphere
    float3 * d_vertices;
    err = cudaMalloc( &d_vertices, data_size );
    assert( err == cudaSuccess );
    make_sphere <<< grid, block>>>( width, height, d_vertices );

    // Copy vertex data back
    vertices.resize( 3, width*height);
    float  *h_vertices = vertices.data();
    err = cudaMemcpy( h_vertices, d_vertices, data_size, cudaMemcpyDeviceToHost);

    // Compute normals
    float3 * d_normals;
    err = cudaMalloc( &d_normals,  data_size );
    assert( err == cudaSuccess );

    // Copy normal data back
    compute_normals<<<grid, block>>>(width, height, d_vertices, d_normals);
    normals.resize( 3, width*height);
    float  *h_normals = normals.data();
    err = cudaMemcpy( h_normals, d_normals, data_size, cudaMemcpyDeviceToHost);

    // Free device memory
    cudaFree( d_vertices );
    cudaFree( d_normals );
}
