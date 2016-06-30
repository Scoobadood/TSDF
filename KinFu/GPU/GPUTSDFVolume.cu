//
//  TSDFVolume.cpp
//  TSDF
//
//  Created by Dave on 11/03/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//
#include "cu_common.hpp"

#include "GPUTSDFVolume.hpp"
#include "GPURaycaster.hpp"
#include "TSDF_kernel.hpp"
#include "../Utilities/TSDFLoader.hpp"

#include <fstream>
#include <iomanip>

namespace phd {

GPUTSDFVolume::~GPUTSDFVolume() {
    std::cout << "TSDFVolume::dtor called. On entry m_voxels:= " << m_voxels << std::endl;
}

/**
 * Constructor with specified number of voxels in each dimension
 * @param size
 * @param physical_size
 */
GPUTSDFVolume::GPUTSDFVolume( const Eigen::Vector3i & size, const Eigen::Vector3f & physical_size ) : m_offset { 0.0, 0.0, 0.0 }, m_voxels {NULL}, m_weights {NULL} {
    std::cout << "TSDFVolume::ctor called." << std::endl;

    if( ( size.x() > 0 ) && ( size.y() > 0 ) && ( size.z() > 0 ) && ( physical_size.x() > 0 ) && ( physical_size.y() > 0 ) && ( physical_size.z() > 0 ) ) {
        set_size( size.x(), size.y(), size.z() , physical_size.x(), physical_size.y(), physical_size.z() );
    } else {
        throw std::invalid_argument( "Attempt to construct CPUTSDFVolume with zero or negative size" );
    }
}


/**
 * Set the size of the volume. This will delete any existing values and resize the volume, clearing it when done.
 * Volume offset is maintained
 * @param volume_x X dimension in voxels
 * @param volume_y Y dimension in voxels
 * @param volume_z Z dimension in voxels
 * @param psize_x Physical size in X dimension in mm
 * @param psize_y Physical size in Y dimension in mm
 * @param psize_z Physical size in Z dimension in mm
 */
__host__
void GPUTSDFVolume::set_size( uint16_t volume_x, uint16_t volume_y, uint16_t volume_z, float psize_x, float psize_y, float psize_z) {
    using namespace Eigen;

    if( ( volume_x != 0 && volume_y != 0 && volume_z != 0 ) && ( psize_x != 0 && psize_y != 0 && psize_z != 0 ) ) {


        // Remove existing data
        if( m_voxels ) {
            cudaFree( m_voxels ) ;
            m_voxels = 0;
        }
        if( m_weights ) {
            cudaFree( m_weights );
            m_weights = 0;
        }

        m_size = dim3 { volume_x, volume_y, volume_z };
        m_physical_size = float3 { psize_x, psize_y, psize_z };

        // Compute truncation distance - must be at least 2x max voxel size
        float cx = m_physical_size.x / m_size.x;
        float cy = m_physical_size.y / m_size.y;
        float cz = m_physical_size.z / m_size.z;

        m_voxel_size = float3 { cx, cy, cz };

        // Set t > diagonal of voxel
        float vs_norm = sqrt( cx*cx+cy*cy+cz*cz );
        m_truncation_distance = 1.1f * vs_norm;

        // Allocate device storage
        cudaError_t err;
        err = cudaMalloc( &m_voxels, volume_x * volume_y * volume_z * sizeof( float ) );
        if( err != cudaSuccess ) {
            throw std::bad_alloc( );
        }
        err = cudaMalloc( &m_weights, volume_x * volume_y * volume_z * sizeof( float ) );
        if( err != cudaSuccess ) {
            cudaFree( m_voxels );
            throw std::bad_alloc( );
        }

        clear();

        // Max weight for integrating depth images
        m_max_weight = 20.0f;

    } else {
        throw std::invalid_argument( "Attempt to set TSDF size to zero" );
    }
}



/**
 * @return the size of this space.
 */
Eigen::Vector3i GPUTSDFVolume::size( ) const {
    return Eigen::Vector3i { static_cast<int>(m_size.x), static_cast<int>(m_size.y), static_cast<int>(m_size.z)};
}



/**
 * @return the dimensions of each voxel in mm
 */
Eigen::Vector3f GPUTSDFVolume::voxel_size( ) const {
    return Eigen::Vector3f { m_voxel_size.x, m_voxel_size.y, m_voxel_size.z};
}

/**
 * @return the physical size of the volume in world coords (mm)
 */
Eigen::Vector3f GPUTSDFVolume::physical_size( ) const {
    return Eigen::Vector3f {m_physical_size.x, m_physical_size.y, m_physical_size.z};
}

/**
 * @return the truncation distance (mm)
 */
float GPUTSDFVolume::truncation_distance( ) const {
    return m_truncation_distance;
}

/**
 * Offset the TSDF volume in space by the given offset. By default, the bottom, left, front corner of
 * voxel (0,0,0) is at world coordinate (0,0,0). This moves that point to the new world coordinate by a
 * @param ox X offset in mm
 * @param oy Y offset in mm
 * @param oz Z offset in mm
 */
void GPUTSDFVolume::offset( float ox, float oy, float oz ) {
    m_offset = float3 {ox, oy, oz};
}

/**
 * @return the offset f the TSDF volume in space
 */
Eigen::Vector3f GPUTSDFVolume::offset( ) const {
    return Eigen::Vector3f { m_offset.x, m_offset.y, m_offset.z };
}



#pragma mark - Data access
/**
 * @param x The horizontal voxel coord
 * @param y The vertical voxel coord
 * @param z The depth voxel coord
 * @return The distance to the surface at that voxel
 */
float GPUTSDFVolume::distance( int x, int y, int z ) const {
    return m_voxels[ index(x, y, z) ];
}

/**
 * @param x The horizontal voxel coord
 * @param y The vertical voxel coord
 * @param z The depth voxel coord
 * @param distance The distance to set
 * @return The distance to the surface at that voxel
 */
void GPUTSDFVolume::set_distance( int x, int y, int z, float distance ) {
    size_t idx =index( x, y, z );
    m_voxels[ idx ] = distance;
}


/**
 * @param x The horizontal voxel coord
 * @param y The vertical voxel coord
 * @param z The depth voxel coord
 * @return The weight at that voxel
 */
float GPUTSDFVolume::weight( int x, int y, int z ) const {
    return m_weights[ index(x, y, z) ];
}

/**
 * @param x The horizontal voxel coord
 * @param y The vertical voxel coord
 * @param z The depth voxel coord
 * @param weight The weight to set
 * @return The weight at that voxel
 */
void GPUTSDFVolume::set_weight( int x, int y, int z, float weight ) {
    m_weights[ index(x, y, z) ] = weight;
}

void GPUTSDFVolume::set_distance_data( const float * distance_data ) {
        size_t data_size = m_size.x * m_size.y * m_size.z;
        cudaMemcpy( m_voxels, distance_data, data_size, cudaMemcpyHostToDevice );
}
void GPUTSDFVolume::set_weight_data( const float * weight_data ) {
        size_t data_size = m_size.x * m_size.y * m_size.z;
        cudaMemcpy( m_weights, weight_data, data_size, cudaMemcpyHostToDevice );
}

/**
 * Clear the TSDF memory on the device
 */
__host__
void GPUTSDFVolume::clear( ) {
    size_t data_size = m_size.x * m_size.y * m_size.z * sizeof( float );

    cudaMemset( m_weights, 0, data_size );
    cudaMemset( m_voxels, 0, data_size );
}


#pragma mark - Integrate new depth data
/**
 * Integrate a range map into the TSDF
 * This follows the approach in Cohen, N.S.V. 2013, 'Open Fusion', pp. 1–35.
 * whereby new maps have less weight than existing maps
 * @param depth_map Pointer to width*height depth values where 0 is an invalid depth and positive values are expressed in mm
 * @param width The horiontal dimension of the depth_map
 * @param height The height of the depth_map
 * @param camera The camera from which the depth_map was taken
 */
__host__
void GPUTSDFVolume::integrate( const uint16_t * depth_map, uint32_t width, uint32_t height, const Camera & camera ) {
    using namespace Eigen;
    std::cout << "Integrate" << std::endl;

    // Call the kernel
    dim3 block( 32, 32 );
    dim3 grid ( divUp( width, block.x ), divUp( height, block.y ) );

    Mat44 pose;
    memcpy( &pose, camera.pose().data(), 16 * sizeof( float ) );


    integrate_kernel<<<grid, block>>>( m_voxels, m_size, m_physical_size, m_offset, pose, width, height, depth_map);
}

#pragma mark - Import/Export

//TODO: Put load and save into base class. Block transfer data from memory. Write as binary file at least an option.
/**
 * Save the TSDF to file
 * @param The filename
 * @return true if the file saved OK otherwise false.
 */
bool GPUTSDFVolume::save_to_file( const std::string & file_name) const {
    using namespace std;

    // Open file
    ofstream ofs { file_name };
    ofs << fixed << setprecision(3);

    // Write Dimensions
    ofs << "voxel size = " << m_size.x << " " << m_size.y << " " << m_size.z << std::endl;
    ofs << "space size = " << m_physical_size.x << " " << m_physical_size.y << " " << m_physical_size.z << std::endl;

    // Write data
    for( uint16_t y = 0; y< m_size.y ; y++ ) {
        for( uint16_t x = 0; x< m_size.x ; x++ ) {
            ofs << std::endl << "# y "<< y << ", x " << x << " tsdf" << std::endl;

            for( uint16_t z = 0; z< m_size.z ; z++ ) {
                size_t idx = index( x, y, z ) ;

                ofs << m_voxels[ idx ] << " ";
            }

            ofs << std::endl << "# y "<< y << ", x " << x << " weights" << std::endl;
            for( uint16_t z = 0; z< m_size.z ; z++ ) {
                size_t idx = index( x, y, z ) ;
                ofs  << m_weights[ idx ] << " ";
            }
        }
    }

    // Close file
    ofs.close();
    return true;
}


/**
 * Load the given TSDF file
 * @param The filename
 * @return true if the file saved OK otherwise false.
 */
bool GPUTSDFVolume::load_from_file( const std::string & file_name) {

    TSDFLoader loader( this );
    return loader.load_from_file( file_name );
}


#pragma mark - Rendering
void GPUTSDFVolume::raycast( uint16_t width, uint16_t height, const Camera& camera, Eigen::Matrix<float, 3, Eigen::Dynamic>& vertices, Eigen::Matrix<float, 3, Eigen::Dynamic>& normals ) const {
    GPURaycaster raycaster( width, height );

    raycaster.raycast( *this, camera, vertices, normals );
}
}

