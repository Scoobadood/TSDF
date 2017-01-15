//
//  GPUTSDFVolume.hpp
//  A GPU Based TSDF
//
//  Created by Dave on 11/03/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#ifndef TSDFVolume_hpp
#define TSDFVolume_hpp

#include "Camera.hpp"

#include <Eigen/Core>
#include "vector_types.h"

#include <cstdint>
#include <iostream>
#include <string>

class TSDFVolume {
public:
    struct Float3 {
        float x;
        float y;
        float z;
        inline Float3( const float3& rhs ) {
            x = rhs.x;
            y = rhs.y;
            z = rhs.z;
        }
        inline Float3( float fx=0.0f, float fy=0.0f, float fz=0.0f ) {
            x = fx;
            y = fy;
            z = fz;
        }
        inline operator float3() const {return float3{x, y, z}; }

        inline Float3 operator -( const Float3& rhs ) const { 
            return float3{x - rhs.x, y - rhs.y, z - rhs.z}; 
        }
        inline Float3 operator +( const Float3& rhs ) const { 
            return float3{x + rhs.x, y + rhs.y, z + rhs.z}; 
        }
        inline Float3 operator /( const float scalar ) const {
            return float3{x / scalar, y / scalar, z / scalar}; 
        }
        inline Float3 operator *( const Float3& rhs ) const {
            return float3{x * rhs.x, y * rhs.y, z * rhs.z}; 
        }
        inline float norm( ) const {
            return std::sqrt( x*x + y*y + z*z );
        }

    };

    struct Int3 {
        int16_t x;
        int16_t y;
        int16_t z;
    };

    struct  UInt3 {
        unsigned int x;
        unsigned int y;
        unsigned int z;

        inline UInt3( const dim3& rhs ) : x{rhs.x}, y{rhs.y}, z{rhs.z} {};
        inline UInt3( uint32_t x, uint32_t y, uint32_t z ) : x{x}, y{y}, z{z} {};
        inline operator dim3() const {return dim3{x, y, z}; }
    };

    ~TSDFVolume();

    /**
    * Make a TSDFVolume with the given dimensions (voxels) and physcial dimensions
    * @param size The number of voxels in each X,Y and Z dimension
    * @param physical_size The size ( in mm ) of the space contained in the volume
    */
    TSDFVolume( const UInt3& size = UInt3{64, 64, 64},
                const Float3& physical_size = Float3 { 3000.0f, 3000.0f, 3000.0f} );


    /**
     * Make a TSDFVolume with the given dimensins and physical dimensions
     * @param volume_x X dimension in voxels
     * @param volume_y Y dimension in voxels
     * @param volume_z Z dimension in voxels
     * @param psize_x Physical size in X dimension in mm
     * @param psize_y Physical size in Y dimension in mm
     * @param psize_z Physical size in Z dimension in mm
     */
    TSDFVolume( uint16_t volume_x, uint16_t volume_y, uint16_t volume_z, float psize_x, float psize_y, float psize_z );

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
    void set_size( uint16_t volume_x, uint16_t volume_y, uint16_t volume_z, float psize_x, float psize_y, float psize_z);

    /**
     * @return the size of this space in voxels.
     */
    inline UInt3 size( ) const { return (UInt3) m_size; }

    /**
     * @return the dimensions of each voxel in mm
     */
    inline Float3 voxel_size( ) const { return (Float3) m_voxel_size; }

    /**
     * @return the physical size of the volume in world coords (mm)
     */
    inline Float3 physical_size( ) const { return (Float3) m_physical_size; }

    /**
     * @return the truncation distance (mm)
     */
    inline float truncation_distance( ) const { return m_truncation_distance; }

    /**
     * Offset the TSDF volume in space by the given offset. By default, the bottom, left, front corner of
     * voxel (0,0,0) is at world coordinate (0,0,0). This moves that point to the new world coordinate by a
     * @param ox X offset in mm
     * @param oy Y offset in mm
     * @param oz Z offset in mm
     */
    inline void offset( float ox, float oy, float oz ) {
        m_offset.x = ox;
        m_offset.y = oy;
        m_offset.z = oz;
    }

    /**
     * @return the offset f the TSDF volume in space
     */
    inline Float3 offset( ) const { return (Float3) m_offset; }

    /**
     * Clear the voxel and weight data
     */
    void clear( );

#pragma mark - Data access

 /**
     *
     */
    inline size_t index( int x, int y, int z ) const {
        return x + (y * m_size.x) + (z * m_size.x * m_size.y);
    };

    /**
     * Return pointer to distance data
     * @return Pointer to distance data
     */
    inline const float * distance_data() const {
        return m_voxels;
    }

    /**
     * @return pointer to translation data
     */
    inline Float3 *  translation_data() const {
        return reinterpret_cast<Float3 *> (m_voxel_translations);
    }

    /**
     * Set the translation dat for this space
     * @param data Data on host memory space; Assumed to be vx*vy*vz float3
     */
    void set_translation_data( Float3 *data);

    /**
     * Return pointer to weight data
     * @return Pointer to weight data
     */
    inline const float * weight_data() const {
        return m_weights;
    }

    /**
     * Set the distance data for the TSDF in one call
     * @param distance_data Pointer to enough floats to populate the TSFD
     */
    void set_distance_data( const float * distance_data );


    /**
     * Set the weight data for the TSDF in one call
     * @param weight_data Pointer to enough floats to populate the TSFD
     */
    void set_weight_data( const float * weight_data );

#pragma mark - Integrate new depth data
    /**
     * Integrate a range map into the TSDF
     * @param depth_map Pointer to width*height depth values where 0 is an invalid depth and positive values are expressed in mm
     * @param width The horiontal dimension of the depth_map
     * @param height The height of the depth_map
     * @param camera The camera from which the depth_map was taken
     */
    void integrate( const uint16_t * depth_map, uint32_t width, uint32_t height, const Camera & camera );

#pragma mark - Import/Export

    /**
     * Save the TSDF to file
     * @param The filename
     * @return true if the file saved OK otherwise false.
     */
    bool save_to_file( const std::string & file_name) const;

    /**
     * Load the given TSDF file
     * @param The filename
     * @return true if the file saved OK otherwise false.
     */
    bool load_from_file( const std::string & file_name);

#pragma mark - Rendering
    void raycast( uint16_t width, uint16_t height, const Camera& camera, Eigen::Matrix<float, 3, Eigen::Dynamic>& vertices, Eigen::Matrix<float, 3, Eigen::Dynamic>& normals ) const ;

protected:
// Size of voxel grid
    dim3 m_size;

// Physical size of space represented
    float3 m_physical_size;

// Offset of physical grid in world coordinates
    float3 m_offset;

// Voxel memory on device
    float * m_voxels;

//  Voxel weight data on device
    float * m_weights;

// Size of a voxel
    float3 m_voxel_size;

//  Truncation distance
    float m_truncation_distance;

// Max weight for a voxel
    float m_max_weight;

// Storage for per voxel transform on device
    float3 * m_voxel_translations;
};
#endif /* TSDFVolume_hpp */
