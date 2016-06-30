//
//  GPUTSDFVolume.hpp
//  A GPU Based TSDF
//
//  Created by Dave on 11/03/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#ifndef GPUTSDFVolume_hpp
#define GPUTSDFVolume_hpp

#include "../TSDFVolume.hpp"
#include "vector_types.h"

namespace phd {
class GPUTSDFVolume : public TSDFVolume {
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

    //  Truncaion distance
    float m_truncation_distance;

    // Max weight for a voxel
    float m_max_weight;
public:
    GPUTSDFVolume( const Eigen::Vector3i & size = Eigen::Vector3i {64, 64, 64}, const Eigen::Vector3f & physical_size = Eigen::Vector3f {3000.0f, 3000.0f, 3000.0f} );
    /**
         * Dtor
         * Release volume
         */
    ~GPUTSDFVolume();

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
    virtual void set_size( uint16_t volume_x, uint16_t volume_y, uint16_t volume_z, float psize_x, float psize_y, float psize_z);

    /**
     * @return the size of this space in voxels.
     */
    virtual Eigen::Vector3i size( ) const;

    /**
     * @return the dimensions of each voxel in mm
     */
    virtual Eigen::Vector3f voxel_size( ) const;

    /**
     * @return the physical size of the volume in world coords (mm)
     */
    virtual Eigen::Vector3f physical_size( ) const;

    /**
     * @return the truncation distance (mm)
     */
    virtual float truncation_distance( ) const;

    /**
     * Offset the TSDF volume in space by the given offset. By default, the bottom, left, front corner of
     * voxel (0,0,0) is at world coordinate (0,0,0). This moves that point to the new world coordinate by a
     * @param ox X offset in mm
     * @param oy Y offset in mm
     * @param oz Z offset in mm
     */
    virtual void offset( float ox, float oy, float oz );

    /**
     * @return the offset f the TSDF volume in space
     */
    virtual Eigen::Vector3f offset( ) const;

#pragma mark - Data access

    /**
     *
     */
    inline size_t index( int x, int y, int z ) const {
        return x + (y * m_size.x) + (z * m_size.x * m_size.y);
    };

    /**
     * Clear the voxel and weight data
     */
    virtual void clear( );

    /**
     * @param x The horizontal voxel coord
     * @param y The vertical voxel coord
     * @param z The depth voxel coord
     * @return The distance to the surface at that voxel
     */
    virtual float distance( int x, int y, int z ) const;

    /**
     * @param x The horizontal voxel coord
     * @param y The vertical voxel coord
     * @param z The depth voxel coord
     * @param distance The distance to set
     * @return The distance to the surface at that voxel
     */
    virtual void set_distance( int x, int y, int z, float distance );

    /**
     * @param x The horizontal voxel coord
     * @param y The vertical voxel coord
     * @param z The depth voxel coord
     * @return The weight at that voxel
     */
    virtual float weight( int x, int y, int z ) const;

    /**
     * @param x The horizontal voxel coord
     * @param y The vertical voxel coord
     * @param z The depth voxel coord
     * @param weight The weight to set
     * @return The weight at that voxel
     */
    virtual void set_weight( int x, int y, int z, float weight );

    /**
     * Return pointer to distance data
     * @return Pointer to distance data
     */
    inline const float * distance_data() const {
        return m_voxels;
    }

    /**
     * Return pointer to weight data
     * @return Pointer to weight data
     */
    inline const float * weight_data() const {
        return m_weights;
    }

    virtual void set_distance_data( const float * distance_data );
    virtual void set_weight_data( const float * weight_data );

#pragma mark - Integrate new depth data
        /**
         * Integrate a range map into the TSDF
         * @param depth_map Pointer to width*height depth values where 0 is an invalid depth and positive values are expressed in mm
         * @param width The horiontal dimension of the depth_map
         * @param height The height of the depth_map
         * @param camera The camera from which the depth_map was taken
         */
        virtual void integrate( const uint16_t * depth_map, uint32_t width, uint32_t height, const Camera & camera );

#pragma mark - Import/Export

        /**
         * Save the TSDF to file
         * @param The filename
         * @return true if the file saved OK otherwise false.
         */
        virtual bool save_to_file( const std::string & file_name) const;

        /**
         * Load the given TSDF file
         * @param The filename
         * @return true if the file saved OK otherwise false.
         */
        virtual bool load_from_file( const std::string & file_name);

#pragma mark - Rendering
        virtual void raycast( uint16_t width, uint16_t height, const Camera& camera, Eigen::Matrix<float, 3, Eigen::Dynamic>& vertices, Eigen::Matrix<float, 3, Eigen::Dynamic>& normals ) const ;
    };
}

#endif /* GPUTSDFVolume_hpp */

