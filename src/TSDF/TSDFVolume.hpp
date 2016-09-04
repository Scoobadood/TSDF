//
//  TSDFVolume.hpp
//  TSDF
//  A direct copy of the PCL Kinfu TSDF volume header
//  But implemented without GPU code
//
//  Created by Dave on 11/03/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#ifndef TSDFVolume_hpp
#define TSDFVolume_hpp

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cstdint>

#include "Camera.hpp"

    typedef struct {
        float x;
        float y;
        float z;
    } Float3;



class TSDFVolume {
protected:
    /**
     * Default constructor does nothing
     */
    TSDFVolume( ) {} ;
public:
    /**
     * Known volume types
     */
    typedef enum {
        CPU,
        GPU
    } volume_type;

    /**
         * Dtor
         * Release volume
         */
    virtual ~TSDFVolume() {};

    /**
    * Factory method to return a CPU or GPU based volume
    * @param volume_x X dimension in voxels
    * @param volume_y Y dimension in voxels
    * @param volume_z Z dimension in voxels
    * @param psize_x Physical size in X dimension in mm
    * @param psize_y Physical size in Y dimension in mm
    * @param psize_z Physical size in Z dimension in mm
    */
    static TSDFVolume *make_volume( TSDFVolume::volume_type type,
                                    const Eigen::Vector3i& voxel_size = Eigen::Vector3i {64, 64, 64},
                                    const Eigen::Vector3f& physical_size = Eigen::Vector3f { 3000, 3000, 3000} );

    /**
     * Factory method to return a CPU or GPU based volume
     * @param volume_x X dimension in voxels
     * @param volume_y Y dimension in voxels
     * @param volume_z Z dimension in voxels
     * @param psize_x Physical size in X dimension in mm
     * @param psize_y Physical size in Y dimension in mm
     * @param psize_z Physical size in Z dimension in mm
     */
    static TSDFVolume *make_volume( volume_type type, uint16_t volume_x, uint16_t volume_y, uint16_t volume_z, float psize_x, float psize_y, float psize_z );

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
    virtual void set_size( uint16_t volume_x, uint16_t volume_y, uint16_t volume_z, float psize_x, float psize_y, float psize_z) = 0;

    /**
     * @return the size of this space in voxels.
     */
    virtual Eigen::Vector3i size( ) const = 0;

    /**
     * @return the dimensions of each voxel in mm
     */
    virtual Eigen::Vector3f voxel_size( ) const = 0;

    /**
     * @return the physical size of the volume in world coords (mm)
     */
    virtual Eigen::Vector3f physical_size( ) const = 0;

    /**
     * @return the truncation distance (mm)
     */
    virtual float truncation_distance( ) const = 0;

    /**
     * Offset the TSDF volume in space by the given offset. By default, the bottom, left, front corner of
     * voxel (0,0,0) is at world coordinate (0,0,0). This moves that point to the new world coordinate by a
     * @param ox X offset in mm
     * @param oy Y offset in mm
     * @param oz Z offset in mm
     */
    virtual void offset( float ox, float oy, float oz ) = 0;

    /**
     * @return the offset f the TSDF volume in space
     */
    virtual Eigen::Vector3f offset( ) const = 0;

#pragma mark - Data access
    /**
     * Clear the voxel and weight data
     */
    virtual void clear( ) = 0;

    /**
     * @param x The horizontal voxel coord
     * @param y The vertical voxel coord
     * @param z The depth voxel coord
     * @return The distance to the surface at that voxel
     */
    virtual float distance( int x, int y, int z ) const = 0;

    /**
     * @param x The horizontal voxel coord
     * @param y The vertical voxel coord
     * @param z The depth voxel coord
     * @param distance The distance to set
     * @return The distance to the surface at that voxel
     */
    virtual void set_distance( int x, int y, int z, float distance ) = 0;

    /**
     * @param x The horizontal voxel coord
     * @param y The vertical voxel coord
     * @param z The depth voxel coord
     * @return The weight at that voxel
     */
    virtual float weight( int x, int y, int z ) const = 0;

    /**
     * @param x The horizontal voxel coord
     * @param y The vertical voxel coord
     * @param z The depth voxel coord
     * @param weight The weight to set
     * @return The weight at that voxel
     */
    virtual void set_weight( int x, int y, int z, float weight ) = 0;

    /**
     * Return pointer to distance data
     * @return Pointer to distance data
     */
    virtual const float *  distance_data() const = 0;

    /**
     * @return pointer to translation data
     */
    virtual const Float3 *  translation_data() const = 0;

    /**
     * Return pointer to weight data
     * @return Pointer to weight data
     */
    virtual const float  * weight_data() const = 0;

    /**
     * Set the data
     */
    virtual void set_distance_data( const float * distance_data ) = 0;
    /**
     * Set the data
     */
    virtual void set_weight_data( const float * distance_data ) = 0;



#pragma mark - Integrate new depth data
    /**
     * Integrate a range map into the TSDF
     * @param depth_map Pointer to width*height depth values where 0 is an invalid depth and positive values are expressed in mm
     * @param width The horiontal dimension of the depth_map
     * @param height The height of the depth_map
     * @param camera The camera from which the depth_map was taken
     */
    virtual void integrate( const uint16_t * depth_map, uint32_t width, uint32_t height, const Camera & camera ) = 0;

#pragma mark - Import/Export

    /**
     * Save the TSDF to file
     * @param The filename
     * @return true if the file saved OK otherwise false.
     */
    virtual bool save_to_file( const std::string & file_name) const = 0;

    /**
     * Load the given TSDF file
     * @param The filename
     * @return true if the file saved OK otherwise false.
     */
    virtual bool load_from_file( const std::string & file_name) = 0;

#pragma mark - Rendering
    /**
     * Raycast the TSDF and store discovered vertices and normals in the ubput arrays
     * @param volume The volume to cast
     * @param camera The camera
     * @param vertices The vertices discovered
     * @param normals The normals
     */
    virtual void raycast( uint16_t width, uint16_t height, const Camera& camera, Eigen::Matrix<float, 3, Eigen::Dynamic>& vertices, Eigen::Matrix<float, 3, Eigen::Dynamic>& normals ) const = 0;
};

#endif /* TSDFVolume_hpp */

