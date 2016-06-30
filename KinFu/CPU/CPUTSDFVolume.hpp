//
//  CPUTSDFVolume.hpp
//  TSDF
//  A direct copy of the PCL Kinfu TSDF volume header
//  But implemented without GPU code
//
//  Created by Dave on 11/03/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#ifndef CPUTSDFVolume_hpp
#define CPUTSDFVolume_hpp

#include "../TSDFVolume.hpp"

namespace phd {
class CPUTSDFVolume : public TSDFVolume {
private:
    //  Size in voxels
    Eigen::Vector3i m_size;

    //  Size in metres
    Eigen::Vector3f m_physical_size;

    //  Voxel size in metres
    Eigen::Vector3f m_voxel_size;

    //  Offset of front, bottom, left corner of voxel space in world coordinates
    Eigen::Vector3f m_offset;

    // Truncation distance
    float           m_truncation_distance;

    // Maximum weight
    float           m_max_weight;

    // Voxel data
    float           *m_voxels;
    float           *m_weights;

    // Convenience values for speeing up index calculation
    size_t          m_x_size;
    size_t          m_xy_slice_size;

public:
    /**
     * Constructor with specified number of voxels in each dimension
     * @param size
     * @param physical_size
     */
    CPUTSDFVolume( const Eigen::Vector3i & size = Eigen::Vector3i {64, 64, 64}, const Eigen::Vector3f & physical_size = Eigen::Vector3f {3000.0f, 3000.0f, 3000.0f} );

    /**
     * Dtor
     * Release volume
     */
    virtual ~CPUTSDFVolume();

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
#pragma mark - Coordinate manipulation
    /**
     * @retrun true if the given point is contained in this TSDF volume
     */
    void voxel_bounds( const Eigen::Vector3i & voxel, Eigen::Vector3f & lower_bounds, Eigen::Vector3f & upper_bounds ) const;

    /**
     * @param x The horizontal coord (0-width-1)
     * @param y The vertical coord (0-height - 1)
     * @param z The depth coord (0-depth - 1)
     * @return te coordinate of the centre of the given voxel in world coords (mm)
     */
    Eigen::Vector3f centre_of_voxel_at( int x, int y, int z ) const;

    /**
     * @return The coordinate of the centre of the given voxel in world coords (mm)
     */
    Eigen::Vector3f centre_of_volume(  ) const;

    /**
     * Convert a point in global coordinates into voxel coordinates
     * Logs an error if the point is outside of the grid by a sufficient margin
     * @param point The point to obtain as a voxel
     * @return voxel The voxel coordinate containing that point
     */
    Eigen::Vector3i point_to_voxel( const Eigen::Vector3f & point) const;

    /**
     * @return the size of this space in voxels.
     */
    Eigen::Vector3i size( ) const;

    /**
     * @return the dimensions of each voxel in (mm)
     */
    Eigen::Vector3f voxel_size( ) const;

    /**
     * @return the physical size of the volume in world coords (mm)
     */
    Eigen::Vector3f physical_size( ) const;

    /**
     * @return the truncation distance (mm)
     */
    float truncation_distance( ) const;

    /**
     * Offset the TSDF volume in space by the given offset. By default, the bottom, left, front corner of
     * voxel (0,0,0) is at world coordinate (0,0,0). This moves that point to the new world coordinate by a
     * @param ox X offset in mm
     * @param oy Y offset in mm
     * @param oz Z offset in mm
     */
    void offset( float ox, float oy, float oz );

    /**
     * @return the offset f the TSDF volume in space
     */
    Eigen::Vector3f offset( ) const;

    /**
     * @retrun true if the given point is contained in this TSDF volume
     */
    bool point_is_in_tsdf( const Eigen::Vector3f & point ) const;

    /**
     * @return true if the TSF contains the given voxel coordinate
     */
    inline bool contains_voxel( const Eigen::Vector3i & voxel ) const {
        return ( ( voxel.x() >= 0 ) && ( voxel.x() < m_size.x() ) ) &&
               ( ( voxel.y() >= 0 ) && ( voxel.y() < m_size.y() ) ) &&
               ( ( voxel.z() >= 0 ) && ( voxel.z() < m_size.z() ) );
    }

    /**
     * Find the point where the given ray first intersects the TSDF space in global coordinates
     * @param origin The r=source of the ray
     * @param ray_direction A unit vector in the direction of the ray
     * @param entry_point The point at which the ray enters the TSDF which may be the origin
     * @param t The ray parameter for the intersection; entry_point = origin + (t * ray_direction)
     * @return true if the ray intersects the TSDF otherwise false
     */
    bool is_intersected_by_ray( const Eigen::Vector3f & origin, const Eigen::Vector3f & ray_direction, Eigen::Vector3f & entry_point, float & t ) const;

#pragma mark - Data access
    /**
     * Clear the voxel and weight data
     */
    void clear( );

    /**
     * @param x The horizontal voxel coord
     * @param y The vertical voxel coord
     * @param z The depth voxel coord
     * @return The distance to the surface at that voxel
     */
    float distance( int x, int y, int z ) const;

    /**
     * @param x The horizontal voxel coord
     * @param y The vertical voxel coord
     * @param z The depth voxel coord
     * @param distance The distance to set
     * @return The distance to the surface at that voxel
     */
    void set_distance( int x, int y, int z, float distance );

    /**
     * @param x The horizontal voxel coord
     * @param y The vertical voxel coord
     * @param z The depth voxel coord
     * @return The weight at that voxel
     */
    float weight( int x, int y, int z ) const;

    /**
     * @param x The horizontal voxel coord
     * @param y The vertical voxel coord
     * @param z The depth voxel coord
     * @param weight The weight to set
     * @return The weight at that voxel
     */
    void set_weight( int x, int y, int z, float weight );

    /**
     * Return pointer to distance data
     * @return Pointer to distance data
     */
    inline const float *  distance_data() const {
        return m_voxels;
    }

    /**
     * Return pointer to weight data
     * @return Pointer to weight data
     */
    inline const float  * weight_data() const {
        return m_weights;
    };
    /**
     * @return the size (number of elements) in this space. Used to index
     * raw data returned by other member functions
     */
    size_t length( ) const;

    /**
     * @param x The horizontal coord (0-width-1)
     * @param y The vertical coord (0-height - 1)
     * @param z The depth coord (0-depth - 1)
     * @return te coordinate of the centre of the given voxel in world coords (m)
     */
    inline uint64_t index( uint16_t x, uint16_t y, uint16_t z ) const {
        uint64_t idx = z * m_xy_slice_size + y * m_x_size + x;

        return idx;
    }

    /**
     * Get the upper and lower bounding voxels for a trilinear interpolation at the given point in
     * global space.
     * @param point The point in global coordinates
     * @param lower_bound The voxel forming the lower, left, near bound
     * @param upper_bound The voxel forming the upper, right, far bound
     */
    void get_interpolation_bounds( const Eigen::Vector3f & point, Eigen::Vector3i & lower_bounds, Eigen::Vector3i & upper_bounds ) const;

    /**
     * Trilinearly interpolate the point p in the voxel grid using the tsdf values
     * of the surrounding voxels. At edges we assume that the voxel values continue
     * @param point the point
     * @return the interpolated value
     */
    float trilinearly_interpolate_sdf_at( const Eigen::Vector3f & point ) const;

    /**
     * Return a pointer to the TSDF data ordered Slice,Col,Row major
     * @return the data
     */
    const float * data() const;

    /**
     * Set the data
     */
    virtual void set_distance_data( const float * distance_data );
    /**
     * Set the data
     */
    virtual void set_weight_data( const float * distance_data );

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
    void raycast( uint16_t width, uint16_t height, const Camera& camera, Eigen::Matrix<float, 3, Eigen::Dynamic>& vertices, Eigen::Matrix<float, 3, Eigen::Dynamic>& normals ) const;

};
}

#endif /* CPUTSDFVolume_hpp */
