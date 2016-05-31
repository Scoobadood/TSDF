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
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <Eigen/Dense>
#include <vector>
#include "Camera.hpp" 

namespace phd
{
    class TSDFVolume {
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
        float           *m_voxels = 0;
        float           *m_weights = 0;
        
        // Convenience values for speeing up index calculation
        size_t          m_x_size;
        size_t          m_xy_slice_size;
        

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

    public:
        /**
         * Constructor with specified number of voxels in each dimension
         * @param size
         * @param physical_size
         */
        TSDFVolume( const Eigen::Vector3i & size = Eigen::Vector3i{64, 64, 64}, const Eigen::Vector3f & physical_size = Eigen::Vector3f{3000.0f, 3000.0f, 3000.0f} );
        
        /**
         * Dtor
         * Release volume
         */
        ~TSDFVolume();
        
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
         * @param point The point to obtain as a voxel
         * @param in_grid Set to true if the voxel is in the grid, otherwise false
         * @return voxel The voxel coordinate containing that point
         */
        Eigen::Vector3i point_to_voxel( const Eigen::Vector3f & point, bool & in_grid ) const;
        
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
        inline size_t index( uint16_t x, uint16_t y, uint16_t z ) const {
            size_t idx = z * m_xy_slice_size + y * m_x_size + x;
            
            return idx;
        }
        
#pragma mark - Integrate new depth data
        /**
         * Integrate a range map into the TSDF
         * @param depth_map Pointer to width*height depth values where 0 is an invalid depth and positive values are expressed in mm
         * @param width The horiontal dimension of the depth_map
         * @param height The height of the depth_map
         * @param camera The camera from which the depth_map was taken
         */
        void integrate( const uint16_t * depth_map, uint32_t width, uint32_t height, const Camera & camera );
        
        /**
         * Extract the ISO Surface corresponding to the 0 crossing
         * as a mesh
         */
        pcl::PolygonMesh extractSISOSurface( ) const;
        
#pragma mark - Ray casting
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
         * Compute the bormal to the ISO surface at the given point
         * Based on http://www.cs.technion.ac.il/~veredc/openfusion/OpenFusionReport.pdf
         * @param point The point; should be inside the TSDF
         * @param normal The returned normal
         */
        void normal_at_point( const Eigen::Vector3f & point, Eigen::Vector3f & normal ) const;

        
        /**
         * Walk ray from start to end seeking intersection with the ISO surface in this TSDF
         * If an intersection is found, return the coordinates in vertex and the surface normal
         * in normal
         */
        bool walk_ray( const Eigen::Vector3f & ray_start, const Eigen::Vector3f & ray_direction, Eigen::Vector3f & vertex, Eigen::Vector3f & normal) const;
        
        /**
         * Find the point where the given ray first intersects the TSDF space in global coordinates
         * @param origin The r=source of the ray
         * @param ray_direction A unit vector in the direction of the ray
         * @param entry_point The point at which the ray enters the TSDF which may be the origin
         * @param t The ray parameter for the intersection; entry_point = origin + (t * ray_direction)
         * @return true if the ray intersects the TSDF otherwise false
         */
        bool is_intersected_by_ray_2( const Eigen::Vector3f & origin, const Eigen::Vector3f & ray_direction, Eigen::Vector3f & entry_point, float & t ) const;

        /**
         * Generate a raycast surface
         * @param camera The camera doing the rendering
         * @param width The width of the output image
         * @param height The height of the output image
         * @param vertex_map A pointer to an array of width * height vertices in frame of reference of camera
         * @param normal_map A pointer to an array of width * height normals in frame of reference of camera
         */
        void raycast( const Camera & camera, uint16_t width, uint16_t height,
                                 Eigen::Vector3f * vertex_map,
                                 Eigen::Vector3f * normal_map) const;


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
    };
}

#endif /* TSDFVolume_hpp */
