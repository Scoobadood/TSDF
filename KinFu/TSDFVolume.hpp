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

struct VERTEX_DATA ;

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
        
        // K is the intrinsic matrix for the Kinect
        Eigen::Matrix3f m_K;
        Eigen::Matrix3f m_K_inv;
        
        // Voxel data
        float           *m_voxels;
        float           *m_weights;

        /**
         * Find the first voxel in a grid along the given ray.
         * @param origin The origin of the ray in world coordinates
         * @ray_direction A unit vector in the directio of the ray
         * @param voxel The returned voxel coordinate of the first voxel the ray hits in the grid
         * @param t Given the ray is origin + t.direction, the value of t for the intersection is returned
         * @return true if the ray intersects the voxel grid or else false
         */
        bool first_voxel_along_ray( const Eigen::Vector3f & origin, const Eigen::Vector3f & ray_direction, Eigen::Vector3i & voxel, float & t ) const;

        
    public:
        /**
         * Constructor with specified number of voxels in each dimension
         * @param size
         * @param physical_size
         */
        TSDFVolume( const Eigen::Vector3i & size, const Eigen::Vector3f & physical_size = Eigen::Vector3f{3000.0f, 3000.0f, 3000.0f} );
        
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
         * Convert a point in the TSDF volume into the corresponding voxel
         * @param point The point to obtain as a voxel
         * @param voxel The voxel coordinate containing that point
         */
        void point_to_voxel( const Eigen::Vector3f & point, Eigen::Vector3i & voxel ) const;
        
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
        float & distance( int x, int y, int z ) const;
        
        /**
         * @param x The horizontal voxel coord
         * @param y The vertical voxel coord
         * @param z The depth voxel coord
         * @return The weight at that voxel
         */
        float & weight( int x, int y, int z ) const;
        
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
        inline size_t index( int x, int y, int z ) const {
            size_t maxIdx = m_size[0] * m_size[1] * m_size[2];
            size_t idx = z * ( m_size[0] * m_size[1]) + y * (m_size[0] ) + x;
            
            assert( idx < maxIdx );
            return idx;
        }
        
#pragma mark - Integrate new depth data
        /**
         * Integrate a range map into the TSDF
         */
        void integrate( const unsigned short * range_map, uint32_t width, uint32_t height, const Eigen::Matrix4f & camera_pose );

        
        /**
         * Extract the ISO Surface corresponding to the 0 crossing
         * as a mesh
         */
        pcl::PolygonMesh extractSISOSurface( ) const;
        
#pragma mark - Ray casting
        
        /**
         * Given an image plane coordinate (x,y) and a depth z, backproject to a point in 3D space
         * using the current camera pose
         */
        Eigen::Vector3f back_project( const Eigen::Matrix4f & pose, const Eigen::Vector3f & point ) const ;
        
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
         * Find the point where the given ray first intersects the TSDF sace in global coordinates
         * @param origin The r=source of the ray
         * @param ray_direction A unit vector in the direction of the ray
         * @param entry_point The point at which the ray enters the TSDF which may be the origin
         * @param t The ray parameter for the intersection; entry_point = origin + (t * ray_direction)
         * @return true if the ray intersects the TSDF otherwise false
         */
        bool is_intersected_by_ray( const Eigen::Vector3f & origin, const Eigen::Vector3f & ray_direction, Eigen::Vector3f & entry_point, float & t ) const;
        
        /**
         * Generate a raycast surface
         * @param pose The point in world coordinates from which to render
         * @param width The width of the output image
         * @param height The height of the output image
         * @param vertex_map A pointer to an array of width * height vertices in frame of reference of camera
         * @param normal_map A pointer to an array of width * height normals in frame of reference of camera
         */
        void raycast( const Eigen::Matrix4f & pose, uint16_t width, uint16_t height,
                                 Eigen::Vector3f * vertex_map,
                                 Eigen::Vector3f * normal_map) const;
    };
}

#endif /* TSDFVolume_hpp */
