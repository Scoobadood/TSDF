//
//  CPURaycaster.hpp
//  KinFu
//
//  Created by Dave on 16/06/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#ifndef CPURaycaster_hpp
#define CPURaycaster_hpp

#include <Eigen/Core>
#include "Raycaster.hpp"

namespace phd {
    class CPURaycaster : public Raycaster {
    public:
        CPURaycaster( int width=640, int height=480) : Raycaster{ width, height} {};
        
        /**
         * Compute the normal to the ISO surface at the given point
         * Based on http://www.cs.technion.ac.il/~veredc/openfusion/OpenFusionReport.pdf
         * @param point The point; should be inside the TSDF
         * @param normal The returned normal
         */
        void normal_at_point( const TSDFVolume & volume, const Eigen::Vector3f & point, Eigen::Vector3f & normal ) const;
        
        /**
         * Find the point where the given ray first intersects the TSDF space in global coordinates
         * @param origin The source of the ray
         * @param ray_direction A unit vector in the direction of the ray
         * @param entry_point The point at which the ray enters the TSDF which may be the origin
         * @param t The ray parameter for the intersection; entry_point = origin + (t * ray_direction)
         * @return true if the ray intersects the TSDF otherwise false
         */
        bool is_intersected_by_ray( const Eigen::Vector3f & origin, const Eigen::Vector3f & ray_direction, Eigen::Vector3f & entry_point, float & t ) const;
        
        /**
         * Walk ray from start to end seeking intersection with the ISO surface in this TSDF
         * If an intersection is found, return the coordnates in vertex and the surface normal
         * in normal
         * @param volume The volume to be rendered
         * @param ray_start The origin of the ray to be traced
         * @param ray_directioon The direction of the ray to be traced
         * @param vertex The returned vertex
         * @param normal The returned normal
         * @return true if the ray intersects the ISOSurface in which case vertex and normal are populated or else false if not
         */
        bool walk_ray( const TSDFVolume & volume, const Eigen::Vector3f & ray_start, const Eigen::Vector3f & ray_direction, Eigen::Vector3f & vertex, Eigen::Vector3f & normal) const;
        
        /**
         * Raycast the TSDF and store discovered vertices and normals in the ubput arrays
         * @param volume The volume to cast
         * @param camera The camera
         * @param vertices The vertices discovered
         * @param normals The normals
         */
        virtual void raycast( const TSDFVolume & volume, const Camera & camera, Eigen::Vector3f * vertices, Eigen::Vector3f * normals ) const;
        
    };
}
#endif /* CPURaycaster_hpp */
