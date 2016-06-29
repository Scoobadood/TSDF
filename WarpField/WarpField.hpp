//
//  WarpField.hpp
//  WarpField
//
//  Created by Dave on 8/03/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#ifndef WarpField_hpp
#define WarpField_hpp

#include "Eigen/Core"
#include "Eigen/Geometry"

#include <vector>

#include "DeformationNode.hpp"

namespace phd {
    
    class WarpField {
    private:
        std::vector<DeformationNode>    mNodes;
        
        // Bounds of space R3 affected by nodes in this warp field
        // May be used to detrmine if a point is affected
        Eigen::AlignedBox3d             mEffectiveBounds;
        Eigen::Vector3d                 mMinimumBounds;
        Eigen::Vector3d                 mMaximumBounds;
        
    public:
        std::size_t numberOfNodes();
        
        WarpField & addNode( const DeformationNode & node );
        WarpField & addNode( const Eigen::Vector3d & position, double rotationAngle, const Eigen::Vector3d & rotationAxis, const Eigen::Vector3d & translation, double weight );

        // Return true if the point is affected otherwise false
        bool isPointAffectedByWarpField( const Eigen::Vector3d & point);
        
        // Get the effective bounds of the warp field
        inline Eigen::AlignedBox3d             effectiveBounds( ) const { return mEffectiveBounds;}
        
        /** Warp some points */
        std::vector<Eigen::Vector3d>           warp( const std::vector<Eigen::Vector3d> & points);
        Eigen::Vector3d                        warp( const Eigen::Vector3d & point );
    };
}

#endif /* WarpField_hpp */
