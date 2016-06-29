//
//  WarpField.cpp
//  WarpField
//
//  Created by Dave on 8/03/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include "WarpField.hpp"
#include "Eigen/Geometry"

namespace phd {
    
    std::size_t WarpField::numberOfNodes() {
        return mNodes.size();
    }
    
    WarpField & WarpField::addNode( const DeformationNode & node ) {
        // Add node
        mNodes.push_back(node);
        
        // Now update the bounds
        double range = node.maxEffectiveRange();
        for( int i=0; i<3; i++ ) {
            
            mEffectiveBounds.min()[i] = fmin(mEffectiveBounds.min()[i], node.getPosition()[i] - range );
            mEffectiveBounds.max()[i] = fmax(mEffectiveBounds.max()[i], node.getPosition()[i] + range );
        }
        return *this;
    }

    
    WarpField & WarpField::addNode(const   Eigen::Vector3d &    position,
                                   double                       rotationAngle,
                                   const   Eigen::Vector3d &    rotationAxis,
                                   const   Eigen::Vector3d &    translation,
                                   double                       weight ) {
        
        
        DeformationNode node{ position, rotationAngle, rotationAxis, translation, weight};
        return addNode(node);
    }
    
    // Return true if the point is affected otherwise false
    bool WarpField::isPointAffectedByWarpField( const Eigen::Vector3d & point) {
        return ( mEffectiveBounds.contains( point ) );
    }
    
    /** Warp some points */
    std::vector<Eigen::Vector3d> WarpField::warp( const std::vector<Eigen::Vector3d> & points) {
        std::vector<Eigen::Vector3d> output;
        
        for( auto i = points.begin(); i!= points.end(); i++ ) {
            if( isPointAffectedByWarpField(*i) ) {
                output.push_back( warp( *i ) );
            } else {
                output.push_back( *i);
            }
        }
        
        return output;
    }
    
    Eigen::Vector3d WarpField::warp( const Eigen::Vector3d & point ) {
        // For each warp node
        DualQuaternion totalEffect;
        
        for( auto n = mNodes.begin(); n != mNodes.end(); n++ ) {
            DeformationNode dn = *n;
            totalEffect += dn.transformationAtPoint(point);
        }
        
        return totalEffect.transformPoint(point);
    }
}