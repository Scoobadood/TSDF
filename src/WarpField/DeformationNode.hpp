//
//  DeformationNode.hpp
//  WarpField
//
//  Created by Dave on 2/03/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#ifndef DeformationNode_hpp
#define DeformationNode_hpp

#include <stdio.h>

#include "DualQuaternion.hpp"
#include "Eigen/Core"

namespace phd {
    
    class DeformationNode {
        
    private:
        // dg_v
        Eigen::Vector3d         mPosition{0.0, 0.0, 0.0};
        
        // dg_se3
        DualQuaternion          mTransformation;
        
        // dg_w
        double                  mRadiusParameter;
        
        double                  computeWeightAtPoint( const Eigen::Vector3d & point ) const;
        
    public:
        DeformationNode( );
        DeformationNode( const phd::DualQuaternion & transformation);
        DeformationNode(const   Eigen::Vector3d &    position,
                        double                       rotationAngle,
                        const   Eigen::Vector3d &    rotationAxis,
                        const   Eigen::Vector3d &    translation,
                        double                       weight );
        
        
        inline const Eigen::Vector3d &      getPosition() const { return mPosition;}
        inline const phd::DualQuaternion &  getTransformation() const {return mTransformation;}
        inline double                       getRadiusParameter() const {return mRadiusParameter;}
        
        inline void                         setRadiusParameter( double radiusParameter ) { mRadiusParameter = radiusParameter; }
        
        inline void                         setPosition( const Eigen::Vector3d position ) { mPosition = position; }
        inline void                         setPosition( double x, double y, double z ) { mPosition(0)=x; mPosition(1) = y, mPosition(2) = z; }

        void                                setMaxEffectiveRange( double range );
        double                              maxEffectiveRange( ) const;
        
        DualQuaternion                      transformationAtPoint( double x, double y, double z ) const;
        DualQuaternion                      transformationAtPoint( const Eigen::Vector3d & location ) const;
        
    };
}
#endif /* DeformationNode_hpp */
