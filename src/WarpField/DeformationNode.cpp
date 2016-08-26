//
//  DeformationNode.cpp
//  WarpField
//
//  Created by Dave on 2/03/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include "DeformationNode.hpp"

namespace phd {
    
    static const double EPS{1e-6};
    
    DeformationNode::DeformationNode( ) {
        mTransformation = DualQuaternion();
        mPosition       =Eigen::Vector3d(0.0,0.0,0.0);
        mRadiusParameter=1.0;
    }
    
    DeformationNode::DeformationNode( const phd::DualQuaternion & transformation) {
        mTransformation = transformation;
        mPosition       =Eigen::Vector3d(0.0,0.0,0.0);
        mRadiusParameter=1.0;
    }
    
    DeformationNode::DeformationNode(const   Eigen::Vector3d &    position,
                                     double                       rotationAngle,
                                     const   Eigen::Vector3d &    rotationAxis,
                                     const   Eigen::Vector3d &    translation,
                                     double                       weight ) {
        mTransformation = phd::DualQuaternion( rotationAngle, rotationAxis, translation );
        mPosition = position;
        mRadiusParameter = weight;
    }
    
    
    /**
     * From Newcombe, R.A., Fox, D. & Seitz, S.M. 2015, 'DynamicFusion: Reconstruction and Tracking of Non-rigid Scenes in Real-Time'.
     *
     * Each of the 1..n nodes has a position in the canonical frame dg_v, its associated transformation Tic = dg_se3 ,
     * and a radial basis weight dg that controls the extent of the transformation
     */
    
    //
    //       -|mPosition - point|^2
    //   e^  ------------------------
    //        2(mRadiusParameter)^2
    //
    
    double DeformationNode::computeWeightAtPoint( const Eigen::Vector3d & point ) const {
        double numerator = (mPosition - point).norm();
        numerator = numerator * -numerator;
        double denominator = 2 * mRadiusParameter * mRadiusParameter;
        
        return exp( numerator / denominator);
    }
    
    
    DualQuaternion DeformationNode::transformationAtPoint( double x, double y, double z ) const {
        return transformationAtPoint(Eigen::Vector3d(x,y,z));
    }
    
    DualQuaternion DeformationNode::transformationAtPoint( const Eigen::Vector3d & point ) const {
        double weight = computeWeightAtPoint(point);
        return  mTransformation * weight;
    }
    
    double DeformationNode::maxEffectiveRange( ) const {
        // Calculate inverse of range function
        // And evaluate at EPS
        return sqrt( -2.0 * log( EPS ) * mRadiusParameter * mRadiusParameter);
    }
    
    // Sets the radius parameyer to make effect at range < EPS
    void DeformationNode::setMaxEffectiveRange( double range ) {
        
        mRadiusParameter = sqrt( (-0.5 * range * range) / log( EPS ) );
    }

    
}