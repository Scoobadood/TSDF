//
//  TestDeformationNode.cpp
//  WarpField
//
//  Created by Dave on 7/03/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include "Constants.hpp"
#include "DeformationNode.hpp"


TEST( TestDeformationNode, testThatDefaultCtorProducesIdentity ) {
    using namespace phd;
    
    DeformationNode dn{};
    
    EXPECT_EQ( dn.getRadiusParameter(), 1.0);
    EXPECT_EQ( dn.getPosition(), Eigen::Vector3d(0.0,0.0,0.0) );
    EXPECT_EQ( dn.getTransformation(), phd::DualQuaternion( IDENTITY_ROTATION, NULL_QUATERNION ) );
}



TEST( TestDeformationNode, testThatTransforationAtPointIsFull ) {
    using namespace phd;
    
    DualQuaternion transformation{ DEGREES_30, X_AXIS, IDENTITY_TRANSLATION };
    DeformationNode dn{ transformation};
    
    dn.setPosition( Eigen::Vector3d( 1.0, 2.0, 3.0) );
    
    DualQuaternion dq = dn.transformationAtPoint(Eigen::Vector3d( 1.0, 2.0, 3.0) );
    
    EXPECT_EQ( dq.getRotation(), transformation.getRotation() );
    EXPECT_EQ( dq.getTranslation(), transformation.getTranslation() );
}


TEST( TestDeformationNode, testRotationBlendingWithTwoNodes1 ) {
    using namespace phd;

    // Setup first node
    DualQuaternion t1{ DEGREES_30, X_AXIS, IDENTITY_TRANSLATION };
    DeformationNode dn1{ t1};
    dn1.setPosition( 0, 0, 0 );
    

    // Set up second node
    DualQuaternion t2{ -DEGREES_30, X_AXIS, IDENTITY_TRANSLATION };
    DeformationNode dn2{ t2};
    dn2.setPosition( 10, 0, 0 );
    
    // Compute combined effect at point
    DualQuaternion pt1 = dn1.transformationAtPoint( 0, 0, 0 );
    DualQuaternion pt2 = dn2.transformationAtPoint( 0, 0, 0 );
    DualQuaternion blend = (pt1 + pt2);
    
    EXPECT_EQ( blend.getRotation(), t1.getRotation() );
    EXPECT_EQ( blend.getTranslation(), t1.getTranslation() );
    
    // DIAGNOSTIC DUMPING //
    // We expect this to be all t1
    double blendedAngle, angle;
    Eigen::Vector3d blendedAxis, axis;
    blend.getRotation().getAxisAngle(blendedAngle, blendedAxis);
    t1.getRotation().getAxisAngle(angle, axis);
    std::cout << "Original transform t1\n\t\tAngle : " << (180.0 / M_PI) * angle << "\n\t\tAxis  : " << axis << std::endl;
    std::cout << "Blended  transform   \n\t\tAngle : " << (180.0 / M_PI) * blendedAngle << "\n\t\tAxis  : " << blendedAxis << std::endl;
    // END        DUMPING //
}

TEST( TestDeformationNode, testRotationBlendingWithTwoNodes2 ) {
    using namespace phd;
    
    DualQuaternion t1{ DEGREES_30, X_AXIS, IDENTITY_TRANSLATION };
    DualQuaternion t2{ -DEGREES_30, X_AXIS, IDENTITY_TRANSLATION };
    DeformationNode dn1{ t1};
    DeformationNode dn2{ t2};
    
    dn1.setPosition( 0, 0, 0 );
    dn2.setPosition( 10, 0, 0 );
    
    DualQuaternion pt1 = dn1.transformationAtPoint( 10, 0, 0 );
    DualQuaternion pt2 = dn2.transformationAtPoint( 10, 0, 0 );
    
    DualQuaternion blend = pt1 + pt2;
    
    EXPECT_EQ( blend.getRotation(), t2.getRotation());
    EXPECT_EQ( blend.getTranslation(), t2.getTranslation());
}

TEST( TestDeformationNode, testRotationBlendingWithTwoNodesMidway ) {
    using namespace phd;
    
    DualQuaternion t1{ DEGREES_30, X_AXIS, IDENTITY_TRANSLATION };
    DualQuaternion t2{ -DEGREES_30, X_AXIS, IDENTITY_TRANSLATION };
    DeformationNode dn1{ t1};
    DeformationNode dn2{ t2};
    
    dn1.setRadiusParameter(7);
    dn2.setRadiusParameter(7);
    
    dn1.setPosition( 0, 0, 0 );
    dn2.setPosition( 10, 0, 0 );
    
    DualQuaternion pt1 = dn1.transformationAtPoint( 5, 0, 0 );
    DualQuaternion pt2 = dn2.transformationAtPoint( 5, 0, 0 );
    
    DualQuaternion blend = (pt1 + pt2);
    
    
    EXPECT_EQ( blend.getRotation(), IDENTITY_ROTATION);
    EXPECT_EQ( blend.getTranslation(), IDENTITY_TRANSLATION);
}

TEST( TestDeformationNode, testRotationBlendingBetweenTwo ) {
    using namespace phd;
    
    DualQuaternion t1{ DEGREES_30, X_AXIS, IDENTITY_TRANSLATION };
    DualQuaternion t2{ -DEGREES_30, X_AXIS, IDENTITY_TRANSLATION };
    DeformationNode dn1{ t1};
    DeformationNode dn2{ t2};
    
    dn1.setRadiusParameter(3);
    dn2.setRadiusParameter(3);
    
    dn1.setPosition( 0, 0, 0 );
    dn2.setPosition( 10, 0, 0 );
    
    for (double px = 0.0; px <= 5.0; px += 0.5 ) {
        DualQuaternion pt1 = dn1.transformationAtPoint( px, 0, 0 );
        DualQuaternion pt2 = dn2.transformationAtPoint( px, 0, 0 );
        DualQuaternion blend1 = (pt1 + pt2);
        double angle1;
        Eigen::Vector3d axis1;
        blend1.getRotation().getAxisAngle(angle1, axis1);
        
        pt1 = dn1.transformationAtPoint( 10.0-px, 0, 0 );
        pt2 = dn2.transformationAtPoint( 10.0-px, 0, 0 );
        DualQuaternion blend2 = (pt1 + pt2);
        double angle2;
        Eigen::Vector3d axis2;
        blend2.getRotation().getAxisAngle(angle2, axis2);
        
        
        // Angles should be same - axis varies for negative angles
        EXPECT_NEAR( angle1, angle2, EPS );
        
        // Axes should be parallel but opposite unless angle is 0 in which case they can be anything
        if( angle1 != 0 ) {
            Eigen::Vector3d comboAxis = axis1 + axis2;
            EXPECT_EQ( comboAxis, IDENTITY_TRANSLATION);
        } else {
            EXPECT_EQ( axis1, axis2);
        }
    }

}

TEST( TestDeformationNode, testTranslationBlendingWithTwoNodes1 ) {
    using namespace phd;
    
    Eigen::Vector3d translation{ 1.0, 2.0, 3.0 };
    DualQuaternion t1{ 0, X_AXIS, translation };
    DualQuaternion t2{ 0, X_AXIS, -translation };
    DeformationNode dn1{ t1};
    DeformationNode dn2{ t2};
    
    dn1.setPosition( 0, 0, 0 );
    dn2.setPosition( 10, 0, 0 );
    
    DualQuaternion pt1 = dn1.transformationAtPoint( 0, 0, 0 );
    DualQuaternion pt2 = dn2.transformationAtPoint( 0, 0, 0 );
    
    DualQuaternion blend = (pt1 + pt2);
    
    
    EXPECT_EQ( blend.getRotation(), IDENTITY_ROTATION );
    EXPECT_EQ( blend.getRotation(), t1.getRotation() );
    EXPECT_EQ( blend.getTranslation(), translation);
}

TEST( TestDeformationNode, testTranslationBlendingWithTwoNodes2 ) {
    using namespace phd;
    
    Eigen::Vector3d translation{ 1.0, 2.0, 3.0 };
    DualQuaternion t1{ 0, X_AXIS, translation };
    DualQuaternion t2{ 0, X_AXIS, -translation };
    DeformationNode dn1{ t1};
    DeformationNode dn2{ t2};
    
    dn1.setPosition( 0, 0, 0 );
    dn2.setPosition( 10, 0, 0 );
    
    DualQuaternion pt1 = dn1.transformationAtPoint( 10, 0, 0 );
    DualQuaternion pt2 = dn2.transformationAtPoint( 10, 0, 0 );
    
    DualQuaternion blend = (pt1 + pt2);
    
    
    EXPECT_EQ( blend.getRotation(), IDENTITY_ROTATION );
    EXPECT_EQ( blend.getRotation(), t1.getRotation() );
    EXPECT_EQ( blend.getTranslation(), -translation);
}

TEST( TestDeformationNode, testTranslationBlendingWithTwoNodesMidway ) {
    using namespace phd;
    
    Eigen::Vector3d translation{ 1.0, 2.0, 3.0 };
    DualQuaternion t1{ 0, X_AXIS, translation };
    DualQuaternion t2{ 0, X_AXIS, -translation };
    DeformationNode dn1{ t1};
    DeformationNode dn2{ t2};
    
    dn1.setPosition( 0, 0, 0 );
    dn2.setPosition( 10, 0, 0 );
    
    DualQuaternion pt1 = dn1.transformationAtPoint( 5, 0, 0 );
    DualQuaternion pt2 = dn2.transformationAtPoint( 5, 0, 0 );
    
    DualQuaternion blend = (pt1 + pt2);
    
    
    EXPECT_EQ( blend.getRotation(), IDENTITY_ROTATION );
    EXPECT_EQ( blend.getRotation(), t1.getRotation() );
    EXPECT_EQ( blend.getTranslation(), IDENTITY_TRANSLATION);
}

TEST( TestDeformationNode, testThatMaxEffectiveRangeIsCorrect ) {
    using namespace phd;
    
    DeformationNode node{ Eigen::Vector3d(0.0, 0.0, 0.0), // Position
                          DEGREES_30,                       // Rotation angle
                          X_AXIS,                           // Rotation axis
                          IDENTITY_TRANSLATION,             // translation
                          10.0};
    
    node.setMaxEffectiveRange(400.0);
    EXPECT_NEAR( node.getRadiusParameter(), 76.0, 1.0 );
}

TEST( TestDeformationNode, testThatSettingMaxEffectiveRangeIsCorrect ) {
    using namespace phd;
    
    DeformationNode node{ Eigen::Vector3d(0.0, 0.0, 0.0), // Position
        DEGREES_30,                       // Rotation angle
        X_AXIS,                           // Rotation axis
        IDENTITY_TRANSLATION,             // translation
        10.0};
    
    node.setRadiusParameter(76.0);
    EXPECT_NEAR( node.maxEffectiveRange(), 400.0, 1.0);
}
