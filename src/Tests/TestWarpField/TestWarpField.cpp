//
//  TestWarpField.cpp
//  WarpField
//
//  Created by Dave on 8/03/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include "Constants.hpp"
#include "../WarpField/WarpField.hpp"

TEST( TestWarpField, testConstructingEmptyWarpField ) {
    using namespace phd;


    WarpField wf{};

    // Should have no nodes
    EXPECT_EQ(wf.numberOfNodes(), 0);
}


TEST( TestWarpField, testThatAddingNodesWorks ) {
    using namespace phd;

    WarpField wf{};

    Eigen::Vector3d position{ 5, 5, 5 };
    double rotationAngle{ DEGREES_30 };
    Eigen::Vector3d rotationAxis{ X_AXIS };
    double weight{ 2.5 };
    Eigen::Vector3d translation{ 2.5, 1.5, 3.5 };

    wf.addNode( position, rotationAngle, rotationAxis, translation, weight );

    // Should have no nodes
    EXPECT_EQ(wf.numberOfNodes(), 1);
}


TEST( TestWarpField, testThatEmptyWarpFieldAffectsNoPoints ) {
    using namespace phd;

    WarpField wf{};

    for( int i=0; i<100; i++ ) {
        double rx = (random() % 200) - (random() % 100);
        double ry = (random() % 200) - (random() % 100);
        double rz = (random() % 200) - (random() % 100);

        EXPECT_FALSE( wf.isPointAffectedByWarpField(Eigen::Vector3d( rx, ry, rz )) );
    }
}

TEST( TestWarpField, testThatWarpFieldWithNodeHasBounds ) {
    using namespace phd;

    WarpField wf{};
    Eigen::Vector3d position{ 5, 5, 5 };
    double rotationAngle{ DEGREES_30 };
    Eigen::Vector3d rotationAxis{ X_AXIS };
    double weight{ 2.5 };
    Eigen::Vector3d translation{ 2.5, 1.5, 3.5 };

    wf.addNode( position, rotationAngle, rotationAxis, translation, weight );

    Eigen::AlignedBox3d box = wf.effectiveBounds();
    EXPECT_GT( 2.5, box.min().x());
    EXPECT_GT( 1.5, box.min().y());
    EXPECT_GT( 3.5, box.min().z());
    EXPECT_LT( 2.5, box.max().x());
    EXPECT_LT( 1.5, box.max().y());
    EXPECT_LT( 3.5, box.max().z());
}

TEST( TestWarpField, testThatPointAtNodeIsAffacted ) {
    using namespace phd;

    WarpField wf{};
    Eigen::Vector3d position{ 5, 5, 5 };
    double rotationAngle{ DEGREES_30 };
    Eigen::Vector3d rotationAxis{ X_AXIS };
    double weight{ 2.5 };
    Eigen::Vector3d translation{ 2.5, 1.5, 3.5 };

    wf.addNode( position, rotationAngle, rotationAxis, translation, weight );

    EXPECT_TRUE( wf.isPointAffectedByWarpField( Eigen::Vector3d( 2.5, 1.5, 3.5 ) ) );
}


TEST( TestWarpField, testThatPointTooFarOnXAxisIsUnaffacted ) {
    using namespace phd;

    WarpField wf{};
    Eigen::Vector3d position{ 5, 5, 5 };
    double rotationAngle{ DEGREES_30 };
    Eigen::Vector3d rotationAxis{ X_AXIS };
    double weight{ 2.5 };
    Eigen::Vector3d translation{ 2.5, 1.5, 3.5 };

    DeformationNode dn{ position, rotationAngle, rotationAxis, translation, weight };
    dn.setMaxEffectiveRange(10);
    wf.addNode(dn);

    EXPECT_FALSE( wf.isPointAffectedByWarpField( Eigen::Vector3d( 16, 5, 5 ) ) );
}

TEST( TestWarpField, testThatPointTooFarOnYAxisIsUnaffacted ) {
    using namespace phd;

    WarpField wf{};
    Eigen::Vector3d position{ 5, 5, 5 };
    double rotationAngle{ DEGREES_30 };
    Eigen::Vector3d rotationAxis{ X_AXIS };
    double weight{ 2.5 };
    Eigen::Vector3d translation{ 2.5, 1.5, 3.5 };

    DeformationNode dn{ position, rotationAngle, rotationAxis, translation, weight };
    dn.setMaxEffectiveRange(10);
    wf.addNode(dn);

    EXPECT_FALSE( wf.isPointAffectedByWarpField( Eigen::Vector3d( 5, 16, 5 ) ) );
}

TEST( TestWarpField, testThatPointTooFarOnZAxisIsUnaffacted ) {
    using namespace phd;

    WarpField wf{};
    Eigen::Vector3d position{ 5, 5, 5 };
    double rotationAngle{ DEGREES_30 };
    Eigen::Vector3d rotationAxis{ X_AXIS };
    double weight{ 2.5 };
    Eigen::Vector3d translation{ 2.5, 1.5, 3.5 };

    DeformationNode dn{ position, rotationAngle, rotationAxis, translation, weight };
    dn.setMaxEffectiveRange(10);
    wf.addNode(dn);

    EXPECT_FALSE( wf.isPointAffectedByWarpField( Eigen::Vector3d( 5, 5, 16 ) ) );
}

