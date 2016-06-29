//
//  main.cpp
//  TestDualQuaternion
//
//  Created by Dave on 3/03/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include "Constants.hpp"
#include "DualQuaternion.hpp"


TEST( TestDualQuaternion, testThatDefaultQuaternionIsIdentityMatrix ) {
    using namespace phd;
    
    // Test that it converts to a matrix OK
    DualQuaternion dq1;
    
    Eigen::Matrix4d m = dq1.toMatrix();
    Eigen::Matrix4d i = Eigen::Matrix4d::Identity();
    
    EXPECT_EQ( m, i) << "Expected matrix for default DualQuaternion to be the Identity";
}

TEST( TestDualQuaternion, testQuaternionConstructor ) {
    using namespace phd;
    
    Quaternion realPart{ 1, 0, 0, 0};
    Quaternion dualPart{ 0, 0, 0, 0};
    
    
    // Test that it converts to a matrix OK
    DualQuaternion dq1{ realPart, dualPart };
    
    Eigen::Matrix4d m = dq1.toMatrix();
    Eigen::Matrix4d i = Eigen::Matrix4d::Identity();
    
    EXPECT_EQ( m, i) << "Expected matrix for default DualQuaternion to be the Identity";
}


TEST( TestDualQuaternion, testThatConstructionWithRotationAndTranlsationIsUnitDQ) {
    using namespace phd;
    using namespace Eigen;
    
    DualQuaternion qd{ M_PI_4, X_AXIS, Vector3d( 1, 2, 3) };
    
    EXPECT_EQ( qd.magnitude(), 1.0 );
}

TEST( TestDualQuaternion, testThatConstructionWithTranlsationIsUnitDQ) {
    using namespace phd;
    using namespace Eigen;
    
    DualQuaternion qd{ Vector3d( 1, 2, 3) };
    
    EXPECT_EQ( qd.magnitude(), 1.0 );
}

TEST( TestDualQuaternion, testThatConstructionWithRotationIsUnitDQ) {
    using namespace phd;
    using namespace Eigen;
    
    DualQuaternion qd{ M_PI_4, X_AXIS };
    
    EXPECT_EQ( qd.magnitude(), 1.0 );
}

TEST( TestDualQuaternion, testEquality) {
    using namespace phd;
    
    // sq = sq_r + sq_d E
    Quaternion qr1{ 1, 2, 3, 4};
    Quaternion qd1{ 5, 6, 7, 8};
    DualQuaternion dq1{qr1, qd1};
    
    Quaternion qr2{ 1, 2, 3, 4};
    Quaternion qd2{ 5, 6, 7, 8};
    DualQuaternion dq2{ qr2, qd2 };
    
    EXPECT_EQ( dq1, dq2 ) << "Equality operator failed";
}


TEST( TestDualQuaternion, testScalarPreMultiplication) {
    using namespace phd;
    
    // sq = sq_r + sq_d E
    Quaternion qr{ 1, 2, 3, 4};
    Quaternion qd{ 5, 6, 7, 8};
    
    DualQuaternion dq{qr, qd};
    double s = 3.7;
    DualQuaternion sdq = dq * s;
    
    Quaternion er{ s, s*2, s*3, s*4};
    Quaternion ed{ s*5, s*6, s*7, s*8};
    DualQuaternion e{ er, ed};
    
    EXPECT_EQ( sdq, e ) << "Scalar pre multiplication yielded incorrect result";
}

TEST( TestDualQuaternion, testScalarPostMultiplication) {
    using namespace phd;
    
    // sq = sq_r + sq_d E
    Quaternion qr{ 1, 2, 3, 4};
    Quaternion qd{ 5, 6, 7, 8};
    
    DualQuaternion dq{qr, qd};
    double s = 3.7;
    DualQuaternion dqs = dq * s;
    
    Quaternion er{ s*1, s*2, s*3, s*4};
    Quaternion ed{ s*5, s*6, s*7, s*8};
    DualQuaternion e{ er, ed};
    
    EXPECT_EQ( dqs, e ) << "Scalar post multiplication yielded incorrect result";
}

TEST( TestDualQuaternion, testScalarMultiplicationNegative) {
    using namespace phd;
    
    // sq = sq_r + sq_d E
    Quaternion qr{ 1, 2, 3, 4};
    Quaternion qd{ 5, 6, 7, 8};
    
    DualQuaternion dq{qr, qd};
    DualQuaternion dq2 = dq;
    
    double s = -1.3;
    DualQuaternion sdq = dq * s;
    
    Quaternion er{ s, s*2, s*3, s*4};
    Quaternion ed{ s*5, s*6, s*7, s*8};
    DualQuaternion e{ er, ed};
    
    EXPECT_EQ( sdq, e ) << "Scalar multiplication yielded incorrect result";
    EXPECT_EQ( dq2, dq ) << "Scalar multiplication should not change original value";
}

TEST( TestDualQuaternion, testAddition ) {
    using namespace phd;
    
    // q1+q2 = (q_r1+q_r2) + (q_d1+q_d2) E
    Quaternion qr1{ 1, 2, 3, 4};
    Quaternion qd1{ 5, 6, 7, 8};
    
    Quaternion qr2{ 10, 20, 30, 40};
    Quaternion qd2{ 50, 60, 70, 80};
    
    DualQuaternion dq1{qr1, qd1};
    DualQuaternion dq2{qr2, qd2};
    
    DualQuaternion actual = dq1 + dq2;
    
    Quaternion expectedRealPart = qr1 + qr2;
    Quaternion expectedDualPart = qd1 + qd2;
    DualQuaternion ex{ expectedRealPart, expectedDualPart };
    
    EXPECT_EQ( actual, ex ) << "Addition yielded incorrect result";
}

TEST( TestDualQuaternion, testMultiplication ) {
    using namespace phd;
    
    // q1q2 = (q_r1*q_r2) + (q_r1*q_d2 + q_d1*q_r2) E
    
    Quaternion qr1{ 1, 2, 3, 4};
    Quaternion qd1{ 5, 6, 7, 8};
    
    Quaternion qr2{ 15, 25, 35, 45
    };
    Quaternion qd2{ 50, 60, 70, 80};
    
    DualQuaternion dq1{qr1, qd1};
    DualQuaternion dq2{qr2, qd2};
    
    DualQuaternion actual = dq1 * dq2;
    
    Quaternion exr = (qr1 * qr2);
    Quaternion q1 = qr1 * qd2;
    Quaternion q2 = qd1 * qr2;
    Quaternion exd = q1 + q2;
    DualQuaternion ex{ exr, exd};
    
    EXPECT_EQ( actual, ex ) << "Multiplication yielded incorrect result";
}


#pragma mark - X Rotations

TEST( TestDualQuaternion, testRotationAroundXAxisOfPointOnXAxis ) {
    using namespace phd;
    
    double theta = M_PI / 6.0;
    
    DualQuaternion dq{ Quaternion{ theta, Eigen::Vector3d{ 1, 0, 0 } } };
    Eigen::Vector3d point_in{ 1, 0, 0 };
    
    Eigen::Vector3d point_out = dq.transformPoint(point_in);
    EXPECT_NEAR( point_out.x(), 1, EPS );
    EXPECT_NEAR( point_out.y(), 0, EPS );
    EXPECT_NEAR( point_out.z(), 0, EPS );
}

TEST( TestDualQuaternion, testRotationAroundXAxisOfPointOnYAxis ) {
    using namespace phd;
    
    double theta = M_PI / 6.0;
    
    DualQuaternion dq{ Quaternion{ theta, Eigen::Vector3d{ 1, 0, 0 } } };
    Eigen::Vector3d point_in{ 0, 1, 0 };
    
    Eigen::Vector3d point_out = dq.transformPoint(point_in);
    EXPECT_NEAR( point_out.x(), 0, EPS );
    EXPECT_NEAR( point_out.y(), HALF_ROOT_3, EPS );
    EXPECT_NEAR( point_out.z(), 0.5, EPS );
}

TEST( TestDualQuaternion, testRotationAroundXAxisOfPointOnZAxis ) {
    using namespace phd;
    
    double theta = M_PI / 6.0;
    
    DualQuaternion dq{ Quaternion{ theta, Eigen::Vector3d{ 1, 0, 0 } } };
    Eigen::Vector3d point_in{ 0, 0, 1 };
    
    Eigen::Vector3d point_out = dq.transformPoint(point_in);
    EXPECT_NEAR( point_out.x(), 0, EPS );
    EXPECT_NEAR( point_out.y(), -0.5, EPS );
    EXPECT_NEAR( point_out.z(), HALF_ROOT_3, EPS );
}




#pragma mark - Y Rotations

TEST( TestDualQuaternion, testRotationAroundYAxisOfPointOnXAxis ) {
    using namespace phd;
    
    double theta = M_PI / 6.0;
    
    DualQuaternion dq{ Quaternion{ theta, Eigen::Vector3d{ 0, 1, 0 } } };
    Eigen::Vector3d point_in{ 1, 0, 0 };
    
    Eigen::Vector3d point_out = dq.transformPoint(point_in);
    EXPECT_NEAR( point_out.x(), HALF_ROOT_3, EPS );
    EXPECT_NEAR( point_out.y(), 0, EPS );
    EXPECT_NEAR( point_out.z(), -0.5, EPS );
}

TEST( TestDualQuaternion, testRotationAroundYAxisOfPointOnYAxis ) {
    using namespace phd;
    
    double theta = M_PI / 6.0;
    
    DualQuaternion dq{ Quaternion{ theta, Eigen::Vector3d{ 0, 1, 0 } } };
    Eigen::Vector3d point_in{ 0, 1, 0 };
    
    Eigen::Vector3d point_out = dq.transformPoint(point_in);
    EXPECT_NEAR( point_out.x(), 0, EPS );
    EXPECT_NEAR( point_out.y(), 1, EPS );
    EXPECT_NEAR( point_out.z(), 0, EPS );
}

TEST( TestDualQuaternion, testRotationAroundYAxisOfPointOnZAxis ) {
    using namespace phd;
    
    double theta = M_PI / 6.0;
    
    DualQuaternion dq{ Quaternion{ theta, Eigen::Vector3d{ 0, 1, 0 } } };
    Eigen::Vector3d point_in{ 0, 0, 1 };
    
    Eigen::Vector3d point_out = dq.transformPoint(point_in);
    EXPECT_NEAR( point_out.x(), 0.5, EPS );
    EXPECT_NEAR( point_out.y(), 0, EPS );
    EXPECT_NEAR( point_out.z(), HALF_ROOT_3, EPS );
}




#pragma mark - Z Rotations

TEST( TestDualQuaternion, testRotationAroundZAxisOfPointOnXAxis ) {
    using namespace phd;
    
    double theta = M_PI / 6.0;
    
    DualQuaternion dq{ Quaternion{ theta, Eigen::Vector3d{ 0, 0, 1 } } };
    Eigen::Vector3d point_in{ 1, 0, 0 };
    
    Eigen::Vector3d point_out = dq.transformPoint(point_in);
    EXPECT_NEAR( point_out.x(), HALF_ROOT_3, EPS );
    EXPECT_NEAR( point_out.y(), 0.5, EPS );
    EXPECT_NEAR( point_out.z(), 0, EPS );
}

TEST( TestDualQuaternion, testRotationAroundZAxisOfPointOnYAxis ) {
    using namespace phd;
    
    double theta = M_PI / 6.0;
    
    DualQuaternion dq{ Quaternion{ theta, Eigen::Vector3d{ 0, 0, 1 } } };
    Eigen::Vector3d point_in{ 0, 1, 0 };
    
    Eigen::Vector3d point_out = dq.transformPoint(point_in);
    EXPECT_NEAR( point_out.x(), -0.5, EPS );
    EXPECT_NEAR( point_out.y(), HALF_ROOT_3, EPS );
    EXPECT_NEAR( point_out.z(), 0, EPS );
}

TEST( TestDualQuaternion, testRotationAroundZAxisOfPointOnZAxis ) {
    using namespace phd;
    
    double theta = M_PI / 6.0;
    
    DualQuaternion dq{ Quaternion{ theta, Eigen::Vector3d{ 0, 0, 1 } } };
    Eigen::Vector3d point_in{ 0, 0, 1 };
    
    Eigen::Vector3d point_out = dq.transformPoint(point_in);
    EXPECT_NEAR( point_out.x(), 0, EPS );
    EXPECT_NEAR( point_out.y(), 0, EPS );
    EXPECT_NEAR( point_out.z(), 1, EPS );
}

#pragma mark - Translation

TEST( TestDualQuaternion, testRandomTranslation ) {
    using namespace phd;

    double dx = arc4random();
    double dy = arc4random();
    double dz = arc4random();
    
    DualQuaternion dq{ Eigen::Vector3d{ dx, dy, dz } };
    
    Eigen::Vector3d point_in{ 1, 2, 3 };
    
    Eigen::Vector3d point_out = dq.transformPoint(point_in);
    
    EXPECT_NEAR( point_out.x(), 1.0 + dx, EPS );
    EXPECT_NEAR( point_out.y(), 2.0 + dy, EPS );
    EXPECT_NEAR( point_out.z(), 3.0 + dz, EPS );
}

TEST( TestDualQuaternion, testRandomRotationPlusTranslation ) {
    using namespace phd;

    Quaternion r{ M_PI / 2.0, Eigen::Vector3d{ 1, 0, 0 } };
    DualQuaternion dq{ r, Eigen::Vector3d{ 2.5, 1.5, 0.5 } };
    
    Eigen::Vector3d point_in{ 1, 2, 3 };
    Eigen::Vector3d point_out = dq.transformPoint(point_in);
    
    
    EXPECT_NEAR( point_out.x(), 3.5, EPS );
    EXPECT_NEAR( point_out.y(), -1.5, EPS );
    EXPECT_NEAR( point_out.z(), 2.5, EPS );
}


