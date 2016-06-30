//
//  TestQuaternion.cpp
//  WarpField
//
//  Created by Dave on 3/03/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include "Constants.hpp"

TEST( TestQuaternion, test4dConstructor ) {
    using namespace phd;

    Quaternion q{ 1.0, 2.0, 3.0, 4.0 };

    EXPECT_EQ( q.w(), 1.0) << "w should be 1.0";
    EXPECT_EQ( q.x(), 2.0) << "x should be 2.0";
    EXPECT_EQ( q.y(), 3.0) << "y should be 3.0";
    EXPECT_EQ( q.z(), 4.0) << "z should be 4.0";
}

TEST( TestQuaternion, testVecConstructorPiX ) {
    using namespace phd;

    // PI about x axis
    Quaternion q{ 2 * M_PI, Eigen::Vector3d{1.0, 0.0, 0.0} };

    EXPECT_NEAR( q.w(), -1, 1e-6 ) << "w should be cos(PI) == -1.0";
    EXPECT_NEAR( q.x(), 0.0, 1e-6 ) << "x should be sin(PI) == 0.0";
    EXPECT_NEAR( q.y(), 0.0, 1e-6) << "y should be 0.0";
    EXPECT_NEAR( q.z(), 0.0, 1e-6) << "z should be 0.0";
}

TEST( TestQuaternion, testVecConstructorPiY ) {
    using namespace phd;

    // PI about x axis
    Quaternion q{ 2 * M_PI, Eigen::Vector3d{0.0, 1.0, 0.0} };

    EXPECT_NEAR( q.w(), -1, 1e-6 ) << "w should be cos(PI) == -1.0";
    EXPECT_NEAR( q.x(), 0.0, 1e-6 ) << "x should be 0.0";
    EXPECT_NEAR( q.y(), 0.0, 1e-6) << "y sin(PI) == should be 0.0";
    EXPECT_NEAR( q.z(), 0.0, 1e-6) << "z should be 0.0";
}

TEST( TestQuaternion, testVecConstructorPiZ ) {
    using namespace phd;

    // PI about x axis
    Quaternion q{ 2 * M_PI, Eigen::Vector3d{0.0, 0.0, 1.0} };

    EXPECT_NEAR( q.w(), -1, 1e-6 ) << "w should be cos(PI) == -1.0";
    EXPECT_NEAR( q.x(), 0.0, 1e-6 ) << "x should be 0.0";
    EXPECT_NEAR( q.y(), 0.0, 1e-6) << "y should be 0.0";
    EXPECT_NEAR( q.z(), 0.0, 1e-6) << "z should be sin(PI) == 0.0";
}

TEST( TestQuaternion, testVecConstructor3Pi2X ) {
    using namespace phd;

    // PI about x axis
    Quaternion q{ (2.0/3.0) * M_PI, Eigen::Vector3d{1.0, 0.0, 0.0} };

    EXPECT_NEAR( q.w(), 0.5, 1e-6 ) << "w should be cos(PI) == 0.5";
    EXPECT_NEAR( q.x(), 0.86602540378, 1e-6 ) << "x should be sin(3PI/2) == 0.86602540378";
    EXPECT_NEAR( q.y(), 0.0, 1e-6) << "y should be 0.0";
    EXPECT_NEAR( q.z(), 0.0, 1e-6) << "z should be 0.0";
}

TEST( TestQuaternion, testVecConstructor3Pi2Y ) {
    using namespace phd;

    // PI about x axis
    Quaternion q{ (2.0/3.0)*M_PI, Eigen::Vector3d{0.0, 1.0, 0.0} };

    EXPECT_NEAR( q.w(), 0.5, 1e-6 ) << "w should be cos(PI) == 0.5";
    EXPECT_NEAR( q.x(), 0.0, 1e-6 ) << "x should be 0.0";
    EXPECT_NEAR( q.y(), 0.86602540378, 1e-6) << "y sin(3PI/2) == should be 0.86602540378";
    EXPECT_NEAR( q.z(), 0.0, 1e-6) << "z should be 0.0";
}

TEST( TestQuaternion, testVecConstructor23PiZ ) {
    using namespace phd;

    // PI about x axis
    Quaternion q{ (2.0/3.0)*M_PI, Eigen::Vector3d{0.0, 0.0, 1.0} };

    EXPECT_NEAR( q.w(), 0.5, 1e-6 ) << "w should be cos(PI/3) == 0.5";
    EXPECT_NEAR( q.x(), 0.0, 1e-6 ) << "x should be 0.0";
    EXPECT_NEAR( q.y(), 0.0, 1e-6) << "y should be 0.0";
    EXPECT_NEAR( q.z(), 0.86602540378, 1e-6) << "z should be sin(3PI/2) == 0.86602540378";
}

TEST( TestQuaternion, testAssignment ) {
    using namespace phd;

    Quaternion q{ 1.0, Eigen::Vector3d{2.0, 3.0, 4.0} };
    Quaternion q2 = q;

    EXPECT_EQ( q.w(), q2.w());
    EXPECT_EQ( q.x(), q2.x());
    EXPECT_EQ( q.y(), q2.y());
    EXPECT_EQ( q.z(), q2.z());
}


TEST( TestQuaternion, testEquality ) {
    using namespace phd;

    Quaternion q{ 1.0, Eigen::Vector3d{2.0, 3.0, 4.0} };
    Quaternion q2{1.0, Eigen::Vector3d{2.0, 3.0, 4.0} };

    EXPECT_EQ( q, q2);
}


TEST( TestQuaternion, testInequality ) {
    using namespace phd;

    Quaternion q{ 1.0, Eigen::Vector3d{2.0, 3.0, 4.0} };
    Quaternion q2{2.0, Eigen::Vector3d{2.0, 3.0, 4.0} };

    EXPECT_NE( q, q2);
}

TEST( TestQuaternion, testScalarPostMultiplication ) {
    using namespace phd;

    Quaternion q{ 1.0, Eigen::Vector3d{2.0, 3.0, 4.0} };
    double scalar = 1.5;

    Quaternion qs = q * scalar;

    EXPECT_EQ( qs.w(), q.w() * scalar);
    EXPECT_EQ( qs.x(), q.x() * scalar);
    EXPECT_EQ( qs.y(), q.y() * scalar);
    EXPECT_EQ( qs.z(), q.z() * scalar);
}

TEST( TestQuaternion, testScalarPreMultiplication ) {
    using namespace phd;

    Quaternion q{ 1.0, Eigen::Vector3d{2.0, 3.0, 4.0} };
    double scalar = 1.5;

    Quaternion qs = scalar * q;

    EXPECT_EQ( qs.w(), q.w() * scalar);
    EXPECT_EQ( qs.x(), q.x() * scalar);
    EXPECT_EQ( qs.y(), q.y() * scalar);
    EXPECT_EQ( qs.z(), q.z() * scalar);
}

TEST( TestQuaternion, testQuaternionMultiplication ) {
    using namespace phd;

    Quaternion q1{ 1.0, Eigen::Vector3d{2.0, 3.0, 4.0} };
    Quaternion q2{ 5.0, Eigen::Vector3d{6.0, 7.0, 8.0} };

    Quaternion q1q2 = q1 * q2;

    EXPECT_FLOAT_EQ( q1q2.w(), q1.w() * q2.w() - q1.vec().dot(q2.vec()));
}

TEST( TestQuaternion, testConjugate ) {
    using namespace phd;

    Quaternion q1{ 1.0, Eigen::Vector3d{2.0, 3.0, 4.0} };
    Quaternion q2 = q1.conjugate();

    EXPECT_EQ( q1.w(), q2.w() ) << "W values should be the same for conjugate";
    EXPECT_EQ( q1.x(), -q2.x() ) << "X values should be negated for conjugate";
    EXPECT_EQ( q1.y(), -q2.y() ) << "Y values should be negated for conjugate";
    EXPECT_EQ( q1.z(), -q2.z() ) << "Z values should be negated for conjugate";
}

TEST( TestQuaternion, testConjugateCommutativity ) {
    using namespace phd;

    Quaternion q1{ M_PI, Eigen::Vector3d{2.0, 3.0, 4.0} };
    Quaternion q2{ M_PI/6.0, Eigen::Vector3d{ 9.0, 4.5, 1.2 } };

    Quaternion q1q2 = q1 * q2;
    Quaternion q1q2c = q1q2.conjugate();

    Quaternion q1c = q1.conjugate();
    Quaternion q2c = q2.conjugate();
    Quaternion q2cq1c = q2c * q1c;


    EXPECT_NEAR( q1q2c.w(), q2cq1c.w(), EPS ) << "W values should be the same for conjugate";
    EXPECT_NEAR( q1q2c.x(), q2cq1c.x(), EPS ) << "X values should be negated for conjugate";
    EXPECT_NEAR( q1q2c.y(), q2cq1c.y(), EPS ) << "Y values should be negated for conjugate";
    EXPECT_NEAR( q1q2c.z(), q2cq1c.z(), EPS ) << "Z values should be negated for conjugate";
}

TEST( TestQuaternion, testMultiplyingByConjugateGivesIdentity ) {
    using namespace phd;

    Quaternion q1{ M_PI/6.0, {1,2,3} };
    Quaternion q2 = q1.conjugate();

    Quaternion i = (q1 * q2);
    Quaternion i2 = q2 * q1;

    EXPECT_NEAR( i.x(), 0.0, EPS ) << "X should be 0";
    EXPECT_NEAR( i.y(), 0.0, EPS ) << "Y should be 0";
    EXPECT_NEAR( i.z(), 0.0, EPS ) << "Z should be 0";

    EXPECT_NEAR( i2.x(), 0.0, EPS ) << "X should be 0";
    EXPECT_NEAR( i2.y(), 0.0, EPS ) << "Y should be 0";
    EXPECT_NEAR( i2.z(), 0.0, EPS ) << "Z should be 0";


    EXPECT_NEAR( i2.w(), i.w(), EPS ) << "W should be 1.0";
}



TEST( TestQuaternion, testMagnitude ) {
    using namespace phd;

    Quaternion q{ cos(M_PI/2), 1.0, 0.0, 0.0 };
    Quaternion q_c = q.conjugate();

    Quaternion qq_c = q * q_c;
    EXPECT_NEAR( qq_c.w(), qq_c.magnitude() , EPS );
}

TEST( TestQuaternion, testQuaternionAddition ) {
    using namespace phd;

    Quaternion q1{ 1.0, Eigen::Vector3d{2.0, 3.0, 4.0} };
    Quaternion q2{ 9.0, 8.0, 7.0, 6.0 };

    Quaternion q3 = q1+q2;

    EXPECT_EQ( q3.w(), q2.w() + q1.w() );
    EXPECT_EQ( q3.x(), q2.x() + q1.x() ) << "X values should be negated for conjugate";
    EXPECT_EQ( q3.y(), q2.y() + q1.y() ) << "Y values should be negated for conjugate";
    EXPECT_EQ( q3.z(), q2.z() + q1.z() ) << "Z values should be negated for conjugate";
}


TEST( TestQuaternion, testRotationAboutXAxisOfXPoint ) {
    using namespace phd;

    double theta = M_PI / 6.0;

    Quaternion q(  theta , Eigen::Vector3d{1.0, 0.0, 0.0} );

    Eigen::Vector3d point_in{1.0, 0.0, 0.0};

    Eigen::Vector3d point_out = q.rotate( point_in );

    EXPECT_NEAR( point_out.x(), 1.0, EPS );
    EXPECT_NEAR( point_out.y(), 0.0, EPS );
    EXPECT_NEAR( point_out.z(), 0.0, EPS );

}

TEST( TestQuaternion, testRotationAboutXAxisOfYPoint ) {
    using namespace phd;

    double theta = M_PI / 6.0;

    Quaternion q(  theta , Eigen::Vector3d{1.0, 0.0, 0.0} );

    Eigen::Vector3d point_in{0.0, 1.0, 0.0};

    Eigen::Vector3d point_out = q.rotate( point_in );

    EXPECT_NEAR( point_out.x(), 0.0, EPS );
    EXPECT_NEAR( point_out.y(), HALF_ROOT_3, EPS );
    EXPECT_NEAR( point_out.z(), 0.5, EPS );

}


TEST( TestQuaternion, testRotationAboutXAxisOfZPoint ) {
    using namespace phd;

    double theta = M_PI / 6.0;

    Quaternion q(  theta , Eigen::Vector3d{1.0, 0.0, 0.0} );

    Eigen::Vector3d point_in{0.0, 0.0, 1.0};

    Eigen::Vector3d point_out = q.rotate( point_in );

    EXPECT_NEAR( point_out.x(), 0.0, EPS );
    EXPECT_NEAR( point_out.y(), -0.5, EPS );
    EXPECT_NEAR( point_out.z(), HALF_ROOT_3, EPS );

}

TEST( TestQuaternion, testRotationAboutYAxisOfXPoint ) {
    using namespace phd;

    double theta = M_PI / 6.0;

    Quaternion q(  theta , Eigen::Vector3d{0.0, 1.0, 0.0} );

    Eigen::Vector3d point_in{1.0, 0.0, 0.0};

    Eigen::Vector3d point_out = q.rotate( point_in );

    EXPECT_NEAR( point_out.x(), HALF_ROOT_3, EPS );
    EXPECT_NEAR( point_out.y(), 0.0, EPS );
    EXPECT_NEAR( point_out.z(), -0.5, EPS );

}

TEST( TestQuaternion, testRotationAboutYAxisOfYPoint ) {
    using namespace phd;

    double theta = M_PI / 6.0;

    Quaternion q(  theta , Eigen::Vector3d{0.0, 1.0, 0.0} );

    Eigen::Vector3d point_in{0.0, 1.0, 0.0};

    Eigen::Vector3d point_out = q.rotate( point_in );

    EXPECT_NEAR( point_out.x(), 0.0, EPS );
    EXPECT_NEAR( point_out.y(), 1.0, EPS );
    EXPECT_NEAR( point_out.z(), 0.0, EPS );

}


TEST( TestQuaternion, testRotationAboutYAxisOfZPoint ) {
    using namespace phd;

    double theta = M_PI / 6.0;

    Quaternion q(  theta , Eigen::Vector3d{0.0, 1.0, 0.0} );

    Eigen::Vector3d point_in{0.0, 0.0, 1.0};

    Eigen::Vector3d point_out = q.rotate( point_in );

    EXPECT_NEAR( point_out.x(), 0.5, EPS );
    EXPECT_NEAR( point_out.y(), 0.0, EPS );
    EXPECT_NEAR( point_out.z(), HALF_ROOT_3, EPS );

}

TEST( TestQuaternion, testRotationAboutZAxisOfXPoint ) {
    using namespace phd;

    double theta = M_PI / 6.0;

    Quaternion q(  theta , Eigen::Vector3d{0.0, 0.0, 1.0} );

    Eigen::Vector3d point_in{1.0, 0.0, 0.0};

    Eigen::Vector3d point_out = q.rotate( point_in );

    EXPECT_NEAR( point_out.x(), HALF_ROOT_3, EPS );
    EXPECT_NEAR( point_out.y(), 0.5, EPS );
    EXPECT_NEAR( point_out.z(), 0.0, EPS );

}

TEST( TestQuaternion, testRotationAboutZAxisOfYPoint ) {
    using namespace phd;

    double theta = M_PI / 6.0;

    Quaternion q(  theta , Eigen::Vector3d{0.0, 0.0, 1.0} );

    Eigen::Vector3d point_in{0.0, 1.0, 0.0};

    Eigen::Vector3d point_out = q.rotate( point_in );

    EXPECT_NEAR( point_out.x(), -0.5, EPS );
    EXPECT_NEAR( point_out.y(), HALF_ROOT_3, EPS );
    EXPECT_NEAR( point_out.z(), 0.0, EPS );

}


TEST( TestQuaternion, testRotationAboutZAxisOfZPoint ) {
    using namespace phd;

    double theta = M_PI / 6.0;

    Quaternion q(  theta , Eigen::Vector3d{0.0, 0.0, 1.0} );

    Eigen::Vector3d point_in{0.0, 0.0, 1.0};

    Eigen::Vector3d point_out = q.rotate( point_in );

    EXPECT_NEAR( point_out.x(), 0.0, EPS );
    EXPECT_NEAR( point_out.y(), 0.0, EPS );
    EXPECT_NEAR( point_out.z(), 1.0, EPS );

}

TEST( TestQuaternion, testAxisAngle ) {
    using namespace phd;

    double theta = M_PI / 6.0;

    Quaternion q(  theta , Eigen::Vector3d{0.0, 0.0, 1.0} );

    double angle;
    Eigen::Vector3d axis;
    q.getAxisAngle(angle, axis);

    EXPECT_NEAR( angle, M_PI / 6.0, EPS );
    EXPECT_NEAR( axis[0], 0.0, EPS);
    EXPECT_NEAR( axis[1], 0.0, EPS);
    EXPECT_NEAR( axis[2], 1.0, EPS);
}






