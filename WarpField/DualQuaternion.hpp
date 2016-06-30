//
//  DualQuat.hpp
//  WarpField
//
//  Created by Dave on 2/03/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#ifndef DualQuaternion_hpp
#define DualQuaternion_hpp

#include <stdio.h>

#include <Eigen/Dense>
#include "Quaternion.hpp"

namespace phd {
    class DualQuaternion {
    private:
        Quaternion mRealPart;
        Quaternion mDualPart;

        void construct( const Quaternion & realPart, const Quaternion & dualPart);

    public:

#pragma mark - Construcors
        /**
         * Default constructor makes identity transformation
         */
        DualQuaternion();

        /**
         * Construct from two quaternions
         */
        DualQuaternion( const Quaternion & r, const Quaternion & d);

        /**
         * Construct from a rotation and a translation
         */
        DualQuaternion( double theta, const Eigen::Vector3d & axis, const Eigen::Vector3d & t );

        /**
         * Construct from a rotation and a translation
         * q_d = 1/2 t r ( t is translation r is rotation)
         */
        DualQuaternion( const Quaternion & r, const Eigen::Vector3d & t );

        /**
         * Construct from a rotation
         */
        DualQuaternion( const Quaternion & r );

        /**
         * Construct from a rotation in angle axis form
         */
        DualQuaternion( double theta, const Eigen::Vector3d & axis );


        /**
         * Construct from a translation
         */
        DualQuaternion( const Eigen::Vector3d & t );

#pragma mark - Operator Overloads
        // Returning non-const refs to allow (a += b) += c
        DualQuaternion & operator*=( const double scalar);
        DualQuaternion & operator+=( const DualQuaternion & rhs );
        DualQuaternion & operator*=( const DualQuaternion & rhs );
        DualQuaternion & unitize( );

        double          dot( const DualQuaternion & rhs  )  const;
        DualQuaternion  unitized( )                         const;
        DualQuaternion  conjugate2( )                       const;
        DualQuaternion  conjugate3( )                       const;
        double          magnitude( )                        const;

#pragma mark - Geometry
        // Extract geometric meanings
        Quaternion      getRotation()                       const;
        Eigen::Vector3d getTranslation( )                   const;
        Eigen::Matrix4d toMatrix( )                         const;


#pragma mark - Friend functions
        // Returning non-const refs to allow (a += b) += c
        friend DualQuaternion   operator*( double scalar, const DualQuaternion & rhs );
        friend bool             operator==( const DualQuaternion & lhs, const DualQuaternion & rhs );
        friend std::ostream &   operator<<( std::ostream & out,  const DualQuaternion & rhs );

        // Geometry
        Eigen::Vector3d transformPoint( const Eigen::Vector3d & point ) const;

    };

    DualQuaternion   operator*( const DualQuaternion & lhs, const DualQuaternion & rhs );
    DualQuaternion   operator*( const DualQuaternion & lhs, double scalar);
    DualQuaternion   operator+( const DualQuaternion & lhs, const DualQuaternion & rhs );

    bool             operator!=( const DualQuaternion & lhs, const DualQuaternion & rhs );
}

#endif /* DualQuaternion_hpp */
