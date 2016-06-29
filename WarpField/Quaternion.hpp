//
//  Quaternion.hpp
//  WarpField
//
//  Created by Dave on 3/03/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#ifndef Quaternion_hpp
#define Quaternion_hpp

#include "Eigen/Dense"
#include <stdio.h>

namespace phd {
    class Quaternion {
    private:
        double  mw;
        double  mx;
        double  my;
        double  mz;
        
        void construct( double w, double x, double y, double z );
    public:
        // Default constructor
        Quaternion();
        
        // Constuct with 4 elements
        Quaternion( double w, double x, double y, double z );
        
        // Construct from angle and axis
        Quaternion( double theta, Eigen::Vector3d axis );
        
        inline double x() const { return mx; }
        inline double y() const { return my; }
        inline double z() const { return mz; }
        inline double w() const { return mw; }
        inline Eigen::Vector3d vec() const { return Eigen::Vector3d{ mx, my, mz}; }
        
        Quaternion & operator+=( const Quaternion & rhs);
        Quaternion & operator*=( const Quaternion & rhs );
        Quaternion & operator*=( double scalar);
        
        double magnitude( ) const;
        Quaternion & unitize( );
        Quaternion unitized( ) const;
        
        Quaternion conjugate( ) const;
        
        double dot( const Quaternion & rhs ) const;
        
        Eigen::Vector3d rotate( const Eigen::Vector3d & point ) const;
        
        
        void getAxisAngle( double &angle, Eigen::Vector3d & axis) const;
        
        friend std::ostream & operator<<( std::ostream & out, Quaternion const & rhs );
        friend Quaternion operator*(double scalar, const Quaternion & rhs);
        friend bool operator==( const Quaternion & lhs, const Quaternion & rhs );
    };

    // Binary Operators
    bool operator!=( const Quaternion & lhs, const Quaternion & rhs );
    
    Quaternion operator+(const Quaternion & lhs, const Quaternion & rhs);
    Quaternion operator*(const Quaternion & lhs, const Quaternion & rhs );
    Quaternion operator*(const Quaternion & lhs, double scalar);
    
    Eigen::Vector3d operator*(const Quaternion & lhs, const Eigen::Vector3d & rhs);
    Eigen::Vector3d operator*(const Eigen::Vector3d & lhs, const Quaternion & rhs );
}
#endif /* Quaternion_hpp */
