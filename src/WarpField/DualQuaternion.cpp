//
//  DualQuat.cpp
//  WarpField
//
//  Created by Dave on 2/03/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include "DualQuaternion.hpp"

namespace  phd {
    
    static const double EPS = 1e-6;
    
    /**
     * Make from a real and dual part both of which are Quaternions
     * Doesn't normalise
     */
    void DualQuaternion::construct( const Quaternion & realPart, const Quaternion & dualPart) {
        mRealPart=realPart;
        mDualPart=dualPart;
    }
    
    DualQuaternion::DualQuaternion( ) {
        construct(  Quaternion{ 1.0, 0.0, 0.0, 0.0}, Quaternion{ 0.0, 0.0, 0.0, 0.0} );
    }
    
    DualQuaternion::DualQuaternion( const Quaternion & r, const Quaternion & d ) {
        construct(  r, d );
    }
    
    /**
     * Construct from a rotation and a translation: gives unit DQ
     */
    DualQuaternion::DualQuaternion( double theta, const Eigen::Vector3d & axis, const Eigen::Vector3d & t ) {
        Quaternion r{theta, axis};
        DualQuaternion dqr{ r };
        DualQuaternion dqt{ t };
        DualQuaternion dqq = dqt * dqr;
        *this = dqq;
    }
    
    
    /**
     * Construct from a rotation gives unit DQ
     */
    DualQuaternion::DualQuaternion( const Quaternion & r ) {
        construct( r, Quaternion{ 0.0, 0.0, 0.0, 0.0 } );
    }
    
    /**
     * Construct from a rotation in angle axis form
     */
    DualQuaternion::DualQuaternion( double theta, const Eigen::Vector3d & axis ) {
        construct( Quaternion( theta, axis), Quaternion{ 0.0, 0.0, 0.0, 0.0 } );
    }
    
    /**
     * Construct from a translation gives unit DQ
     */
    DualQuaternion::DualQuaternion( const Eigen::Vector3d & t ) {
        DualQuaternion dqr{};
        DualQuaternion dqt{ Quaternion{1.0, 0.0, 0.0, 0.0}, Quaternion{ 0.0, t.x()/2.0, t.y()/2.0, t.z()/2.0 } };
        DualQuaternion dqq = dqt * dqr;
        
        //    double m = dqq * dqq.conjugate();
        *this = dqq;
    }
    
    /**
     * Construct from a rotation and a translation
     * q_d = 1/2 t r ( t is translation r is rotation)
     */
    DualQuaternion::DualQuaternion( const Quaternion & r, const Eigen::Vector3d & t ) {
        DualQuaternion dqr{ r };
        DualQuaternion dqt{ t };
        DualQuaternion dqq = dqt * dqr;
        *this = dqq;
    }
    
    /**
     * Simple scalar multiplication of parts
     */
    DualQuaternion & DualQuaternion::operator*=(double scalar) {
        mRealPart *= scalar;
        mDualPart *= scalar;
        return *this;
    }
    
    /**
     * Simple addition of parts
     */
    DualQuaternion & DualQuaternion::operator+=( const DualQuaternion & rhs ) {
        mRealPart += rhs.mRealPart;
        mDualPart += rhs.mDualPart;
        return *this;
    }
    
    
    DualQuaternion & DualQuaternion::operator*=( const DualQuaternion & rhs ) {
        Quaternion newRealPart = mRealPart * rhs.mRealPart;
        Quaternion newDualPart = (mRealPart * rhs.mDualPart) + (mDualPart*rhs.mRealPart);
        
        mRealPart = newRealPart;
        mDualPart = newDualPart;
        return *this;
    }
    

    //
    DualQuaternion DualQuaternion::unitized( ) const {
        DualQuaternion ret{*this};
        return ret.unitize();
    }
    
    
    DualQuaternion & DualQuaternion::unitize() {
        mRealPart.unitize();
        double m = mRealPart.magnitude();
        
        mRealPart *= ( 1.0 / m );
        mDualPart *= ( 1.0 / m );
        return *this;
    }
    
    /**
     * Conjugate 2
     * conj(dq) = conj(real) + conj(dual) E
     * Used for magnitude calculation
     */
    DualQuaternion DualQuaternion::conjugate2( ) const {
        DualQuaternion ret{ mRealPart.conjugate(), mDualPart.conjugate() };
        return ret;
    }
    
    /**
     * Conjugate 3
     * conj(dq) = conj(real) - conj(dual) E
     * Used for transformations
     */
    DualQuaternion DualQuaternion::conjugate3( ) const {
        DualQuaternion ret{ mRealPart.conjugate(), (-1 * mDualPart.conjugate() )};
        return ret;
    }
    
    double DualQuaternion::magnitude( ) const {
        return ( (*this) * this->conjugate2() ).mRealPart.w();
    }
    
    
    Quaternion DualQuaternion::getRotation() const {
        return mRealPart.unitized();
     //   return this->unitized().mRealPart;
    }
    
    Eigen::Vector3d DualQuaternion::getTranslation( ) const {
        
        Quaternion t = 2.0 * mDualPart* mRealPart.conjugate();
        
        return t.vec();
    }
    
    Eigen::Matrix4d DualQuaternion::toMatrix( ) const {
        DualQuaternion q{ this->unitized() };
        
        Eigen::Matrix4d M= Eigen::Matrix4d::Identity( );
        
        double w = q.mRealPart.w();
        double x = q.mRealPart.x();
        double y = q.mRealPart.y();
        double z = q.mRealPart.z();
        
        // Extract rotational information
        M(0,0) = w*w + x*x - y*y - z*z;
        M(0,1) = 2*x*y + 2*w*z;
        M(0,2) = 2*x*z - 2*w*y;
        M(1,0) = 2*x*y - 2*w*z;
        M(1,1) = w*w + y*y - x*x - z*z;
        M(1,2) = 2*y*z + 2*w*x;
        M(2,0) = 2*x*z + 2*w*y;
        M(2,1) = 2*y*z - 2*w*x;
        M(2,2) = w*w + z*z - x*x - y*y;
        
        // Extract translation information
        Quaternion t = mDualPart * 2.0 * mRealPart.conjugate();
        
        M(3,0) = t.x();
        M(3,1) = t.y();
        M(3,2) = t.z();
        return M;
    }
    
    
    Eigen::Vector3d DualQuaternion::transformPoint( const Eigen::Vector3d & point ) const {
        // Convert point to dual quat
        DualQuaternion pdq{ Quaternion{1,0,0,0}, Quaternion{0, point.x(), point.y(), point.z()} };
        DualQuaternion u=this->unitized();
        DualQuaternion result = (u) * pdq * u.conjugate3();
        return Eigen::Vector3d{ result.mDualPart.x(), result.mDualPart.y(), result.mDualPart.z() };
    }
    
    DualQuaternion operator+( const DualQuaternion & lhs, const DualQuaternion & rhs ) {
        DualQuaternion tmp{lhs};
        tmp += rhs;
        return tmp;
    }
    
    
    DualQuaternion operator*( const DualQuaternion & lhs, const DualQuaternion & rhs ) {
        DualQuaternion tmp{lhs};
        tmp *= rhs;
        return tmp;
    }
    
    DualQuaternion operator*( const DualQuaternion & lhs, double scalar ) {
        DualQuaternion tmp{lhs};
        tmp *= scalar;
        return tmp;
    }
    
    DualQuaternion operator*( double scalar, const DualQuaternion & rhs ) {
        return DualQuaternion{ rhs.mRealPart * scalar, rhs.mDualPart * scalar };
    }
    
    double DualQuaternion::dot( const DualQuaternion & otherDQ  ) const {
        return mRealPart.dot( otherDQ.mRealPart );
    }
    
    bool operator==( const DualQuaternion & lhs, const DualQuaternion & rhs ) {
        return ( ( lhs.mRealPart == rhs.mRealPart ) && ( lhs.mDualPart == rhs.mDualPart ) );
    }
    
    bool operator!=( const DualQuaternion & lhs, const DualQuaternion & rhs ) {
        return !( lhs == rhs );
    }
    
    std::ostream & operator<<( std::ostream & out,  DualQuaternion const & rhs ) {
        Quaternion rot = rhs.getRotation();
        Eigen::Vector3d tran = rhs.getTranslation();
        
        out << "Rotation " << rot << ", translation " << tran;
        return out;
    }
    
}
