//
//  Quaternion.cpp
//  WarpField
//
//  Created by Dave on 3/03/2016.
//  Copyright © 2016 Sindesso. All rights reserved.
//

#include "Quaternion.hpp"


using namespace phd;

const double EPS = 1e-6;

#pragma mark - Constructors

static const Quaternion & IDENTITY{ 1.0, 0.0, 0.0, 0.0 };


void Quaternion::construct( double w, double x, double y, double z ) {
    mw = w;
    mx = x;
    my = y;
    mz = z;
}

Quaternion::Quaternion( ) {
    construct( 1.0, 0.0, 0.0, 0.0 );
}


Quaternion::Quaternion( double theta, Eigen::Vector3d axis ) {
    double s = sin( theta / 2.0 );
    double c = cos( theta / 2 );
    
    construct( c, s * axis(0), s * axis(1), s * axis(2) );
}

Quaternion::Quaternion( double w, double x, double y, double z ) {
    construct( w, x, y, z );
}


Quaternion & Quaternion::operator+=( const Quaternion & rhs) {
    mw += rhs.mw;
    mx += rhs.mx;
    my += rhs.my;
    mz += rhs.mz;
    
    // return the result by value (uses move constructor)
    return *this;
}

Quaternion & Quaternion::operator*=( double scalar) {
    mw *= scalar;
    mx *= scalar;
    my *= scalar;
    mz *= scalar;
    
    // return the result by value (uses move constructor)
    return *this;
}

Quaternion & Quaternion::operator*=( const Quaternion & rhs ) {
    double new_mw = mw*rhs.mw - mx*rhs.mx - my*rhs.my - mz*rhs.mz;
    double new_mx = mw*rhs.mx + mx*rhs.mw + my*rhs.mz - mz*rhs.my;
    double new_my = mw*rhs.my - mx*rhs.mz + my*rhs.mw + mz*rhs.mx;
    double new_mz = mw*rhs.mz + mx*rhs.my - my*rhs.mx + mz*rhs.mw;
    
    mw = new_mw;
    mx = new_mx;
    my = new_my;
    mz = new_mz;
    
    return *this;
}

double Quaternion::magnitude( ) const {
    return sqrt( mw*mw + mx*mx + my*my + mz*mz );
}


Eigen::Vector3d Quaternion::rotate( const Eigen::Vector3d & point ) const {
    // COnvert point to quat with 0 w
    Quaternion pq{0.0, point.x(), point.y(), point.z() };
    
    Quaternion pqout= (*this) * pq * this->conjugate();
    
    return Eigen::Vector3d{ pqout.x(), pqout.y(), pqout.z() };
}


Quaternion Quaternion::unitized( ) const{
    Quaternion u = *this;
    u.unitize();
    return u;
}

Quaternion Quaternion::conjugate( ) const {
    return Quaternion{ mw, -mx, -my, -mz };
}

double Quaternion::dot( const Quaternion & rhs ) const {
    return (mw * rhs.mw) + (mx * rhs.mx) + ( my * rhs.my ) + ( mz * rhs.mz );
}


Quaternion & Quaternion::unitize( ) {
    double dotSelf  = mw*mw + mx*mx + my*my + mz*mz;
    
    if( fabs(dotSelf - 1.0 ) > EPS ) {
        double length = std::sqrt( dotSelf );
        
        // Handle zero case
        if( fabs(length) < EPS ) {
            mw = 1.0;
            mx = my = mz = 0.0;
        } else {
            double oneOverLength = 1.0 / length;
            
            mw *= oneOverLength;
            mx *= oneOverLength;
            my *= oneOverLength;
            mz *= oneOverLength;
        }
    }
    
    return *this;
}

void Quaternion::getAxisAngle( double &angle, Eigen::Vector3d & axis) const {
    Quaternion tmp{*this};
    
    tmp.unitize();
    double theta = 2 * acos( tmp.w() );
    angle = theta;
    
    if ( theta == 0 ) {
        axis = Eigen::Vector3d{ 1.0, 0.0, 0.0 };
    } else {
        double s = sqrt( 1 - mw*mw );
        axis = { tmp.x() / s, tmp.y() / s, tmp.z() / s };
    }
}



namespace phd {
    Quaternion operator+(const Quaternion & lhs, const Quaternion & rhs) {
        Quaternion tmp{lhs};
        tmp += rhs;
        return tmp;
    }
    
    Quaternion operator*(const Quaternion & lhs, double scalar) {
        Quaternion tmp{lhs};
        tmp *= scalar;
        return tmp;
    }
    
    Quaternion operator*(double scalar, const Quaternion & rhs) {
        return Quaternion{
            rhs.mw * scalar,
            rhs.mx * scalar,
            rhs.my * scalar,
            rhs.mz * scalar };
    }
    
    Quaternion operator*(const Quaternion & lhs, const Quaternion & rhs ) {
        Quaternion tmp{lhs};
        tmp *= rhs;
        return tmp;
    }
    
    
    bool operator==( const Quaternion & lhs, const Quaternion & rhs ) {
        return ( (fabs(lhs.mw - rhs.mw) < EPS) &&
                (fabs(lhs.mx - rhs.mx) < EPS) &&
                (fabs(lhs.my - rhs.my) < EPS) &&
                (fabs(lhs.mz - rhs.mz) < EPS));
    }
    
    bool operator!=( const Quaternion & lhs, const Quaternion & rhs ) {
        return ! ( lhs == rhs );
    }
    
    Eigen::Vector3d operator*(const Quaternion & lhs, const Eigen::Vector3d & rhs) {
        Quaternion p_in{ 0.0, rhs.x(), rhs.y(), rhs.z() };
        Quaternion p_out= lhs * p_in;
        return Eigen::Vector3d{ p_out.x(), p_out.y(), p_out.z() };
    }
    
    Eigen::Vector3d operator*(const Eigen::Vector3d & lhs, const Quaternion & rhs ) {
        Quaternion p_in{ 0.0, lhs.x(), lhs.y(), lhs.z() };
        Quaternion p_out= p_in * rhs;
        return Eigen::Vector3d{ p_out.x(), p_out.y(), p_out.z() };
    }
    
    std::ostream & operator<<( std::ostream & out,  Quaternion const & rhs ) {
        double angle;
        Eigen::Vector3d axis;
        rhs.getAxisAngle(angle, axis);
        angle = angle * (180/M_PI);
        out << "  : Angle " << angle << ", axis " << axis << "   [ " << rhs.mw << ", " << rhs.mx << ", " << rhs.my << ", " << rhs.mz << " ]" ;
        return out;
    }
}
