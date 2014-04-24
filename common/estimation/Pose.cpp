#include "Pose.h"

namespace common
{
namespace estimation
{

namespace Math
{
    double norm( const common::estimation::SE3& a, const common::estimation::SE3& b )
    {
        return sqrt( pow(a.X() - b.X(), 2.0 ) + pow(a.Y() -b.Y(), 2.0 ) + pow(a.Z() -b.Z(), 2.0 ) );
    }

    double SE2Metric( const common::estimation::SE3& a, const common::estimation::SE3& b )
    {
        return sqrt( pow(a.X() - b.X(), 2.0 ) + pow(a.Y() -b.Y(), 2.0 ) );
    }

} // Math

/*
 *  SE2
 */
SE2 poseFromHomogeneous( const Eigen::Matrix3f& mat )
{
    double x = mat(0, 2 );
    double y = mat(1, 2 );

    double q = atan2( mat(1,0), mat( 0, 0 ) );

    return SE2( x, y, q );
}

SE3 poseFromHomogeneous( const Eigen::Matrix4f& mat )
{
    double x = mat(0, 3 );
    double y = mat(1, 3 );
    double z = mat(2, 3 );

    double q = atan2( mat(1,0), mat( 0, 0 ) );
    double r = atan2( -1* mat(2, 0), sqrt( pow( mat(2,1),2 )+ pow( mat(2,2) ,2) ) );
    double p = atan2( mat(2,1), mat(2,2) );

    return SE3( x,y,z,r,p,q );
}


SE2::SE2( double x, double y, double q ) : x(x), y(y), q(q)
{
    updateHomogeneous();
}
        
SE2::SE2( const SE2& pose )
{
    this->x = pose.x; 
    this->y = pose.y; 
    this->q = pose.q; 
    this->homogeneous = pose.homogeneous; 

    updateEuler();
}

//SE2& SE2::operator=( const SE2& pose )
//{
    //this->x = pose.x; 
    //this->y = pose.y; 
    //this->q = pose.q; 
    //this->homogeneous = pose.homogeneous; 

    //updateEuler();

    //return *this;
//}


void SE2::updateEuler() 
{
    poseFromHomogeneous( this->homogeneous );
}

void SE2::updateHomogeneous() 
{
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
    R(0,0) = cos( q );
    R(0,1) = -1*sin( q );
    R(1,0) = sin( q );
    R(1,1) = cos( q );

    homogeneous = R;
    homogeneous(0,2) = x;
    homogeneous(1,2) = y;

}

SE2& operator*=( SE2& lhs,  SE2& rhs )
{
    Eigen::Matrix3f mat = lhs.homogeneous;
    mat *= rhs.homogeneous; 
    lhs = poseFromHomogeneous( mat );
    return lhs;
}

std::ostream& operator<<( std::ostream& stream, SE2& pose )
{
    stream << pose.X() << ":" << pose.Y() << ":" << pose.Q();
    return stream;
}

/*
 *  SE3
 */
SE3 SE3::ZERO()
{
    return SE3(0,0,0,0,0,0);
}

SE3 SE3::UNIT_X()
{
    return SE3(1,0,0,0,0,0);
}

SE3 SE3::UNIT_Y()
{
    return SE3(0,1,0,0,0,0);
}

SE3 SE3::UNIT_Z()
{
    return SE3(0,0,1,0,0,0);
}

SE3::SE3()
{
    x = y = z = r = p = q = 0;        
}

SE3::SE3( double x, double y, double z, double r, double p, double q)
    : x(x), y(y), z(z), 
        r(r), p(p), q(q)
{
    updateHomogeneous();
}

double SE3::X() const
{
    return x;
}

void SE3::X( double x )
{
    this->x = x;
    updateHomogeneous();
}

double SE3::Y() const
{
    return y;
}

void SE3::Y( double y )
{
    this->y = y;
    updateHomogeneous();
}

double SE3::Z() const
{
    return z;
}

void SE3::Z( double z )
{
    this->z = z;
    updateHomogeneous();
}

double SE3::R() const
{
    return r;
}

void SE3::R( double r )
{
    this->r = r;
    updateHomogeneous();
}

double SE3::P() const
{
    return p;
}

void SE3::P( double p )
{
    this->p = p;
    updateHomogeneous();
}

double SE3::Q() const
{
    return q;
}

void SE3::Q( double q )
{
    this->q = q;
    updateHomogeneous();
}

void SE3::updateEuler() 
{
    //common::estimation::SE3 pose = poseFromHomogeneous( const Eigen::Matrix4f& mat );
}

void SE3::updateHomogeneous() 
{
    Eigen::Matrix4f Rz = Eigen::Matrix4f::Identity();
    Rz(0,0) = cos( q );
    Rz(0,1) = -1*sin( q );
    Rz(1,0) = sin( q );
    Rz(1,1) = cos( q );

    Eigen::Matrix4f Rx = Eigen::Matrix4f::Identity();
    Rx(1,1) = cos( p );
    Rx(1,2) = -1*sin( p );
    Rx(2,1) = sin( p );
    Rx(2,2) = cos( p );

    Eigen::Matrix4f Ry = Eigen::Matrix4f::Identity();
    Ry(0,0) = cos( r );
    Ry(0,2) = sin( r );
    Ry(2,0) = -1*sin( r );
    Ry(2,2) = cos( r );

    // SO3
    homogeneous = Rz*Ry*Rx;

    // SE3
    homogeneous(0,3) = x;
    homogeneous(1,3) = y;
    homogeneous(2,3) = z;
}

bool operator==( const common::estimation::SE3& lhs,  const common::estimation::SE3& rhs )
{
    return ( (lhs.X() == rhs.X()) &&
            (lhs.Y() == rhs.Y()) &&
            (lhs.Z() == rhs.Z()) &&
            (lhs.R() == rhs.R()) &&
            (lhs.P() == rhs.P()) &&
            (lhs.Q() == rhs.Q()) );

}

std::ostream& operator<<( std::ostream& o, const common::estimation::SE3& pose )
{
    o << pose.X() << " " << pose.Y() << " " << pose.Z() << " " 
        << pose.R() << " " << pose.P() << " " << pose.Q();

    return o;
}

SE2 SE3ToSE2( SE3& pose )
{
    Eigen::Matrix3f mat = Eigen::Matrix3f::Identity();

    mat.block<2,2>(0,0) = pose.getHomogeneous().block<2,2>(0,0);

    mat( 0, 2 ) = pose.X();
    mat( 1, 2 ) = pose.Y();

    return poseFromHomogeneous( mat );
}

SE3 SE2ToSE3( SE2& pose )
{
    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();

    mat.block<2,2>(0,0) = pose.getHomogeneous().block<2,2>(0,0);

    mat( 0, 3 ) = pose.X();
    mat( 1, 3 ) = pose.Y();

    return poseFromHomogeneous( mat );
}



}
}

