#include "Datatypes.h"

namespace L3
{

namespace Math
{
    double norm( const L3::SE3& a, const L3::SE3& b )
    {
        return sqrt( pow(a.X() - b.X(), 2.0 ) + pow(a.Y() -b.Y(), 2.0 ) + pow(a.Z() -b.Z(), 2.0 ) );
    }
}


/*
 *  SE2
 */
SE2::SE2( double x, double y, double q ) : x(x), y(y), q(q)
{
}

SE2::SE2( std::vector<double> input )
{

}

void SE2::updateEuler() 
{

}


void SE2::updateHomogeneous() 
{

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

SE3::SE3( const std::vector<double> v ) 
{
    assert( v.size() == 6 );
    x = v[0];
    y = v[1];
    z = v[2];
    r = v[3];
    p = v[4];
    q = v[5];

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
    //L3::SE3 pose = poseFromRotation( const Eigen::Matrix4f& mat );
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

bool operator==( const L3::SE3& lhs,  const L3::SE3& rhs )
{
    return ( (lhs.X() == rhs.X()) &&
            (lhs.Y() == rhs.Y()) &&
            (lhs.Z() == rhs.Z()) &&
            (lhs.R() == rhs.R()) &&
            (lhs.P() == rhs.P()) &&
            (lhs.Q() == rhs.Q()) );

}





std::ostream& operator<<( std::ostream& o, const L3::LHLV& lhlv )
{
    std::copy( lhlv.data.begin(), 
                lhlv.data.end(),
                std::ostream_iterator<double>( o, " " ) );

    o << std::endl;

    return o;
}

std::ostream& operator<<( std::ostream& o, const L3::SE3& pose )
{
    o << pose.X() << " " << pose.Y() << " " << pose.Z() << " " 
        << pose.R() << " " << pose.P() << " " << pose.Q();

    return o;
}

}

