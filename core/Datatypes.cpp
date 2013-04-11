#include "Datatypes.h"

namespace L3
{
namespace Math
{
    double norm( const L3::SE3& a, const L3::SE3& b )
    {
        return sqrt( pow(a.x - b.x, 2.0 ) + pow(a.y -b.y, 2.0 ) + pow(a.z -b.z, 2.0 ) );
    }
}


/*
 *  SE2
 */
SE2::SE2( double X, double Y, double Q ) : q(Q)
{
    x = X;
    y = Y;
}

SE2::SE2( std::vector<double> input )
{

}

void SE2::_update()
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

SE3::SE3( double X, double Y, double Z, double R, double P, double Q)
    : z(Z), r(R), p(P), q(Q)
{
    x = X;
    y = Y;

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

void SE3::_update()
{
    updateHomogeneous();
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




std::ostream& operator<<( std::ostream& o, const L3::LHLV& lhlv )
{
    std::copy( lhlv.data.begin(), 
                lhlv.data.end(),
                std::ostream_iterator<double>( o, " " ) );

    o << std::endl;
}

std::ostream& operator<<( std::ostream& o, const L3::SE3& pose )
{
    o << pose.x << " " << pose.y << " " << pose.z << " " 
        << pose.r << " " << pose.p << " " << pose.q;

    return o;
}

}

