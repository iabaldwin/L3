#ifndef L3_DATATYPES_H
#define L3_DATATYPES_H

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <assert.h>
#include <Eigen/core>

namespace L3
{

class Pose 
{
    public:

        virtual ~Pose(){}

        double x, y;

        Eigen::Matrix4f& getHomogeneous(){ return homogeneous; };

        //TODO
        //This is dirty
        virtual void _update() = 0;
        
        friend std::ostream& operator<<( std::ostream& o, const Pose& p )
        {
            p.print( o );
            return o;
        }

        virtual void print( std::ostream& o ) const = 0;

    protected:

        Eigen::Matrix4f homogeneous;
        virtual void updateHomogeneous()  = 0;


};

class SE2 : public Pose
{

    public:

        SE2( double X, double Y, double Q ) : q(Q)
        {
            x = X;
            y = Y;
        }

        SE2( std::vector<double> input )
        {

        }

        void print( std::ostream& o ) const {
            o << x << "," << y << ","  << q;
        }
      
        double q;

        void _update()
        {

        }

    private:

        void updateHomogeneous() 
        {

        }
};

class SE3 : public Pose
{

    public:
    
        SE3( double X, double Y, double Z, double R, double P, double Q) 
            : z(Z), r(R), p(P), q(Q)
        {
            x = X;
            y = Y;

            updateHomogeneous();
        }

        SE3( const std::vector<double> v ) 
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

        void print( std::ostream& o ) const {
            o << x << "," << y << ","  << z << "," << r << "," << p << "," << q;
        }
      
        void _update()
        {
            updateHomogeneous();
        }

        double z;
        double r,p,q;

        
    private:
    
        void updateHomogeneous() 
        {
            Eigen::Matrix4f Rx = Eigen::Matrix4f::Identity();
            Rx(1,1) = cos( r );
            Rx(1,2) = -1*sin( r );
            Rx(2,1) = sin( r );
            Rx(2,2) = cos( r );

            Eigen::Matrix4f Ry = Eigen::Matrix4f::Identity();
            Ry(0,0) = cos( p );
            Ry(0,2) = sin( p );
            Ry(2,0) = -1*sin( p );
            Ry(2,2) = cos( p );


            Eigen::Matrix4f Rz = Eigen::Matrix4f::Identity();
            Rz(0,0) = cos( q );
            Rz(0,1) = -1*sin( q );
            Rz(1,0) = sin( q );
            Rz(1,1) = cos( q );

            // SO3
            homogeneous = Rz*Ry*Rx;
            
            // SE3
            homogeneous(0,3) = x;
            homogeneous(1,3) = y;
            homogeneous(2,3) = z;
        }

};

struct LHLV  
{
    typedef std::vector<double>::iterator ITERATOR;

    LHLV( const std::vector<double> v ) 
    {
        data.assign( v.begin(), v.end() );
    }

    void print( std::ostream& o ) const {
        std::copy( data.begin(), 
                data.end(),
                std::ostream_iterator<double>( std::cout, " " ) );
    }

    std::vector< double > data;
};

struct LIDAR 
{
    std::vector<float> ranges;
    std::vector<float> reflectances;

    LIDAR( double start, double end ) : 
        angle_start(start), angle_end(end)
    {}

    double angle_start;
    double angle_end;
    double angle_spacing;

    int num_scans;
};

struct LMS151 : LIDAR
{
    LMS151() : LIDAR( -1*M_PI/4, M_PI+(M_PI/4))
    {
        num_scans = 541;
        
        angle_spacing = (angle_end - angle_start ) / (double)num_scans;
            
        ranges.resize( num_scans );
   
    }

    LMS151( std::vector<double> vec )  : LIDAR( -1*M_PI/4, M_PI+(M_PI/4)) 
    {
        num_scans = 541;
        
        angle_spacing = (angle_end - angle_start ) / (double)num_scans;

        ranges.resize( num_scans );
        ranges.assign( vec.begin(), vec.end() );
    }

    void print( std::ostream& o ) const {
      
        o << angle_start << ":" << angle_end << ":" << angle_spacing;
        o << std::endl;
        std::copy( ranges.begin(), ranges.end(), std::ostream_iterator<double>( o, " " ) );
    }
    
};

template <typename T>
struct Sizes
{
    const static int elements = 0;
};

template <>
struct Sizes<L3::SE3>
{
    const static int elements = 6+1;
};

template <>
struct Sizes<L3::LMS151>
{
    const static int elements = 541+1;
};

template <>
struct Sizes<L3::LHLV>
{
    const static int elements = 11+1;
};


}

#endif
