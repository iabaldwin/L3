#ifndef L3_DATATYPES_H
#define L3_DATATYPES_H

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <assert.h>

#include <Eigen/core>

namespace L3
{

struct Base 
{

    friend std::ostream& operator<<( std::ostream& o, const Base& b )
    {
        b.print( o );
        return o;
    }
        
    virtual void print( std::ostream& o ) const = 0;

};

class Pose : public Base
{
    public:
        
        Pose()
        {
        }
       
        virtual ~Pose(){}

        double x, y, time;

        Eigen::Matrix4f& getHomogeneous(){ return homogeneous; };

    protected:

        virtual void updateHomogeneous() = 0;

        std::stringstream ss;
        Eigen::Matrix4f homogeneous;

};

class SE2 : public Pose
{

    public:

        const static int NUM_ELEMENTS = 4;

        SE2( double X, double Y, double Q ) : q(Q)
        {
            x = X;
            y = Y;
        }
        
        void print( std::ostream& o ) const {
            o << x << "," << y << ","  << q;
        }
      
        double q;

    private:

        void updateHomogeneous() 
        {

        }
};

class SE3 : public Pose
{

    public:
    
        const static int NUM_ELEMENTS = 7;

        SE3( double X, double Y, double Z, double R, double P, double Q) 
            : z(Z), r(R), p(P), q(Q)
        {
            x = X;
            y = Y;

            updateHomogeneous();
        }

        SE3( const std::vector<double> v ) 
        {
            assert( v.size() == 7 );
            time = v[0]; 
            x = v[1];
            y = v[2];
            z = v[3];
            r = v[4];
            p = v[5];
            q = v[6];
            
            updateHomogeneous();
        }

        void print( std::ostream& o ) const {
            o << time << ":" << x << "," << y << ","  << z << "," << r << "," << p << "," << q;
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
            
            // E3
            homogeneous(0,3) = x;
            homogeneous(1,3) = y;
            homogeneous(2,3) = z;
        }

};

struct LIDAR : Base
{
    double time;
    unsigned int num_scans;
    std::vector<float> reflectances;
    std::vector<float> ranges;

};

struct LMS151 : LIDAR
{
    const static int NUM_ELEMENTS = 542;

    LMS151() 
    {
        ranges.resize( LMS151::NUM_ELEMENTS ); 
    }

    LMS151( std::vector<double> vec )  
    {
        time = vec[0];

        ranges.resize( LMS151::NUM_ELEMENTS ); 

        ranges.assign( ++vec.begin(), vec.end() );
    }

    void print( std::ostream& o ) const {
        o << ranges[0];
    }

    
};

}

#endif

