#ifndef L3_DATATYPES_H
#define L3_DATATYPES_H

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <assert.h>


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
        
        Pose(){}
        virtual ~Pose(){}

    protected:

        std::stringstream ss;
};

class SE2 : public Pose
{

    public:
    
        const static int NUM_ELEMENTS = 4;

        SE2( double X, double Y, double Q ) : x(X), y(Y), q(Q)
        {
        }
        
        void print( std::ostream& o ) const {
            o << x << "," << y << ","  << q;
        }
        
        double x;
        double y;
        double q;

};

class SE3 : public Pose
{

    public:
    
        const static int NUM_ELEMENTS = 7;

        SE3( double X, double Y, double Z, double R, double P, double Q) 
            : x(X), y(Y), z(Z), r(R), p(P), q(Q)
        {
        }

        SE3( const std::vector<double> v ) 
        {
            assert( v.size() == 7 );
            x = v[1];
            y = v[2];
            z = v[3];
            r = v[4];
            p = v[5];
            q = v[6];
        }

        void print( std::ostream& o ) const {
            o << x << "," << y << ","  << q;
        }

        double x,y,z;
        double r,p,q;

};

struct LIDAR
{
    const static int NUM_ELEMENTS = 542;

    uint64_t timestamp;
    unsigned int num_scans;
    std::vector<float> reflectances;
    std::vector<float> ranges;

};

struct LMS151 : LIDAR
{

    LMS151() 
    {
        //reflectances.reserve( num_scans ); 
        //ranges.reserve( num_scans ); 
    }

    LMS151( std::vector<double> vec )  
    {
        //reflectances.reserve( num_scans ); 
        //ranges.reserve( num_scans ); 
    }


};
}

#endif

