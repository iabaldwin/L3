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

        double x,y;

        float time;

    protected:

        std::stringstream ss;
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
        
        double z;
        double r,p,q;

};

struct LIDAR : Base
{
    float timestamp;
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
        timestamp = vec[0];

        ranges.resize( LMS151::NUM_ELEMENTS ); 

        ranges.assign( ++vec.begin(), vec.end() );
    }

    void print( std::ostream& o ) const {
        o << ranges[0];
    }

    
};



}

#endif

