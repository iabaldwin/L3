#ifndef L3_DATATYPES_H
#define L3_DATATYPES_H

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <assert.h>
#include <Eigen/Core>

namespace L3
{

class Pose 
{
    public:

        virtual ~Pose(){}

        Eigen::Matrix4f& getHomogeneous(){ return homogeneous; };
        void setHomogeneous( Eigen::Matrix4f homogeneous ){ this->homogeneous = homogeneous; };

    protected:
        
        virtual void updateHomogeneous()  = 0;
        virtual void updateEuler()  = 0;

        Eigen::Matrix4f homogeneous;


};

class SE2 : public Pose
{

    public:

        SE2( double X, double Y, double Q );
        SE2( std::vector<double> input );

        double& X()
        {
            return x;
        }

        void X( double x )
        {
            x = x;
        }

        double& Y()
        {
            return y;
        }

        void Y( double y )
        {
            y = y;
        }


    private:
        
        double x,y,q;

        void updateHomogeneous();
        void updateEuler();

};

class SE3 : public Pose
{

    public:

        static SE3 ZERO();
        static SE3 UNIT_X();
        static SE3 UNIT_Y();
        static SE3 UNIT_Z();

        SE3( double X, double Y, double Z, double R, double P, double Q);
        SE3( const std::vector<double> v );
        
        double  X() const;
        void    X( double x );
        double  Y() const;
        void    Y( double y );
        double  Z() const;
        void    Z( double z );
        double  R() const;
        void    R( double r );
        double  P() const;
        void    P( double P );
        double  Q() const;
        void    Q( double q );


    private:
 
        double x,y,z;
        double r,p,q;
   
        void updateHomogeneous();
        void updateEuler();
        
};

struct LHLV  
{
    typedef std::vector<double>::iterator ITERATOR;

    LHLV( const std::vector<double> v ) 
    {
        data.assign( v.begin(), v.end() );
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
            
        ranges.resize( num_scans, 0 );
   
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

/*
 *  I/O
 */
std::ostream& operator<<( std::ostream& o, const L3::SE2& pose );
std::ostream& operator<<( std::ostream& o, const L3::SE3& pose );
std::ostream& operator<<( std::ostream& o, const L3::LIDAR& scan );
std::ostream& operator<<( std::ostream& o, const L3::LMS151& scan );
std::ostream& operator<<( std::ostream& o, const L3::LHLV& lhlv );

/*
 *  Intrinsic math
 */
namespace Math
{
    double norm( const L3::SE3& a, const L3::SE3& b );

} //Math
} //L3

#endif
