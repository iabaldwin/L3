#ifndef HEADER_POSE_H_
#define HEADER_POSE_H_

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <list>
#include <iterator>
#include <assert.h>
#include <Eigen/Core>

namespace common
{
namespace estimation
{

class Pose 
{
    public:

        virtual ~Pose(){}

    protected:
        
        virtual void updateHomogeneous()  = 0;
        virtual void updateEuler()  = 0;
};

class SE2 : public Pose
{
    public:

        SE2( double X, double Y, double Q );
        SE2( const SE2& pose );
        //SE2& operator=( const SE2& pose );

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

        double& Q()
        {
            return q;
        }

        void Q( double q )
        {
            q = q;
        }

        Eigen::Matrix3f& getHomogeneous(){return homogeneous;}

    private:

        Eigen::Matrix3f homogeneous;

        friend SE2& operator*=( SE2& lhs, SE2& rhs );
        friend std::ostream& operator<<( std::ostream& stream, SE2& pose );
        
        double x,y,q;

        void updateHomogeneous();
        void updateEuler();

};

/*
 *SE3
 */
class SE3 : public Pose
{

    public:

        static SE3 ZERO();
        static SE3 UNIT_X();
        static SE3 UNIT_Y();
        static SE3 UNIT_Z();

        SE3();
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

        Eigen::Matrix4f& getHomogeneous(){return homogeneous;}

    private:
        Eigen::Matrix4f homogeneous;
 
        double x,y,z;
        double r,p,q;
   
        void updateHomogeneous();
        void updateEuler();
        
};

SE2 SE3ToSE2( SE3& pose  );
SE3 SE2ToSE3( SE2& pose  );

bool operator==( const common::estimation::SE3& lhs,  const common::estimation::SE3& rhs );

/*
 *  Intrinsic math
 */
namespace Math
{
    double norm( const common::estimation::SE3& a, const common::estimation::SE3& b );
    double SE2Metric( const common::estimation::SE3& a, const common::estimation::SE3& b );

} // math

} // estimation
} // common
#endif
