#include "Estimator.h"

#include <boost/random.hpp>

struct Point
{
    Point( double x, double y ) : x(x), y(y)
    {

    }

    double x,y;

};

int main()
{
    boost::mt19937 rng;
    boost::normal_distribution<> normal(0.0, 10.0);

    boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor(rng, normal );

    std::vector< Point > points(10000, Point(0,0));
    std::vector< Point >::iterator it = points.begin();

    while( it != points.end() )
        *it++ = Point( var_nor(), var_nor() );


}
