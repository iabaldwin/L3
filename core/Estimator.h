#ifndef L3_ESTIMATOR_H
#define L3_ESTIMATOR_H

#include "Histogram.h"

namespace L3
{
namespace Estimator
{


/*
 *  Base cost function
 */
template <typename T>
struct CostFunction
{
    virtual double operator()( Histogram<T>* exp, Histogram<T>* swathe, L3::SE3& estimated_pose, L3::SE3 pose_guess ) = 0;

    virtual ~CostFunction()
    {

    }
};

template <typename T>
struct KLCostFunction : CostFunction<T>
{
    double operator()( Histogram<T>* exp, Histogram<T>* swathe, L3::SE3& estimated_pose, L3::SE3 pose_guess ) ;
};

/*
 *Estimator types
 */
template< typename T >
struct Estimator
{
    Estimator( CostFunction<T>* f ) : cost_function(f)
    {
    }

    std::auto_ptr< L3::Histogram<T> >   experience_histogram;
    L3::Histogram<T>                    swathe_histogram;
    CostFunction<T>*                    cost_function;

    virtual ~Estimator()
    {
    }

    virtual double operator()( PointCloud<T>* experience, PointCloud<T>* swathe, SE3 estimate ) = 0;

};


template< typename T >
struct DiscreteEstimator : Estimator<T>
{

    DiscreteEstimator( CostFunction<T>* f ) : Estimator<T>(f)
    {
    }
 
    double operator()( PointCloud<T>* experience, PointCloud<T>* swathe, SE3 estimate );

};

}
}

#endif
