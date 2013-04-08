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
    virtual double operator()( const Histogram<T>& experience, const Histogram<T>& swathe, const L3::SE3& estimated_pose ) = 0;

    virtual ~CostFunction()
    {

    }
};

template <typename T>
struct KLCostFunction : CostFunction<T>
{
    double operator()( const Histogram<T>& experience, const Histogram<T>& swathe, const L3::SE3& estimated_pose );
};

/*
 *Estimator types
 */
template< typename T >
struct Estimator
{
    Estimator( CostFunction<T>* f, boost::shared_ptr<L3::Histogram<double> > experience ) : cost_function(f), experience_histogram(experience)
    {
        swathe_histogram.reset( new L3::HistogramUniformDistance<double>() );
    }

    CostFunction<T>*                        cost_function;
    boost::shared_ptr<L3::Histogram<T> >    experience_histogram;
    boost::shared_ptr<L3::Histogram<T> >    swathe_histogram;

    virtual ~Estimator()
    {
    }

    virtual double operator()( PointCloud<T>* swathe, SE3 estimate ) = 0;

};


template< typename T >
struct DiscreteEstimator : Estimator<T>
{

    DiscreteEstimator( CostFunction<T>* f, boost::shared_ptr<L3::Histogram<double> > experience ) : Estimator<T>(f, experience)
    {
    }
 
    double operator()( PointCloud<T>* swathe, SE3 estimate );

};

}
}

#endif
