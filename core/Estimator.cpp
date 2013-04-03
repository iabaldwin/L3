#include "Estimator.h"

namespace L3
{
namespace Estimator
{

/*
 *KL Cost function
 */
template <typename T>
double KLCostFunction<T>::operator()( Histogram<T>* exp, Histogram<T>* swathe, L3::SE3& estimated_pose, L3::SE3 pose_guess ) 
{
}


/*
 *Discrete Estimator
 */
template <typename T>
double DiscreteEstimator<T>::operator()( PointCloud<T>* experience, PointCloud<T>* swathe, SE3 estimate )
{

    std::pair<T,T> min_bound = L3::min<T>( experience );
    std::pair<T,T> max_bound = L3::max<T>( experience );
    std::pair<T,T> means     = L3::mean( experience );

    int granularity = 100;

    // Build experience histogram 
    L3::Histogram<T> histogram( means.first, 
                                        means.first - min_bound.first, 
                                        max_bound.first - means.first, 
                                        means.second, 
                                        means.second - min_bound.second, 
                                        max_bound.second - means.second, 
                                        granularity );

    histogram( experience );

    // Restart
    histogram.reset();

    histogram( swathe );

}



}   // Estimator
}   // L3

// Explicit instantiations
template double L3::Estimator::KLCostFunction<double>::operator()(L3::Histogram<double>*, L3::Histogram<double>*, L3::SE3&, L3::SE3);
template double L3::Estimator::DiscreteEstimator<double>::operator()(L3::PointCloud<double>*, L3::PointCloud<double>*, L3::SE3);
