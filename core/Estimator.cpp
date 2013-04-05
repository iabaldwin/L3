#include "Estimator.h"
#include <boost/timer.hpp>

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

    if ( experience->num_points == 0 )
        return std::numeric_limits<T>::infinity();

    int granularity = 10;

    // Build experience histogram 
    this->experience_histogram->create( means.first, 
                                        min_bound.first, 
                                        max_bound.first,
                                        means.second, 
                                        min_bound.second, 
                                        max_bound.second,
                                        granularity );

    //(*this->experience_histogram)( experience );

}



}   // Estimator
}   // L3

// Explicit instantiations
template double L3::Estimator::KLCostFunction<double>::operator()(L3::Histogram<double>*, L3::Histogram<double>*, L3::SE3&, L3::SE3);
template double L3::Estimator::DiscreteEstimator<double>::operator()(L3::PointCloud<double>*, L3::PointCloud<double>*, L3::SE3);
