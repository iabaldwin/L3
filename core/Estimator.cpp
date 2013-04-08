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
    
double DiscreteEstimator<T>::operator()( PointCloud<T>* swathe, SE3 estimate ) 
{
    L3::ReadLock( this->experience_histogram->mutex );
    L3::WriteLock( this->swathe_histogram->mutex );
    this->swathe_histogram->copy( *this->experience_histogram );
    (*this->swathe_histogram)( swathe );
}



}   // Estimator
}   // L3

// Explicit instantiations
template double L3::Estimator::KLCostFunction<double>::operator()(L3::Histogram<double>*, L3::Histogram<double>*, L3::SE3&, L3::SE3);
template double L3::Estimator::DiscreteEstimator<double>::operator()( PointCloud<double>* swathe, SE3 estimate );
