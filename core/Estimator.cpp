#include "Estimator.h"
#include <iterator>

namespace L3
{
namespace Estimator
{

/*
 *KL Cost function
 */
template <typename T>
double KLCostFunction<T>::operator()( const Histogram<T>& experience, const Histogram<T>& swathe, const L3::SE3& estimated_pose )
{
   
    std::copy( experience.hist->bin, 
                experience.hist->bin + (experience.x_bins * experience.y_bins), 
                std::ostream_iterator<T>( std::cout, " " ) );
        
    std::cout << std::endl;

    return std::numeric_limits<T>::infinity();
}


/*
 *Discrete Estimator
 */
template <typename T>
double DiscreteEstimator<T>::operator()( PointCloud<T>* swathe, SE3 estimate ) 
{
    L3::ReadLock( this->experience_histogram->mutex );
    this->swathe_histogram->copy( *this->experience_histogram );
    (*this->swathe_histogram)( swathe );

    std::cout << (*this->cost_function)( *this->experience_histogram, *this->swathe_histogram, estimate ) << std::endl; 
}



}   // Estimator
}   // L3

// Explicit instantiations
template double L3::Estimator::KLCostFunction<double>::operator()( const Histogram<double>& exp, const Histogram<double>& swathe, const L3::SE3& estimated_pose );
template double L3::Estimator::DiscreteEstimator<double>::operator()( PointCloud<double>* swathe, SE3 estimate );
