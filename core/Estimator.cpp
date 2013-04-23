#include "Estimator.h"
#include <iterator>
#include <numeric>
#include <boost/timer.hpp>
#include <boost/scoped_ptr.hpp>

namespace L3
{
    /*
     *Estimator types
     */
    namespace Estimator
    {

        /*
         *  Cost Functions
         */
        struct MI :  std::binary_function<double,double,double>
        {

        };


        /*
         *  KL divergence
         */
        struct KL : std::binary_function<double,double,double>
        {
            KL( double p_normaliser, double q_normaliser ) : p_norm(p_normaliser), q_norm(q_normaliser)
            {

            }

            double p_norm, q_norm;

            double operator()( double p, double q )
            {
                return log( ((p/p_norm)/(q/q_norm) )) *(p/p_norm);
            }

        };

        template <typename T>
            double KLCostFunction<T>::operator()( const Histogram<T>& experience, const Histogram<T>& swathe )
            {
                // Convert to probability
                double experience_normaliser = gsl_histogram2d_sum( experience.hist );
                double swathe_normaliser     = gsl_histogram2d_sum( swathe.hist );

                KL kl( experience_normaliser, swathe_normaliser );

                double* kl_estimate = new double[(experience.x_bins * experience.y_bins)];

                std::transform( experience.hist->bin, experience.hist->bin + (experience.x_bins * experience.y_bins), swathe.hist->bin, kl_estimate, kl );

                std::accumulate( kl_estimate, kl_estimate+(experience.x_bins * experience.y_bins), 0.0 );

                delete [] kl_estimate;

                return std::numeric_limits<T>::infinity();
            }


        /*
         *  Hypothesis Builder
         */
        struct HypothesisBuilder
        {
            HypothesisBuilder( L3::PointCloud<double> const * swathe, L3::SE3 const* estimate, L3::Histogram<double> const* experience , CostFunction<double>* cost_function ) 
                : swathe(swathe), estimate(estimate), experience(experience), cost_function(cost_function)
            {
            }

            double                          cost;

            L3::PointCloud<double> const *  swathe;
            L3::Histogram<double> const*    experience;
            CostFunction<double> *          cost_function;
            L3::SE3 const*                  estimate ;

            void operator()()
            {
                boost::scoped_ptr< L3::PointCloud<double> > hypothesis( new L3::PointCloud<double>() );
                L3::copy( const_cast<L3::PointCloud<double>* >(swathe), hypothesis.get() );

                L3::Histogram<double> swathe_histogram;
               
                L3::copy( experience, &swathe_histogram );
                
                // Transform to pose estimate
                transform( hypothesis.get(), const_cast<L3::SE3*>(estimate) );

                // Histogram 
                swathe_histogram( hypothesis.get() );

                // Compute cost
                cost = cost_function->operator()( *this->experience, swathe_histogram );

            }

        };

        /*
         *  Discrete Estimator
         */
        template <typename T>
            double DiscreteEstimator<T>::operator()( PointCloud<T>* swathe, SE3 estimate ) 
            {
                // Lock the experience histogram
                L3::ReadLock lock( this->experience_histogram->mutex );

                // Rebuild pose estimates
                this->pose_estimates->operator()( estimate );

                if ( __builtin_expect( (this->experience_histogram->empty() ) , 0 ) )
                    return std::numeric_limits<T>::infinity();

                /*
                 *  Speed considerations
                 */
                //PointCloud<T>* sampled_swathe = new PointCloud<T>();
                //L3::sample( swathe, sampled_swathe, 2000 );

                /*
                 *  Smoothing
                 */
                //L3::Smoother< double, 5 > smoother;
                //smoother.smooth( &swathe_histogram );
                
                std::vector< L3::SE3 >::iterator it = this->pose_estimates->estimates.begin();
                while( it != this->pose_estimates->estimates.end() )
                {
                    group.run( HypothesisBuilder( swathe, &estimate, this->experience_histogram.get() , this->cost_function ) );
                    it++;
                }

                // Synch
                group.wait();

            }

    }   // Estimator
}       // L3

// Explicit instantiations
template double L3::Estimator::KLCostFunction<double>::operator()( const Histogram<double>& exp, const Histogram<double>& swathe );
template double L3::Estimator::DiscreteEstimator<double>::operator()( PointCloud<double>* swathe, SE3 estimate );
