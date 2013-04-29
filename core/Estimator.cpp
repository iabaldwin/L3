#include "Estimator.h"

#include <iostream>
#include <iterator>
#include <fstream>
#include <numeric>
#include <boost/timer.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_array.hpp>

#include <boost/math/special_functions/log1p.hpp>

namespace L3
{
    /*
     *  Estimator types
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
        template <typename T>
            struct KL : std::binary_function<double,double,double>
        {
            KL( double p_normalizer, double q_normalizer, SmoothingPolicy<T>* policy ) 
                : p_norm(p_normalizer), 
                    q_norm(q_normalizer), 
                    policy(policy)
            {
                assert( p_normalizer && q_normalizer );
            }

            double p_norm, q_norm;

            SmoothingPolicy<T>* policy;

            double operator()( int p, int q )
            {
                double p_i = p/p_norm;
                double q_i = q/q_norm;

                p_i = (p_i == 0) ? std::numeric_limits<T>::epsilon() : p_i;
                q_i = (q_i == 0) ? std::numeric_limits<T>::epsilon() : q_i;

                double val = boost::math::log1p( (p_i/q_i))*p_i;

                if ( std::isnan( val ) )
                    throw std::exception();
                //{
                    //std::cout << p << "," << q << "," << p_i << ":" << q_i << "->" << val <<  std::endl;
                //}

                return val;
            }

        };

        template < typename T >
            double KLCostFunction<T>::operator()( const Histogram<T>& experience, const Histogram<T>& swathe )
            {
                // Convert to probability
                double experience_normalizer = experience.normalizer();
                double swathe_normalizer     = swathe.normalizer();

                if( swathe_normalizer == 0 )
                    return std::numeric_limits<T>::infinity();

                NoneSmoothing<T> no_smoothing;

                KL<T> kl( experience_normalizer, swathe_normalizer, &no_smoothing );

                boost::shared_array<double> kl_estimate( new double[(experience.x_bins * experience.y_bins)] );

                // Compute the cell-wise divergence
                std::transform( experience.hist->bin, experience.hist->bin + (experience.x_bins * experience.y_bins), swathe.hist->bin, kl_estimate.get(), kl );

                // Accumulate 
                return std::accumulate( kl_estimate.get(), kl_estimate.get()+(experience.x_bins * experience.y_bins), T(0) );
            }

        /*
         *  Hypothesis Builder
         */
        struct Hypothesis
        {
            Hypothesis( L3::PointCloud<double> const * swathe, 
                        L3::SE3 const* estimate, 
                        L3::Histogram<double> const* experience , 
                        CostFunction<double>* cost_function, 
                        __gnu_cxx::__normal_iterator<double*, std::vector<double, std::allocator<double> > > result_iterator )
                        : swathe(swathe), 
                            estimate(estimate), 
                            experience(experience), 
                            cost_function(cost_function),
                            result_iterator(result_iterator)
            {
            }

            L3::SE3 const*                  estimate;
            CostFunction<double> *          cost_function;
            L3::Histogram<double> const*    experience;
            L3::PointCloud<double> const *  swathe;
            __gnu_cxx::__normal_iterator<double*, std::vector<double, std::allocator<double> > > result_iterator ;
            
            void operator()()
            {
                boost::scoped_ptr< L3::PointCloud<double> > hypothesis( new L3::PointCloud<double>() );

                std::cout << *estimate << std::endl;

                /*
                 *  Copy point cloud
                 */
                L3::copy( const_cast<L3::PointCloud<double>* >(swathe), hypothesis.get() );

                L3::transform( hypothesis.get(), const_cast<L3::SE3*>(estimate) ); 

                /*
                 *  Histogram
                 */
                L3::Histogram<double> swathe_histogram;

                L3::copy( const_cast<L3::Histogram<double>*>(experience), &swathe_histogram );

                // Produce swathe histogram
                swathe_histogram( hypothesis.get() );

                // Compute cost
                *result_iterator = cost_function->operator()( *this->experience, swathe_histogram );
            }

        };

        /*
         *  Discrete Estimator
         */
        template < typename T >
            double DiscreteEstimator<T>::operator()( PointCloud<T>* swathe, SE3 estimate ) 
            {

                // Rebuild pose estimates
                this->pose_estimates->operator()( estimate );

                if ( this->experience_histogram->empty() )
                    return std::numeric_limits<T>::infinity();

                // Lock the experience histogram
                L3::ReadLock histogram_lock( this->experience_histogram->mutex );
                L3::ReadLock swathe_lock( swathe->mutex );

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

                this->pose_estimates->costs.resize( this->pose_estimates->estimates.size() );
                std::vector<double>::iterator result_iterator = this->pose_estimates->costs.begin();

                std::vector< L3::SE3 >::iterator it = this->pose_estimates->estimates.begin();
                while( it != this->pose_estimates->estimates.end() )
                {
                    group.run( Hypothesis( swathe, &*it, this->experience_histogram.get() , this->cost_function, result_iterator++ ) );
                    break; 
                    it++;
                }

                // Synch
                group.wait();

            }

        /*
         *  Ground truth Estimator
         */
        template < typename T >
            double GroundTruthEstimator<T>::operator()( PointCloud<T>* swathe, SE3 estimate ) 
            {
                if ( this->experience_histogram->empty() )
                    throw std::exception();

                // Lock the experience histogram
                //L3::ReadLock histogram_lock( this->experience_histogram->mutex );
                //L3::ReadLock swathe_lock( swathe->mutex );
                
                //this->pose_estimates->costs.resize( 1 );
              
                //std::cout << *(&estimate) << std::endl;

                //Hypothesis( swathe, &estimate, this->experience_histogram.get() , this->cost_function, this->pose_estimates->costs.begin() );

            }

    }   // Estimator
}       // L3

// Explicit instantiations
template double L3::Estimator::KLCostFunction<double>::operator()(L3::Histogram<double> const&, L3::Histogram<double> const&);
template double L3::Estimator::DiscreteEstimator<double>::operator()(L3::PointCloud<double>*, L3::SE3);
template double L3::Estimator::GroundTruthEstimator<double>::operator()(L3::PointCloud<double>*, L3::SE3);
