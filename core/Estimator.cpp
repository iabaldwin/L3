#include "Estimator.h"

#include <iostream>
#include <iterator>
#include <fstream>
#include <numeric>
#include <boost/timer.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_array.hpp>

#include "Poco/Runnable.h"
#include "Poco/Thread.h"

#include "Timing.h"

template <typename T>
struct DataWriter : Poco::Runnable
{

    DataWriter( T* t ) : t(t)
    {
        
    }

    T* t;
        
    Poco::Thread thread;

    std::string target;

    ~DataWriter()
    {
    }

    virtual void write( std::string target )
    {
        this->target = target;
        thread.start( *this );
    }

    virtual void run()
    {
        std::ofstream output( target.c_str() );
        output << *t;
        output.close(); 
    }

};

namespace L3
{

    namespace Estimator
    {
        /*
         *  Gridded X,Y pose estimate
         */
        void GridEstimates::operator()( const L3::SE3& pose ) 
        {
            L3::WriteLock( this->mutex );
            
            position.reset( new L3::SE3( pose ) );
            estimates.clear();

            for( float x_delta = -1*x_width; x_delta < x_width; x_delta += spacing )
                for( float y_delta = -1*y_width; y_delta < y_width; y_delta += spacing )
                {
                    L3::SE3 estimate( x_delta, y_delta, 0, 0, 0, 0 ) ;

                    Eigen::Matrix4f res = const_cast<L3::SE3*>(&pose)->getHomogeneous()*estimate.getHomogeneous();

                    estimates.push_back( L3::SE3( res(0,3), res(1,3), res(2,3), pose.R(), pose.P(), pose.Q() ) );
                }
       
            costs.resize( estimates.size(), std::numeric_limits<double>::infinity() );
        }


        /*
         *  Rotational pose estimates
         */
        void RotationEstimates::operator()( const L3::SE3& pose )
        {
            L3::WriteLock( this->mutex );
            
            position.reset( new L3::SE3( pose ) );
            estimates.clear();

            for( float delta = -1*lower; delta < upper; delta += spacing )
                estimates.push_back( L3::SE3( pose.X(), pose.Y(), pose.Z(), pose.R(), pose.P(), pose.Q()+delta ) );

            costs.resize( estimates.size(), std::numeric_limits<double>::infinity() );
        }


        /*
         *  Cost Functions
         */


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


                // Policy should be here
                p_i = (p_i == 0) ? std::numeric_limits<T>::epsilon() : p_i;
                q_i = (q_i == 0) ? std::numeric_limits<T>::epsilon() : q_i;

                //policy->P( p_i );
                //policy->Q( q_i );

                //if ( q_i==0 || p_i ==0 || q_norm==0 || p_norm == 0 )
                    //{
                    //std::cout << p << "," << p_norm << " " << q << q_norm << std::endl;
                    //exit(1);
                    //}

                return boost::math::log1p( (p_i/q_i))*p_i;
            }

        };

        template < typename T >
            double KLCostFunction<T>::operator()( const Histogram<T>& experience, const Histogram<T>& swathe )
            {
                // Convert to probability
                double experience_normalizer = experience.normalizer();
                double swathe_normalizer     = swathe.normalizer();

                if( swathe_normalizer == 0 || experience_normalizer  == 0 )
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
         *  Mutual information
         */
        template < typename T >
            double MICostFunction<T>::operator()( const Histogram<T>& experience, const Histogram<T>& swathe )
            {
                double experience_normalizer = experience.normalizer();
                double swathe_normalizer     = swathe.normalizer();

                if( swathe_normalizer == 0 || experience_normalizer  == 0 )
                    return std::numeric_limits<T>::infinity();

                int num_bins = 100;

                gsl_histogram2d* joint = gsl_histogram2d_alloc( num_bins, num_bins );
                gsl_histogram* swathe_marginal = gsl_histogram_alloc( num_bins );
                gsl_histogram* experience_marginal = gsl_histogram_alloc( num_bins );

                // Find max of both
                double max_val = (std::max( experience.max(), swathe.max() ))+.5;

                gsl_histogram2d_set_ranges_uniform( joint, 0, max_val, 0, max_val );
                gsl_histogram_set_ranges_uniform( swathe_marginal, 0, max_val ) ;
                gsl_histogram_set_ranges_uniform( experience_marginal, 0, max_val ) ;

                double* experience_data = experience.hist->bin;
                double* swathe_data = swathe.hist->bin;

                for( int i=0; i< swathe.hist->nx*swathe.hist->ny; i++ )
                {
                    gsl_histogram2d_increment( joint, *experience_data, *swathe_data );

                    gsl_histogram_increment( swathe_marginal, *swathe_data ) ;
                    gsl_histogram_increment( experience_marginal, *experience_data );
                    
                    experience_data++;
                    swathe_data++;
                }

                double mi = compute_entropy(swathe_marginal) + compute_entropy(experience_marginal) - compute_entropy(joint);

                if( std::isnan( mi ) )
                {
                    std::cout << "INVALID" << std::endl;
                    mi = std::numeric_limits<T>::infinity();
                }

                gsl_histogram2d_free( joint );
                gsl_histogram_free( swathe_marginal );
                gsl_histogram_free( experience_marginal );

                return -1*mi;
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

            L3::PointCloud<double> const *  swathe;
            L3::SE3 const*                  estimate;
            L3::Histogram<double> const*    experience;
            CostFunction<double> *          cost_function;
            __gnu_cxx::__normal_iterator<double*, std::vector<double, std::allocator<double> > > result_iterator ;

            void operator()()
            {
                boost::scoped_ptr< L3::PointCloud<double> > hypothesis( new L3::PointCloud<double>() );

                /*
                 *  Copy point cloud
                 */
                L3::copy( const_cast<L3::PointCloud<double>* >(swathe), hypothesis.get() );

                /*
                 *  Transform cloud to the current estimate
                 */
                L3::transform( hypothesis.get(), const_cast<L3::SE3*>(estimate) ); 

                /*
                 *  Histogram
                 */
                L3::Histogram<double> swathe_histogram;

                if( !L3::copy( const_cast<L3::Histogram<double>*>(experience), &swathe_histogram ) )
                    return;
                
                // Produce swathe histogram
                swathe_histogram( hypothesis.get() );

                /*
                 *  Smoothing
                 */
                //L3::BoxSmoother< double, 5 > smoother;
                //smoother.smooth( &swathe_histogram );

                // Compute cost
                *result_iterator = cost_function->operator()( *this->experience, swathe_histogram );
            }

        };

        /*
         *  Discrete Estimator
         */
        template < typename T >
            bool DiscreteEstimator<T>::operator()( PointCloud<T>* swathe, SE3 estimate ) 
            {
                // Rebuild pose estimates
                this->pose_estimates->operator()( estimate );

                // Lock the experience histogram
                L3::ReadLock histogram_lock( this->experience_histogram->mutex );
                L3::ReadLock swathe_lock( swathe->mutex );

                if (swathe->num_points == 0 ) 
                    return false;

                /*
                 *  Speed considerations
                 */
                L3::sample( swathe, this->sampled_swathe.get(), 5000 );
                
                std::vector<double>::iterator result_iterator = this->pose_estimates->costs.begin();

                std::vector< L3::SE3 >::iterator it = this->pose_estimates->estimates.begin();

                while( it != this->pose_estimates->estimates.end() )
                {
                    group.run( Hypothesis( this->sampled_swathe.get(), &*it, this->experience_histogram.get() , this->cost_function, result_iterator++ ) );
                    it++;
                }

                // Synch
                group.wait();

                return true;

            }

    template < typename T>
        SE3 IterativeDescent<T>::operator()( PointCloud<T>* swathe, SE3 estimate )
        {
            discrete_estimators[0]->operator()( swathe, estimate );
            std::vector<double>::iterator it = std::min_element( discrete_estimators[0]->pose_estimates->costs.begin() , discrete_estimators[0]->pose_estimates->costs.end() );
            SE3 refined = discrete_estimators[0]->pose_estimates->estimates[ std::distance( discrete_estimators[0]->pose_estimates->costs.begin(), it )] ;

            discrete_estimators[1]->operator()( swathe, refined );
            it = std::min_element( discrete_estimators[1]->pose_estimates->costs.begin() , discrete_estimators[1]->pose_estimates->costs.end() );
            refined = discrete_estimators[1]->pose_estimates->estimates[ std::distance( discrete_estimators[1]->pose_estimates->costs.begin(), it )] ;

            discrete_estimators[2]->operator()( swathe, refined );
            it = std::min_element( discrete_estimators[2]->pose_estimates->costs.begin() , discrete_estimators[2]->pose_estimates->costs.end() );
            refined = discrete_estimators[2]->pose_estimates->estimates[ std::distance( discrete_estimators[2]->pose_estimates->costs.begin(), it )] ;

            discrete_estimators[3]->operator()( swathe, refined );
            it = std::min_element( discrete_estimators[3]->pose_estimates->costs.begin() , discrete_estimators[3]->pose_estimates->costs.end() );
            refined = discrete_estimators[3]->pose_estimates->estimates[ std::distance( discrete_estimators[3]->pose_estimates->costs.begin(), it )] ;

            return refined;
        }

    double global_minimisation_function( const gsl_vector * x, void * params)
    {


    }

    L3::Estimator::Algorithm<double>* L3::Estimator::MinimisationParameters::global_minimiser = NULL;

    template < typename T >
        SE3 Minimisation<T>::operator()( PointCloud<T>* swathe, SE3 estimate )
        {
            current_swathe = swathe;

            gsl_vector_set (x, 0, estimate.X() );
            gsl_vector_set (x, 1, estimate.Y() );
            gsl_vector_set (x, 2, estimate.Q() );

            gsl_vector_set_all (ss, 1.0);
        
        }

    template <typename T>
        double Minimisation<T>::getHypothesisCost( gsl_vector* hypothesis )
        {
            std::vector<double> cost( 1 );

            //Construct pose from hypothesis
            L3::SE3 pose_estimate( gsl_vector_get( hypothesis, 0 ), gsl_vector_get( hypothesis, 1 ), 0.0, 0.0, 0.0, gsl_vector_get( hypothesis, 2 ) );

            Hypothesis( this->current_swathe, &pose_estimate, (*this->pyramid)[0].get() , this->cost_function, cost.begin() );

        }
    
    }   // Estimator
}       // L3

// Explicit instantiations
template double L3::Estimator::KLCostFunction<double>::operator()(L3::Histogram<double> const&, L3::Histogram<double> const&);
template double L3::Estimator::MICostFunction<double>::operator()(L3::Histogram<double> const&, L3::Histogram<double> const&);
template bool L3::Estimator::DiscreteEstimator<double>::operator()(L3::PointCloud<double>*, L3::SE3);
template L3::SE3 L3::Estimator::IterativeDescent<double>::operator()(L3::PointCloud<double>*, L3::SE3);
template L3::SE3 L3::Estimator::Minimisation<double>::operator()(L3::PointCloud<double>*, L3::SE3);
