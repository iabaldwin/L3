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

#include "MI/Entropy.h"
#include "MI/MutualInformation.h"
#include "MI/RenyiMutualInformation.h"

namespace L3
{
    namespace Estimator
    {
        /*
         *  Gridded X,Y pose estimate
         */
        void GridEstimates::operator()( const L3::SE3& pose ) 
        {
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
            position.reset( new L3::SE3( pose ) );
            estimates.clear();

            for( float delta = -1*lower; delta < upper; delta += spacing )
                estimates.push_back( L3::SE3( pose.X(), pose.Y(), pose.Z(), pose.R(), pose.P(), pose.Q()+delta ) );

            costs.resize( estimates.size(), std::numeric_limits<double>::infinity() );
        }


        /*
         *  Weightings
         */

        void GridWeighting::operator()( PoseEstimates* estimates ) 
        {
            if( PoseEstimates* ptr = dynamic_cast< GridEstimates* > (estimates ) )
            {
                boost::math::normal_distribution<double> normal_weighting(0,.100);

                for( std::vector< L3::SE3 >::iterator it = ptr->estimates.begin();
                        it != ptr->estimates.end();
                        it++ )
                {
                    double dist = sqrt( pow( it->X() - pose.X(), 2 ) +
                            pow( it->Y() - pose.Y(), 2 ) );

                    double d = boost::math::pdf( normal_weighting , dist); 

                    ptr->costs[ std::distance( ptr->estimates.begin(),it) ] *= d;
                }
            }
        }


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
                if ( experience.empty() || swathe.empty() )
                    return std::numeric_limits<T>::infinity();

                int size = experience.hist->nx*experience.hist->ny;

                return -1*calculateMutualInformation( experience.hist->bin, swathe.hist->bin, size);
            }

        template < typename T >
            double NMICostFunction<T>::operator()( const Histogram<T>& experience, const Histogram<T>& swathe )
            {
                if ( experience.empty() || swathe.empty() )
                    return std::numeric_limits<T>::infinity();

                int size = experience.hist->nx*experience.hist->ny;

                return -1*( calculateMutualInformation( experience.hist->bin, swathe.hist->bin, size)/ (calculateEntropy( experience.hist->bin, size) + calculateEntropy( swathe.hist->bin, size ) ) );

            }


        template < typename T >
            double RenyiMICostFunction<T>::operator()( const Histogram<T>& experience, const Histogram<T>& swathe )
            {
                if ( experience.empty() || swathe.empty() )
                    return std::numeric_limits<T>::infinity();

                int size = experience.hist->nx*experience.hist->ny;

                return -1*calculateRenyiMIDivergence( 0.05, experience.hist->bin, swathe.hist->bin, size);
            }


        template < typename T >
            double SSDCostFunction<T>::operator()( const Histogram<T>& experience, const Histogram<T>& swathe )
            {
                if ( experience.empty() || swathe.empty() )
                    return std::numeric_limits<T>::infinity();

                int size = experience.hist->nx*experience.hist->ny;

                // Compute differences
                double diff[size];

                std::transform( experience.hist->bin, experience.hist->bin+size, swathe.hist->bin, diff, std::minus<double>() );
                std::for_each( diff, diff+size, std::bind2nd( std::ptr_fun( pow ), 2 ) );
                double retval = std::accumulate( diff, diff+size, 0.0  );

                return -1*retval;

            }

        template < typename T >
            double BattacharyaCostFunction<T>::operator()( const Histogram<T>& experience, const Histogram<T>& swathe )
            {
                if ( experience.empty() || swathe.empty() )
                    return std::numeric_limits<T>::infinity();

                int size = experience.hist->nx*experience.hist->ny;

                // Compute distance
                double distance[size];

                //std::inner_product( 
                // etc... 

                return -1;

            }




        /*
         *  Hypothesis Builder
         */
            void Hypothesis::operator()()
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

                L3::LogisticSmoother<double> smoother;
                //L3::GaussianSmoother<double> smoother;
                smoother.smooth( &swathe_histogram );

                *result_iterator = cost_function->operator()( *this->experience, swathe_histogram );
            }

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

                if (swathe->num_points == 0 ) 
                    return false;

                //  Speed considerations
                L3::sample( swathe, this->sampled_swathe.get(), 2*1000, false );
                
                // Allocate results
                std::vector<double>::iterator result_iterator = this->pose_estimates->costs.begin();
                // Pointer to beginning
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
                GridWeighting weighting(estimate);

                discrete_estimators[0]->cost_function = this->cost_function.get();
                discrete_estimators[0]->operator()( swathe, estimate );
                std::vector<double>::iterator it = std::min_element( discrete_estimators[0]->pose_estimates->costs.begin() , discrete_estimators[0]->pose_estimates->costs.end() );
                SE3 refined = discrete_estimators[0]->pose_estimates->estimates[ std::distance( discrete_estimators[0]->pose_estimates->costs.begin(), it )] ;

                discrete_estimators[1]->cost_function = this->cost_function.get();
                discrete_estimators[1]->operator()( swathe, refined );
                it = std::min_element( discrete_estimators[1]->pose_estimates->costs.begin() , discrete_estimators[1]->pose_estimates->costs.end() );
                refined = discrete_estimators[1]->pose_estimates->estimates[ std::distance( discrete_estimators[1]->pose_estimates->costs.begin(), it )] ;

                discrete_estimators[2]->cost_function = this->cost_function.get();
                discrete_estimators[2]->operator()( swathe, refined );
                it = std::min_element( discrete_estimators[2]->pose_estimates->costs.begin() , discrete_estimators[2]->pose_estimates->costs.end() );
                refined = discrete_estimators[2]->pose_estimates->estimates[ std::distance( discrete_estimators[2]->pose_estimates->costs.begin(), it )] ;

                discrete_estimators[3]->cost_function = this->cost_function.get();
                discrete_estimators[3]->operator()( swathe, refined );
                it = std::min_element( discrete_estimators[3]->pose_estimates->costs.begin() , discrete_estimators[3]->pose_estimates->costs.end() );
                refined = discrete_estimators[3]->pose_estimates->estimates[ std::distance( discrete_estimators[3]->pose_estimates->costs.begin(), it )] ;

                //discrete_estimators[4]->operator()( swathe, refined );
                //it = std::min_element( discrete_estimators[4]->pose_estimates->costs.begin() , discrete_estimators[4]->pose_estimates->costs.end() );
                //refined = discrete_estimators[4]->pose_estimates->estimates[ std::distance( discrete_estimators[4]->pose_estimates->costs.begin(), it )] ;

                //discrete_estimators[5]->operator()( swathe, refined );
                //it = std::min_element( discrete_estimators[5]->pose_estimates->costs.begin() , discrete_estimators[5]->pose_estimates->costs.end() );
                //refined = discrete_estimators[5]->pose_estimates->estimates[ std::distance( discrete_estimators[5]->pose_estimates->costs.begin(), it )] ;

                *(this->current_prediction) = refined;
                
                return refined;
            }

        template < typename T>
            SE3 PassThrough<T>::operator()( PointCloud<T>* swathe, SE3 estimate ) 
            {
                // Lock the experience histogram
                L3::ReadLock histogram_read_lock( (*pyramid)[1]->mutex );

                if (swathe->num_points == 0 ) 
                    return L3::SE3::ZERO();

                /*
                 *  Speed considerations
                 */
                L3::sample( swathe, this->sampled_swathe.get(), 1000, false );

                std::vector< double > costs(1);

                Hypothesis h( swathe, &estimate, (*pyramid)[1].get() , this->algorithm->cost_function.get(), costs.begin() );
                h();

                
                return L3::SE3::ZERO();
            }



        /*
         *  Algorithm: Minimisation
         */

        double global_minimisation_function( const gsl_vector * x, void * params)
        {
            Minimisation<double>* minimiser = static_cast< Minimisation<double>* >( MinimisationParameters::global_minimiser );

            return minimiser->getHypothesisCost( x );
        }

        L3::Estimator::Algorithm<double>* L3::Estimator::MinimisationParameters::global_minimiser = NULL;

        template < typename T >
            SE3 Minimisation<T>::operator()( PointCloud<T>* swathe, SE3 estimate )
            {
                if( timer.elapsed() < 1.0/this->fundamental_frequency )
                    return estimate;
                
                timer.begin();

                //DBG
                //predicted = estimate;
                //DBG

                int _pyramid_index = this->pyramid_index;

                L3::WriteLock master( this->mutex );
                L3::ReadLock  histogram_lock( (*this->pyramid)[_pyramid_index]->mutex );

                _cost_function = this->cost_function.get();

                current_swathe = swathe;

                gsl_vector_set (x, 0, estimate.X() );
                gsl_vector_set (x, 1, estimate.Y() );
                gsl_vector_set (x, 2, estimate.Q() );

                // Construct tolerances
                static int counter = 0;

                if ( counter++ == 120 )
                {
                    gsl_vector_set_all (ss, 0.05);
                    gsl_vector_set( ss, 2, .01 );
                }
                else
                {
                    gsl_vector_set_all (ss, 0.5);
                    gsl_vector_set( ss, 2, .05 );
                }
                //gsl_vector_set( ss, 2, .05 );

                gsl_multimin_fminimizer_set (s, &minex_func, x, ss);

                int status;
                algorithm_iterations = 0;
                evaluations.clear();
                evaluations.reserve(max_iterations+1);
              
                evaluations.push_back(estimate);

                double size;            
                
                do
                {
                    algorithm_iterations++;
                    status = gsl_multimin_fminimizer_iterate(s);

                    if (status) 
                        break;

                    size = gsl_multimin_fminimizer_size (s);
                    status = gsl_multimin_test_size (size, tolerance );

                    if (status == GSL_SUCCESS)
                        break;
                
                }
                while (status == GSL_CONTINUE && algorithm_iterations < max_iterations);

                gsl_vector* res = gsl_multimin_fminimizer_x ( s );

                L3::SE3 pose( gsl_vector_get(res, 0), gsl_vector_get(res, 1), 0.0, 0.0, 0.0, gsl_vector_get(res, 2) );
            
                *(this->current_prediction) = pose;

                return  pose;
            }

    template <typename T>
        double Minimisation<T>::getHypothesisCost( const gsl_vector* hypothesis )
        {
            std::vector<double> cost( 1 );

            //Construct pose from hypothesis
            L3::SE3 pose_estimate( gsl_vector_get( hypothesis, 0 ), gsl_vector_get( hypothesis, 1 ), 0.0, 0.0, 0.0, gsl_vector_get( hypothesis, 2 ) );

            evaluations.push_back( pose_estimate );


            // Estimate, here
            Hypothesis( this->current_swathe, &pose_estimate, (*this->pyramid)[this->pyramid_index].get() , this->_cost_function, cost.begin() )();

            //return cost[0]+dist/100.0;
            //double dist = sqrt( pow( pose_estimate.X() - predicted.X(),2 ) + pow( pose_estimate.Y() - predicted.Y(),2 ) );
            return cost[0];
        }

    template <typename T>
        SE3 Hybrid<T>::operator()( PointCloud<T>* swathe, SE3 estimate )

        {
            discrete_estimators[0]->cost_function = this->cost_function.get();
            discrete_estimators[0]->operator()( swathe, estimate );
            std::vector<double>::iterator it = std::min_element( discrete_estimators[0]->pose_estimates->costs.begin() , discrete_estimators[0]->pose_estimates->costs.end() );
            SE3 refined = discrete_estimators[0]->pose_estimates->estimates[ std::distance( discrete_estimators[0]->pose_estimates->costs.begin(), it )] ;

            discrete_estimators[1]->cost_function = this->cost_function.get();
            discrete_estimators[1]->operator()( swathe, estimate );
            it = std::min_element( discrete_estimators[0]->pose_estimates->costs.begin() , discrete_estimators[0]->pose_estimates->costs.end() );
            refined = discrete_estimators[0]->pose_estimates->estimates[ std::distance( discrete_estimators[0]->pose_estimates->costs.begin(), it )] ;

            minimisation->max_iterations = 20;

            return this->minimisation->operator()( swathe, refined );
        
        }


    double dlib_minimisation_function( const dlib::matrix<double,0,1>& x )
    {
        BFGS<double>* minimiser = static_cast< BFGS<double>* >( DLIBParameters::global_minimiser );
  
        double cost = minimiser->getHypothesisCost( x );
        
        return cost;
    }   
        
    L3::Estimator::Algorithm<double>* L3::Estimator::DLIBParameters::global_minimiser = NULL;

    template <typename T>
        SE3 BFGS<T>::operator()( PointCloud<T>* swathe, SE3 estimate )
        {
            int _pyramid_index = this->pyramid_index;

            L3::WriteLock master( this->mutex );
            L3::ReadLock  histogram_lock( (*this->pyramid)[_pyramid_index]->mutex );

            _cost_function = this->cost_function.get();

            current_swathe = swathe;

            dlib::matrix<double,0,1> starting_point;
            starting_point.set_size(3);

            starting_point(0) = estimate.X();
            starting_point(1) = estimate.Y();
            starting_point(2) = estimate.Q();

            //dlib::find_min_using_approximate_derivatives( dlib::bfgs_search_strategy(),
            dlib::find_max_using_approximate_derivatives( dlib::cg_search_strategy(),
                    dlib::objective_delta_stop_strategy(1e-8).be_verbose(),
                    &dlib_minimisation_function, 
                    starting_point, 
                    -std::numeric_limits<double>::infinity() );

            std::cout << "--------------" << std::endl;

            return L3::SE3( starting_point(0), starting_point(1), 0.0, 0.0, 0.0, starting_point(2) );
        }


    template <typename T>
        double BFGS<T>::getHypothesisCost( const dlib::matrix<double,0,1>& hypothesis )
        {
            std::vector<double> cost( 1 );

            //Construct pose from hypothesis
            L3::SE3 pose_estimate( hypothesis( 0 ), hypothesis( 1 ), 0.0, 0.0, 0.0, hypothesis( 2 ) );

            std::cout.precision(12);
            std::cout << pose_estimate << std::endl;

            evaluations.push_back( pose_estimate );

            // Estimate, here
            Hypothesis( this->current_swathe, &pose_estimate, (*this->pyramid)[this->pyramid_index].get() , this->_cost_function, cost.begin() )();

            return cost[0];
        }

    
    
    }   // Estimator
}       // L3

// Explicit instantiations
template double L3::Estimator::KLCostFunction<double>::operator()(L3::Histogram<double> const&, L3::Histogram<double> const&);
template double L3::Estimator::RenyiMICostFunction<double>::operator()(L3::Histogram<double> const&, L3::Histogram<double> const&);
template double L3::Estimator::NMICostFunction<double>::operator()(L3::Histogram<double> const&, L3::Histogram<double> const&);
template double L3::Estimator::MICostFunction<double>::operator()(L3::Histogram<double> const&, L3::Histogram<double> const&);
template double L3::Estimator::SSDCostFunction<double>::operator()(L3::Histogram<double> const&, L3::Histogram<double> const&);

template bool L3::Estimator::DiscreteEstimator<double>::operator()(L3::PointCloud<double>*, L3::SE3);

template L3::SE3 L3::Estimator::PassThrough<double>::operator()(L3::PointCloud<double>*, L3::SE3);
template L3::SE3 L3::Estimator::IterativeDescent<double>::operator()(L3::PointCloud<double>*, L3::SE3);
template L3::SE3 L3::Estimator::Minimisation<double>::operator()(L3::PointCloud<double>*, L3::SE3);
template L3::SE3 L3::Estimator::Hybrid<double>::operator()(L3::PointCloud<double>*, L3::SE3);
template L3::SE3 L3::Estimator::BFGS<double>::operator()(L3::PointCloud<double>*, L3::SE3);
