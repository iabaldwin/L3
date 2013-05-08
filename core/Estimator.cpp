#include "Estimator.h"

#include <iostream>
#include <iterator>
#include <fstream>
#include <numeric>
#include <boost/timer.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_array.hpp>

#include <boost/math/special_functions/log1p.hpp>

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


                // Policy should be here
                p_i = (p_i == 0) ? std::numeric_limits<T>::epsilon() : p_i;
                q_i = (q_i == 0) ? std::numeric_limits<T>::epsilon() : q_i;

                if ( q_i==0 || p_i ==0 || q_norm==0 || p_norm == 0 )
                    {
                    std::cout << p << "," << p_norm << " " << q << q_norm << std::endl;
                    exit(1);
                    }

                //double val = boost::math::log1p( (p_i/q_i))*p_i;

                //return val;
            
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
                L3::sample( swathe, this->sampled_swathe.get(), 2000 );
                
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

        /*
         *  Ground truth Estimator
         */
        template < typename T >
            bool GroundTruthEstimator<T>::operator()( PointCloud<T>* swathe, SE3 estimate ) 
            {
                // Lock the experience histogram
                //L3::ReadLock histogram_lock( this->experience_histogram->mutex );
                //// Lock the swathe
                //L3::ReadLock swathe_lock( swathe->mutex );
                //// Estimates
                //L3::WriteLock estimates_lock( this->pose_estimates->mutex );

                //this->pose_estimates->costs.resize( 1 );
                //this->pose_estimates->estimates.clear();
                //this->pose_estimates->estimates.push_back( estimate );

                ////Hypothesis( swathe, &estimate, this->experience_histogram.get() , this->cost_function, this->pose_estimates->costs.begin() )();

                /*
                 *  Copy point cloud
                 */
                //boost::scoped_ptr< L3::PointCloud<double> > hypothesis( new L3::PointCloud<double>() );
                //L3::copy( const_cast<L3::PointCloud<double>* >(swathe), hypothesis.get() );

                //L3::transform( hypothesis.get(), &estimate ); 

                /*
                 *  Histogram
                 */
                //L3::Histogram<double> swathe_histogram;

                //L3::copy( const_cast<L3::Histogram<double>*>(this->experience_histogram.get()), &swathe_histogram );

                //// Produce swathe histogram
                //swathe_histogram( hypothesis.get() );


                // DBG
                //L3::clone( &swathe_histogram, this->current_histogram.get() );
                //L3::copy( swathe, this->current_swathe.get() );
                //L3::copy( hypothesis.get(), this->current_swathe.get() );

            }

        std::list< boost::shared_ptr< DataWriter<L3::PointCloud<double> > > > cloud_writers;
        std::list< boost::shared_ptr< DataWriter<L3::Histogram<double> > > > histogram_writers;

        template < typename T >
            void GroundTruthEstimator<T>::dump()
            {
                //boost::shared_ptr< DataWriter< L3::PointCloud<double> > > cloud_writer( new DataWriter< L3::PointCloud<double> >( this->current_swathe.get() ) );
                //cloud_writer->write(  "point_cloud.dat" );
                //cloud_writers.push_back( cloud_writer );

                //boost::shared_ptr< DataWriter< L3::Histogram<double> > > histogram_writer( new DataWriter< L3::Histogram<double> >( this->current_histogram.get() ) );
                //histogram_writer->write(  "histogram.dat" );
                //histogram_writers.push_back( histogram_writer );

                //std::ofstream point_cloud_file( "point_cloud.dat" );
                //point_cloud_file << *this->current_swathe;
                //point_cloud_file.close();

                //std::ofstream histogram_file( "histogram.dat" );
                //histogram_file << *this->current_histogram;
                //histogram_file.close();

            }


    template < typename T>
        SE3 IterativeDescent<T>::operator()( PointCloud<T>* swathe, SE3 estimate )
        {
            discrete_estimators[0]->operator()( swathe, estimate );
            std::vector<double>::iterator it = std::min( discrete_estimators[0]->pose_estimates->costs.begin() , discrete_estimators[0]->pose_estimates->costs.end() );
            return discrete_estimators[0]->pose_estimates->estimates[ std::distance( discrete_estimators[0]->pose_estimates->costs.begin(), it )] ;

            //discrete_estimators[1]->operator()( swathe, estimate );
            //discrete_estimators[2]->operator()( swathe, estimate );
        }

    }   // Estimator
}       // L3

// Explicit instantiations
template double L3::Estimator::KLCostFunction<double>::operator()(L3::Histogram<double> const&, L3::Histogram<double> const&);
template bool L3::Estimator::DiscreteEstimator<double>::operator()(L3::PointCloud<double>*, L3::SE3);
template bool L3::Estimator::GroundTruthEstimator<double>::operator()(L3::PointCloud<double>*, L3::SE3);
template void L3::Estimator::GroundTruthEstimator<double>::dump();
template L3::SE3 L3::Estimator::IterativeDescent<double>::operator()(L3::PointCloud<double>*, L3::SE3);
