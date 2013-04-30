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
        std::cout << "Done" << std::endl;
    }

    virtual void write( std::string target )
    {
        this->target = target;
        thread.start( *this );
    }

    virtual void run()
    {
        std::cout << "Writing to " << target << std::endl;
        std::ofstream output( target.c_str() );
        output << *t;
        output.close(); 
        std::cout << "Done " << target << std::endl;
    }

};

namespace L3
{

    namespace Estimator
    {
    void GridEstimates::operator()( const L3::SE3& pose ) 
        {
            position.reset( new L3::SE3( pose ) );
            estimates.clear();
        
            for( float x_delta = -1*x_width; x_delta < x_width; x_delta += spacing )
                for( float y_delta = -1*y_width; y_delta < y_width; y_delta += spacing )
                {
                    L3::SE3 estimate( x_delta, y_delta, 0, 0, 0, 0 ) ;

                    Eigen::Matrix4f res = estimate.getHomogeneous()*const_cast<L3::SE3*>(&pose)->getHomogeneous();

                    estimates.push_back( L3::SE3( res(0,3), res(1,3), res(2,3), pose.R(), pose.P(), pose.Q() ) );
                }
        }



    /*
     *  Estimator types
     */
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

                //// Produce swathe histogram
                //swathe_histogram( hypothesis.get() );

                //// Compute cost
                //*result_iterator = cost_function->operator()( *this->experience, swathe_histogram );
            }

        };

        /*
         *  Discrete Estimator
         */
        template < typename T >
            double DiscreteEstimator<T>::operator()( PointCloud<T>* swathe, SE3 estimate ) 
            {
                L3::WriteLock estimates_lock( this->pose_estimates->mutex );

                std::cout << 1 << std::endl;
                // Rebuild pose estimates
                this->pose_estimates->operator()( estimate );

                std::cout << 2 << std::endl;
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

                std::cout << 3 << std::endl;
                this->pose_estimates->costs.resize( this->pose_estimates->estimates.size() );
                std::vector<double>::iterator result_iterator = this->pose_estimates->costs.begin();

                std::cout << 4 << std::endl;
                std::vector< L3::SE3 >::iterator it = this->pose_estimates->estimates.begin();
                while( it != this->pose_estimates->estimates.end() )
                {
                    std::cout << std::distance( this->pose_estimates->estimates.begin(), it ) << ":" << this->pose_estimates->estimates.size()  << std::endl;
                    group.run( Hypothesis( swathe, &*it, this->experience_histogram.get() , this->cost_function, result_iterator++ ) );
                    it++;
                }

                // Synch
                group.wait();

                //L3::clone( &swathe_histogram, this->current_histogram.get() );
                //L3::copy( swathe, this->current_swathe.get() );
                //L3::copy( hypothesis.get(), this->current_swathe.get() );

                std::cout << 5 << std::endl;

            }

        /*
         *  Ground truth Estimator
         */
        template < typename T >
            double GroundTruthEstimator<T>::operator()( PointCloud<T>* swathe, SE3 estimate ) 
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

    }   // Estimator
}       // L3

// Explicit instantiations
template double L3::Estimator::KLCostFunction<double>::operator()(L3::Histogram<double> const&, L3::Histogram<double> const&);
template double L3::Estimator::DiscreteEstimator<double>::operator()(L3::PointCloud<double>*, L3::SE3);
template double L3::Estimator::GroundTruthEstimator<double>::operator()(L3::PointCloud<double>*, L3::SE3);
template void L3::Estimator::GroundTruthEstimator<double>::dump();
