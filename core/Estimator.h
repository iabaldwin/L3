#ifndef L3_ESTIMATOR_H
#define L3_ESTIMATOR_H

#include <Eigen/LU>
#include <tbb/task.h>
#include <tbb/task_group.h>

#include "Histogram.h"
#include "Smoother.h"
namespace L3
{
namespace Estimator
{

    struct PoseEstimates : Lockable
    {
        boost::shared_ptr<L3::SE3> position;

        std::vector<double> costs;
        
        std::vector< L3::SE3 > estimates;

        typedef std::vector< L3::SE3 >::iterator ESTIMATES_ITERATOR; 

        virtual void operator()( const L3::SE3& pose ) 
        {}

        virtual ~PoseEstimates()
        {}

    };


    struct GridEstimates : PoseEstimates
    {
        GridEstimates( float x_width=10.0, float y_width=10.0, float spacing=1.0 ) 
            : x_width(x_width), 
                y_width(y_width), 
                spacing(spacing) 
        {
            position.reset( new L3::SE3( L3::SE3::ZERO() ) );
        }

        float x_width, y_width, spacing;

        virtual void operator()( const L3::SE3& pose ) 
        {
            L3::WriteLock( this->mutex );

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

    };

    /*
     *  Smoothing policy
     */
    template <typename T>
        struct SmoothingPolicy
        {
            virtual T  P( T  p ) = 0;
            virtual T  Q( T  q ) = 0;
        };


    template <typename T>
        struct NoneSmoothing : SmoothingPolicy<T>
    {

        T P( T  p ) 
        {
            return p;
        }

        T Q( T  q ) 
        {
            return q;
        }

    };

    template <typename T>
        struct EpsilonSmoothing : SmoothingPolicy<T>
    {
        EpsilonSmoothing( T p_norm, T q_norm ) : SmoothingPolicy<T>( p_norm, q_norm )
        {

        }

        T P( T  p ) 
        {
            return p + std::numeric_limits<T>::epsilon();
        }

        T Q( T  q ) 
        {
            return q + std::numeric_limits<T>::epsilon();
        }
    };

    /*
     *  Cost function
     */
    template < typename T >
        struct CostFunction
        {
            virtual double operator()( const Histogram<T>& experience, const Histogram<T>& swathe ) = 0;

            virtual ~CostFunction()
            {

            }

        };

    template < typename T >  
        struct KLCostFunction : CostFunction<T>
    {
        double operator()( const Histogram<T>& experience, const Histogram<T>& swathe );
    };

    /*
     *Estimator types
     */
    template < typename T >
        struct Estimator
        {
            Estimator( CostFunction<T>* f, boost::shared_ptr<L3::Histogram<double> > experience ) 
                : cost_function(f), 
                     experience_histogram(experience)
            {
                swathe_histogram.reset( new L3::HistogramUniformDistance<double>() );
                pose_estimates.reset( new GridEstimates(2, 2, 1 ) );
            }

            CostFunction<T>*    cost_function;
            boost::shared_ptr<PoseEstimates>        pose_estimates;
            boost::shared_ptr<L3::Histogram<T> >    swathe_histogram;
            boost::shared_ptr<L3::Histogram<T> >    experience_histogram;

            virtual ~Estimator()
            {
            }

            virtual double operator()( PointCloud<T>* swathe, SE3 estimate ) = 0;

        };


    template < typename T >
        struct DiscreteEstimator : Estimator<T>
    {

        DiscreteEstimator( CostFunction<T>* f, boost::shared_ptr<L3::Histogram<double> > experience ) : Estimator<T>(f, experience)
        {
        }

        tbb::task_group group;
        double operator()( PointCloud<T>* swathe, SE3 estimate );

    };

    template < typename T >
        struct GroundTruthEstimator : Estimator<T>
    {

        GroundTruthEstimator( CostFunction<T>* f, boost::shared_ptr<L3::Histogram<double> > experience ) : Estimator<T>(f, experience)
        {
        }

        double operator()( PointCloud<T>* swathe, SE3 estimate );

    };


}
}

#endif
