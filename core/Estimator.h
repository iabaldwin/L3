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

        PoseEstimates() : position( new L3::SE3( L3::SE3::ZERO() ) )
        {

        }

        std::vector<double> costs;
        
        std::vector< L3::SE3 > estimates;
            
        boost::shared_ptr<L3::SE3> position;
                
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

        void operator()( const L3::SE3& pose ) ;
    };


    struct RotationEstimates : PoseEstimates
    {
        RotationEstimates( float lower=1.0, float upper=1.0, float spacing=0.1 ) 
            : lower(lower), 
                upper(upper), 
                spacing(spacing) 
        {
            position.reset( new L3::SE3( L3::SE3::ZERO() ) );
        }

        float lower, upper, spacing;

        void operator()( const L3::SE3& pose ) ;
    };




    /*
     *  Smoothing policy
     */
    template <typename T>
        struct SmoothingPolicy
        {
            virtual void P( T& p ) = 0;
            virtual void Q( T& q ) = 0;
        };


    template <typename T>
        struct NoneSmoothing : SmoothingPolicy<T>
    {

        void P( T& p ) 
        {
        }

        void Q( T& q ) 
        {
        }

    };

    template <typename T>
        struct EpsilonSmoothing : SmoothingPolicy<T>
    {
        EpsilonSmoothing( T p_norm, T q_norm ) : SmoothingPolicy<T>( p_norm, q_norm )
        {

        }

        void P( T& p ) 
        {
            p = (p + std::numeric_limits<T>::epsilon());
        }

        void Q( T& q ) 
        {
            q = (q + std::numeric_limits<T>::epsilon());
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
     * Estimator types
     */
    template < typename T >
        struct Estimator 
        {
            Estimator( CostFunction<T>* f, boost::shared_ptr< L3::Histogram<double> > experience ) 
                : cost_function(f), 
                     experience_histogram(experience)
            {
                // Allocations
                swathe_histogram.reset( new L3::HistogramUniformDistance<double>() );
                current_swathe.reset( new L3::PointCloud<double>() );
                current_histogram.reset( new L3::Histogram<double>() );
                sampled_swathe.reset( new PointCloud<T>() );
            }

            CostFunction<T>*                        cost_function;
            boost::shared_ptr<PoseEstimates>        pose_estimates;
            boost::shared_ptr<L3::Histogram<T> >    swathe_histogram;
            boost::shared_ptr<L3::Histogram<T> >    current_histogram;
            boost::shared_ptr<L3::Histogram<T> >    experience_histogram;
            boost::shared_ptr<L3::PointCloud<T> >   current_swathe;

            boost::shared_ptr< PointCloud<T> > sampled_swathe;
            
            virtual ~Estimator()
            {
            }

            virtual bool operator()( PointCloud<T>* swathe, SE3 estimate ) = 0;

        };


    template < typename T >
        struct DiscreteEstimator : Estimator<T>
        {
            //DiscreteEstimator( CostFunction<T>* f, boost::shared_ptr< L3::Histogram<T> > experience, float lower=10.0, float upper=10.0, float granularity=1.0 ) 
                //: Estimator<T>(f, experience)

            DiscreteEstimator( CostFunction<T>* f, boost::shared_ptr< L3::Histogram<T> > experience, boost::shared_ptr< PoseEstimates > estimates )
                : Estimator<T>(f, experience)
            {
                //this->pose_estimates.reset( new GridEstimates( lower, upper, granularity ) );
                this->pose_estimates = estimates;
            }
            
            tbb::task_group group;
            
            void dump(){};

            bool operator()( PointCloud<T>* swathe, SE3 estimate );

        };

    template < typename T>
        struct Algorithm
        {
            virtual SE3 operator()( PointCloud<T>* swathe, SE3 estimate ) = 0;
        };

    template < typename T>
        struct PassThrough : Algorithm<T>
        {
            PassThrough(  Estimator<T>* estimator ) : estimator(estimator)
            {
            }

            Estimator<T>* estimator;

            SE3 operator()( PointCloud<T>* swathe, SE3 estimate ) 
            {
                estimator->operator()( swathe, estimate );
           
                return estimate;
            }
        };

    template < typename T>
        struct IterativeDescent : Algorithm<T>
        {
            IterativeDescent( CostFunction<T>* cost_function, boost::shared_ptr< L3::HistogramPyramid<T> > experience_pyramid ) : pyramid(experience_pyramid)
            {

                GridEstimates       grid ( 10, 10, 1);
                RotationEstimates   rotation( .5, .5, .1);

                for( typename L3::HistogramPyramid<T>::PYRAMID_ITERATOR it = pyramid->begin();
                        it != pyramid->end();
                        it++ )
                {
                    L3::ReadLock( (*it)->mutex );

                    // Grid 
                    discrete_estimators.push_back( 
                            boost::make_shared< DiscreteEstimator<T> >( cost_function, *it, boost::shared_ptr< GridEstimates >( new GridEstimates( 10, 10, 1) ) )
                            );
              
                    // Rotation
                    discrete_estimators.push_back( 
                            boost::make_shared< DiscreteEstimator<T> >( cost_function, *it, boost::shared_ptr< RotationEstimates >( new RotationEstimates( .5 , .5, .1 ) ) )
                            );

                }

            }
            
            boost::shared_ptr< HistogramPyramid<T> > pyramid;
       
            std::deque< boost::shared_ptr< DiscreteEstimator<T> > > discrete_estimators;
            
            SE3 operator()( PointCloud<T>* swathe, SE3 estimate );

        };

    struct AlgorithmFactory 
    {
        //static boost::shared_ptr< Algorithm > produce( std::string algorithm_type, std::string cost_function )
        //{

        //}

    };
}
}

#endif
