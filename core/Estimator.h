#ifndef L3_ESTIMATOR_H
#define L3_ESTIMATOR_H

#include <Eigen/LU>
#include <tbb/task.h>
#include <tbb/task_group.h>

#include <boost/bind.hpp>
#include <boost/math/distributions/normal.hpp>

#include "dlib/optimization.h"

#include "Timing.h"
#include "Smoother.h"
#include "Histogram.h"
#include "Experience.h"
#include "PoseProvider.h"

namespace L3
{
namespace Estimator
{
    struct PoseEstimates 
    {
        PoseEstimates() : position( new L3::SE3() )
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
            position.reset( new L3::SE3() );
        }

        float x_width, y_width, spacing;

        void operator()( const L3::SE3& pose ) ;
    };


    struct RotationEstimates : PoseEstimates
    {
        RotationEstimates( float lower=1, float upper=1, float spacing=0.01 ) 
            : lower(lower), 
                upper(upper), 
                spacing(spacing) 
        {
            position.reset( new L3::SE3() );
        }

        float lower, upper, spacing;

        void operator()( const L3::SE3& pose ) ;
    };

    struct Weighting
    {
        virtual void operator()( PoseEstimates* estimates ) = 0;
    };


    struct GridWeighting : Weighting
    {
        GridWeighting( L3::SE3 pose ) : pose(pose)
        {

        }
        L3::SE3 pose;
        void operator()( PoseEstimates* estimates ) ;
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
            virtual double operator()( const Histogram<T>& experience, const Histogram<T>& swathe )  = 0;

            virtual ~CostFunction()
            {

            }

        };

    template < typename T >  
        struct KLCostFunction : CostFunction<T>
        {
            double operator()( const Histogram<T>& experience, const Histogram<T>& swathe );
        };

    template < typename T >  
        struct MICostFunction : CostFunction<T>
        {
            double operator()( const Histogram<T>& experience, const Histogram<T>& swathe );
        };

    template < typename T >  
        struct NMICostFunction : CostFunction<T>
        {
            double operator()( const Histogram<T>& experience, const Histogram<T>& swathe );
        };


    template < typename T >  
        struct RenyiMICostFunction: CostFunction<T>
        {
            double operator()( const Histogram<T>& experience, const Histogram<T>& swathe );
        };

    template < typename T >  
        struct SSDCostFunction : CostFunction<T>
        {
            double operator()( const Histogram<T>& experience, const Histogram<T>& swathe );
        };

    template < typename T >  
        struct BattacharyaCostFunction : CostFunction<T>
        {
            double operator()( const Histogram<T>& experience, const Histogram<T>& swathe );
        };



    /*
     *Discrete hypothesis
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
        CostFunction<double>*           cost_function;
        __gnu_cxx::__normal_iterator<double*, std::vector<double, std::allocator<double> > > result_iterator ;

        /*
         *Changes
         */
        boost::shared_ptr< L3::Histogram<double> > swathe_histogram;
        boost::shared_ptr< L3::PointCloud<double> > hypothesis;

        void operator()();

    };



    /*
     * Estimator types
     */
    template < typename T >
        struct Estimator : Lockable
        {
            Estimator( boost::shared_ptr< L3::Histogram<double> > experience ) 
                : experience_histogram(experience)
            {
                // Allocations
                swathe_histogram.reset( new L3::HistogramUniformDistance<double>() );
                current_swathe.reset( new L3::PointCloud<double>() );
                current_histogram.reset( new L3::Histogram<double>() );
               
                // Allocate space for a sampled swathe
                sampled_swathe.reset( new PointCloud<T>() );
                L3::allocate( sampled_swathe.get(), 4*1000 );
            }
            
            CostFunction<T> *    cost_function;

            boost::shared_ptr< PoseEstimates >      pose_estimates;
            boost::shared_ptr<L3::Histogram<T> >    swathe_histogram;
            boost::shared_ptr<L3::Histogram<T> >    current_histogram;
            boost::shared_ptr<L3::Histogram<T> >    experience_histogram;
            boost::shared_ptr<L3::PointCloud<T> >   current_swathe;

            boost::shared_ptr< PointCloud<T> >      sampled_swathe;
            
            virtual ~Estimator()
            {
            }

            virtual bool operator()( PointCloud<T>* swathe, SE3 estimate ) = 0;

        };


    template < typename T >
        struct DiscreteEstimator : Estimator<T>
        {
            DiscreteEstimator( boost::shared_ptr< L3::Histogram<T> > experience, boost::shared_ptr< PoseEstimates > estimates )
                : Estimator<T>(experience)
            {
                this->pose_estimates = estimates;
            }
            
            tbb::task_group group;
            
            bool operator()( PointCloud<T>* swathe, SE3 estimate );

        };

    template < typename T>
        struct Algorithm : Lockable, PoseProvider
        {
            Algorithm( boost::shared_ptr< CostFunction<T> > cost_function, float fundamental_frequency=std::numeric_limits<float>::infinity() ) 
                : cost_function(cost_function), 
                fundamental_frequency(fundamental_frequency),
                current_prediction( new SE3() )
            {

            }
        
            boost::shared_ptr< SE3 > current_prediction;

            float fundamental_frequency;
            boost::shared_ptr< CostFunction<T> > cost_function;

            virtual std::string name() = 0;

            virtual SE3 operator()( PointCloud<T>* swathe, SE3 estimate ) = 0;
       
            L3::SE3 operator()()
            {
                return *current_prediction;
            }
        };

    template < typename T>
        struct PassThrough 
        {
            PassThrough(  Algorithm<T>* algorithm, boost::shared_ptr< L3::HistogramPyramid<T> > pyramid )
                : algorithm( algorithm ),
                pyramid(pyramid)
                  
            {
                sampled_swathe.reset( new PointCloud<T>() );
                L3::allocate( sampled_swathe.get(), 1000 );
           
                data.swathe_histogram.reset( new L3::Histogram<double>() );
                data.experience_histogram.reset( new L3::Histogram<double>() );
            }
            
            Algorithm<T>*                               algorithm;
            boost::shared_ptr< HistogramPyramid<T> >    pyramid;
            boost::shared_ptr< PointCloud<T> >          sampled_swathe;

            struct EstimateData : Lockable
            {
                boost::shared_ptr< L3::Histogram< double > > swathe_histogram;
                boost::shared_ptr< L3::Histogram< double > > experience_histogram;
            }data;


            SE3 operator()( PointCloud<T>* swathe, SE3 estimate );
            
        };

    
    template < typename T>
        struct IterativeDescent : Algorithm<T>
        {
            IterativeDescent( boost::shared_ptr< CostFunction<T> > cost_function, boost::shared_ptr< L3::HistogramPyramid<T> > pyramid ) 
                : Algorithm<T>(cost_function), 
                pyramid(pyramid)
            {

                float range = 2.0;
                float granularity = .5;

                int counter = 2;

                for( typename L3::HistogramPyramid<T>::PYRAMID_ITERATOR it = pyramid->begin();
                        it != pyramid->end();
                        it++ )
                {
                    L3::ReadLock lock( (*it)->mutex );
                           
                    // Grid 
                    discrete_estimators.push_back( 
                            boost::make_shared< DiscreteEstimator<T> >( *it, boost::make_shared< GridEstimates >( range, range, granularity) )
                            );


                    // Rotation
                    discrete_estimators.push_back( 
                            boost::make_shared< DiscreteEstimator<T> >( *it, boost::make_shared< RotationEstimates >() )
                            );


                    range /= 2.0;
                    granularity /= 1.8;
                    //granularity /= 1.5;
                    //granularity /= 2;
               
                    if (counter-- == 0 )
                        break;
                }

            }
            
            std::string name() 
            {
                return "IterativeDescent";
            }
            
            boost::shared_ptr< HistogramPyramid<T> > pyramid;
       
            std::deque< boost::shared_ptr< DiscreteEstimator<T> > > discrete_estimators;
            
            SE3 operator()( PointCloud<T>* swathe, SE3 estimate );

        };

    /*
     *  Minimisation 
     */
    double global_minimisation_function( const gsl_vector * x, void * params);

    struct MinimisationParameters
    {
        static Algorithm<double>* global_minimiser; 
    };

    template <typename T>
        struct Minimisation : Algorithm<T>
        {
            Minimisation( boost::shared_ptr< CostFunction<T> > cost_function, boost::shared_ptr< L3::HistogramPyramid<T> > experience_pyramid, int pyramid_index = 1, int max_iterations = 100, double tolerance = 0.075 ) 
                : Algorithm<T>(cost_function), 
                    pyramid(experience_pyramid),
                    max_iterations(max_iterations),
                    tolerance(tolerance),
                    pyramid_index(pyramid_index)
            {
                const gsl_multimin_fminimizer_type* type = gsl_multimin_fminimizer_nmsimplex2;
                //const gsl_multimin_fminimizer_type* type = gsl_multimin_fminimizer_nmsimplex2rand;
                
                minex_func.n = 3;
                minex_func.f = L3::Estimator::global_minimisation_function;

                x = gsl_vector_alloc (3);
                ss = gsl_vector_alloc (3);

                s = gsl_multimin_fminimizer_alloc (type, 3);

                MinimisationParameters::global_minimiser = this;
         
                timer.begin();
            }
     
            L3::Timing::ChronoTimer timer;

            int max_iterations, algorithm_iterations, pyramid_index;

            double tolerance;

            gsl_vector *ss, *x;
                
            gsl_multimin_function minex_func;
                   
            gsl_multimin_fminimizer *s;

            boost::shared_ptr< HistogramPyramid<T> > pyramid;
        
            PointCloud<T>* current_swathe;

            L3::SE3 predicted;
            std::vector< L3::SE3 > evaluations;
            
            CostFunction<T>* _cost_function;

            SE3 operator()( PointCloud<T>* swathe, SE3 estimate );
      
            double getHypothesisCost( const gsl_vector* hypothesis );

            std::string name()
            {
                return "Minimisation";
            }

        };

    template <typename T>
        struct Hybrid : Algorithm<T>
    {

        Hybrid( boost::shared_ptr< CostFunction<T> > cost_function, boost::shared_ptr< L3::HistogramPyramid<T> > pyramid ) 
            : Algorithm<T>(cost_function)
        {

            // Grid 
            discrete_estimators.push_back( 
                    boost::make_shared< DiscreteEstimator<T> >( (*pyramid)[0], boost::make_shared< GridEstimates >( 2, 2, .5) )
                    );

            // Rotation
            discrete_estimators.push_back( 
                    boost::make_shared< DiscreteEstimator<T> >( (*pyramid)[0], boost::make_shared< RotationEstimates >() )
                    );

            this->minimisation.reset( new Minimisation<T>( cost_function, pyramid ) );
        }



        boost::shared_ptr< Minimisation<T> >                    minimisation;
        std::deque< boost::shared_ptr< DiscreteEstimator<T> > > discrete_estimators;

        SE3 operator()( PointCloud<T>* swathe, SE3 estimate );


        std::string name()
        {
            return "Hybrid";
        }
    };

    double dlib_minimisation_function( const dlib::matrix<double,0,1>& input );

    struct DLIBParameters
    {
        static Algorithm<double>* global_minimiser; 
    };

    template <typename T>
        struct BFGS : Algorithm<T>
    {
        BFGS( boost::shared_ptr< CostFunction<T> > cost_function, boost::shared_ptr< L3::HistogramPyramid<T> > experience_pyramid ) 
            : Algorithm<T>(cost_function),
            pyramid(experience_pyramid),
            pyramid_index(2)
        {
            DLIBParameters::global_minimiser = this;
        }

        boost::shared_ptr< Minimisation<T> >                    minimisation;
        std::deque< boost::shared_ptr< DiscreteEstimator<T> > > discrete_estimators;
            
        int pyramid_index;
        std::vector< L3::SE3 > evaluations;

        PointCloud<T>* current_swathe;
            
        CostFunction<T>* _cost_function;
            
        boost::shared_ptr< HistogramPyramid<T> > pyramid;
        
        SE3 operator()( PointCloud<T>* swathe, SE3 estimate );
        
        double getHypothesisCost( const dlib::matrix<double,0,1>& x  );

        std::string name()
        {
            return "BFGS";
        }
    };

}
}

#endif
