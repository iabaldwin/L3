#ifndef L3_FILTER_H
#define L3_FILTER_H

#include <boost/random.hpp>

#include "Iterator.h"
#include "Estimator.h"
#include "PoseProvider.h"
#include "VelocityProvider.h"

#include "BayesFilter/unsFlt.hpp"

namespace L3
{

L3::SE3 operator/( const L3::SE3& lhs,  const double divisor );
L3::SE3 operator+( const L3::SE3& lhs,  const L3::SE3& rhs );
L3::SE3 operator*( const L3::SE3& lhs,  const double divisor );    
    
namespace Estimator
{
    template <typename T>
        struct Filter
    {
        Filter( boost::shared_ptr< L3::VelocityProvider > iterator ) 
            : iterator(iterator)
        {
        }

        L3::SE3 current_prediction;
        
        boost::weak_ptr < L3::VelocityProvider > iterator ;
        
    };

        
    struct PredictionModel : Bayesian_filter::Additive_predict_model, TemporalObserver
    {
        PredictionModel( boost::shared_ptr< L3::VelocityProvider > iterator ) ;
        
        mutable double last_update, current_update;

        boost::weak_ptr< L3::VelocityProvider > iterator ;

        boost::shared_ptr< Bayesian_filter_matrix::Vec > _x;
        
        mutable L3::SE3 prediction, delta;

        bool update( double time );

        const Bayesian_filter_matrix::Vec& f (const Bayesian_filter_matrix::Vec &x) const;
    };
    
    struct ObservationModel : Bayesian_filter::Uncorrelated_additive_observe_model 
    {
        ObservationModel ();
   
        const Bayesian_filter_matrix::Vec& h(const Bayesian_filter_matrix::Vec& x) const;
    };

    template <typename T>
        struct UKF : Filter<T>, Algorithm<T>, L3::TemporalObserver, PoseProvider
    {
        UKF( boost::shared_ptr<CostFunction<T> > cost_function,  
                boost::shared_ptr< L3::HistogramPyramid<T> > experience_pyramid, 
                boost::shared_ptr< L3::VelocityProvider > iterator );
                    
        boost::shared_ptr< Bayesian_filter::Unscented_scheme > ukf;

        boost::shared_ptr< Bayesian_filter_matrix::Vec >       x_init;
        boost::shared_ptr< Bayesian_filter_matrix::SymMatrix>  X_init;

        boost::shared_ptr< PredictionModel >    prediction_model;
        boost::shared_ptr< ObservationModel >   observation_model;
            
        L3::Timing::ChronoTimer timer;

        bool initialised;
        double previous_time, current_time;
       
        std::vector< double > sigma_points;

        boost::shared_ptr< Minimisation<T> > minimiser;

        SE3 current_estimate;

        SE3 operator()( PointCloud<T>* swathe, SE3 estimate );

        bool update( double time);
    
    };

    template <typename T>
        struct ParticleFilter : Filter<T>, Algorithm<T>, L3::TemporalObserver
    {
        ParticleFilter( boost::shared_ptr<CostFunction<T> > cost_function,  
                boost::shared_ptr< L3::HistogramPyramid<T> > experience_pyramid, 
                boost::shared_ptr< L3::VelocityProvider > iterator, 
                int num_particles = 800 ) 
            : Filter<T>(iterator), 
            Algorithm<T>(cost_function), 
            previous_time(0.0), 
            pyramid( experience_pyramid ),
            initialised(false),
            num_particles(num_particles)
        {
            sampled_swathe = boost::make_shared< PointCloud<T> >();

            L3::allocate( sampled_swathe.get(), 4*1000 );
       
            linear_uncertainty = 2;
            rotational_uncertainty = .2;
        }
            

        bool initialised;
        int num_particles;
        double previous_time, current_time;
        double linear_uncertainty, rotational_uncertainty;
        
        boost::mt19937 rng;
        tbb::task_group group;

        boost::shared_ptr< PointCloud<T> > sampled_swathe;
        boost::shared_ptr< HistogramPyramid<T> > pyramid;

        std::vector< double > weights;
        std::vector< L3::SE3 > hypotheses;

        typedef std::vector< L3::SE3 >::iterator PARTICLE_ITERATOR; 

        // Search structure
        Comparator< VELOCITY_WINDOW::value_type > comparator;

        SE3 operator()( PointCloud<T>* swathe, SE3 estimate );

        bool update( double time);
    };



}
}

#endif

