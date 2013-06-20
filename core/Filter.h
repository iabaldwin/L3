#ifndef L3_FILTER_H
#define L3_FILTER_H

#include <boost/random.hpp>

#include "Iterator.h"
#include "Estimator.h"
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
        
        boost::weak_ptr < L3::VelocityProvider >  iterator ;
        
    };

        
    struct PredictionModel : Bayesian_filter::Unscented_predict_model, TemporalObserver
    {
        PredictionModel( boost::shared_ptr< L3::VelocityProvider > iterator ) 
            : Bayesian_filter::Unscented_predict_model(3),
            iterator(iterator),
            last_update(0.0),
            current_update(0.0)
        {
            _Q.reset( new Bayesian_filter_matrix::SymMatrix(3,3) );
            _x.reset( new Bayesian_filter_matrix::Vec(3) );
            Fx.reset( new Bayesian_filter_matrix::Vec(3) );
        }
        
        mutable double last_update, current_update;

        boost::weak_ptr< L3::VelocityProvider > iterator ;

        boost::shared_ptr< Bayesian_filter_matrix::Vec > Fx;
        boost::shared_ptr< Bayesian_filter_matrix::Vec > _x;
        boost::shared_ptr< Bayesian_filter_matrix::SymMatrix > _Q;

        bool update( double time )
        {
            current_update = time;
        }

        const Bayesian_filter_matrix::Vec& f (const Bayesian_filter_matrix::Vec &x) const;
       
        const Bayesian_filter_matrix::SymMatrix& Q(const Bayesian_filter_matrix::Vec& x) const
        {
            return *_Q;
        }
    };
    
    struct ObservationModel : Bayesian_filter::Linear_uncorrelated_observe_model
    {
        ObservationModel ();
    };


    template <typename T>
        struct EKF : Filter<T>, Algorithm<T>, L3::TemporalObserver
    {
        EKF( boost::shared_ptr<CostFunction<T> > cost_function,  
                boost::shared_ptr< L3::HistogramPyramid<T> > experience_pyramid, 
                boost::shared_ptr< L3::VelocityProvider > iterator );
                    
        boost::shared_ptr< Bayesian_filter::Unscented_scheme > ukf;

        boost::shared_ptr< Bayesian_filter_matrix::Vec >       x_init;
        boost::shared_ptr< Bayesian_filter_matrix::SymMatrix>  X_init;

        boost::shared_ptr< PredictionModel >    prediction_model;
        boost::shared_ptr< ObservationModel >   observation_model;

        bool initialised;
        double previous_time, current_time;

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

