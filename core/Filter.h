#ifndef L3_FILTER_H
#define L3_FILTER_H

#include <boost/random.hpp>

#include "Iterator.h"
#include "Estimator.h"
#include "VelocityProvider.h"

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
        Filter( boost::shared_ptr< L3::VelocityProvider > iterator ) : iterator(iterator)
        {


        }

        L3::SE3 current_prediction;
        boost::weak_ptr < L3::VelocityProvider >  iterator ;

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
        
        boost::shared_ptr< PointCloud<T> > sampled_swathe;

        tbb::task_group group;

        // Generator
        boost::mt19937 rng;

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

