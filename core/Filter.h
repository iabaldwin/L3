#ifndef L3_FILTER_H
#define L3_FILTER_H

#include "Estimator.h"
#include "Iterator.h"
#include <boost/random.hpp>

namespace L3
{
namespace Estimator
{

    template <typename T>
        struct Filter 
    {
        Filter( boost::shared_ptr< L3::ConstantTimeIterator<L3::LHLV> > iterator ) : iterator(iterator)
        {


        }
        
        boost::weak_ptr < L3::ConstantTimeIterator<L3::LHLV> >  iterator ;

    };




    template <typename T>
        struct ParticleFilter : Filter<T>, Algorithm<T>, L3::TemporalObserver
    {

        ParticleFilter(  CostFunction<T>* cost_function,  boost::shared_ptr< L3::HistogramPyramid<T> > experience_pyramid, boost::shared_ptr< L3::ConstantTimeIterator<L3::LHLV> > iterator, int num_particles = 500 ) 
            : Filter<T>(iterator), 
            Algorithm<T>(cost_function), 
            previous_time(0.0), 
            pyramid( experience_pyramid ),
            initialised(false),
            num_particles(num_particles)

        {
            hypotheses.resize( num_particles );
        
            sampled_swathe.reset( new PointCloud<T>() );
            L3::allocate( sampled_swathe.get(), 4*1000 );

        }
            
        boost::shared_ptr< PointCloud<T> > sampled_swathe;
        
        bool initialised;

        int num_particles;

        double previous_time, current_time;

        tbb::task_group group;

        // Generator
        boost::mt19937 rng;

        boost::shared_ptr< HistogramPyramid<T> > pyramid;

        std::vector< double > weights;
        std::vector< L3::SE3 > hypotheses;

        typedef std::vector< L3::SE3 >::iterator PARTICLE_ITERATOR; 

        // Search structure
        Comparator< std::pair< double, boost::shared_ptr<L3::LHLV> > > comparator;

        SE3 operator()( PointCloud<T>* swathe, SE3 estimate );

        bool update( double time);
    };



}
}

#endif

