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
        struct ParticleFilter : Filter<T>, Algorithm<T>
    {

        ParticleFilter(  CostFunction<T>* cost_function,  boost::shared_ptr< L3::HistogramPyramid<T> > experience_pyramid, boost::shared_ptr< L3::ConstantTimeIterator<L3::LHLV> > iterator, int num_particles = 1000 ) 
            : Filter<T>(iterator), 
            Algorithm<T>(cost_function), 
            previous_update(0.0), 
            pyramid( experience_pyramid ),
            initialised(false)
        {
            hypotheses.resize( num_particles );
        }
        
        bool initialised;

        int num_particles;

        double previous_update;


        // Generator
        boost::mt19937 rng;

        boost::shared_ptr< HistogramPyramid<T> > pyramid;

        std::vector< L3::SE3 > hypotheses;

        typedef std::vector< L3::SE3 >::iterator PARTICLE_ITERATOR; 

        // Search structure
        Comparator< std::pair< double, boost::shared_ptr<L3::LHLV> > > comparator;

        SE3 operator()( PointCloud<T>* swathe, SE3 estimate );

    };



}
}

#endif

