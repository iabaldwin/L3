#ifndef L3_FILTER_H
#define L3_FILTER_H

#include "Estimator.h"
#include "Iterator.h"
#include <boost/random.hpp>

namespace L3
{

L3::SE3 operator/( const L3::SE3& lhs,  const double divisor );
L3::SE3 operator+( const L3::SE3& lhs,  const L3::SE3& rhs );
L3::SE3 operator*( const L3::SE3& lhs,  const double divisor );    
    
namespace Estimator
{

    struct AlphaBetaFilter
    {

        struct state
        {
            double x,v; 

            state& operator=( const state& lhs )
            {
                this->x = lhs.x;
                this->v = lhs.v;

                return *this;
            }

            
        } _state;
            
        AlphaBetaFilter( float alpha, float beta, double x=0.0, double v=0.0) 
            : alpha(alpha), beta(beta),
                previous_update(0.0)
        {
            _state.x = x;
            _state.v = v;
        }

        float alpha, beta;
        double previous_update;

        void update( double time, double measurement );
        

    };

        
    std::ostream& operator<<( std::ostream& out, const AlphaBetaFilter::state& state );


    template <typename T>
        struct Filter
    {
        Filter( boost::shared_ptr< L3::ConstantTimeIterator<L3::LHLV> > iterator ) : iterator(iterator)
        {


        }

        L3::SE3 current_prediction;
        boost::weak_ptr < L3::ConstantTimeIterator<L3::LHLV> >  iterator ;

    };

    template <typename T>
        struct ParticleFilter : Filter<T>, Algorithm<T>, L3::TemporalObserver
    {
        ParticleFilter( boost::shared_ptr<CostFunction<T> > cost_function,  
                        boost::shared_ptr< L3::HistogramPyramid<T> > experience_pyramid, 
                        boost::shared_ptr< L3::ConstantTimeIterator<L3::LHLV> > iterator, 
                        int num_particles = 450 ) 
            : Filter<T>(iterator), 
                Algorithm<T>(cost_function), 
                previous_time(0.0), 
                pyramid( experience_pyramid ),
                initialised(false),
                num_particles(num_particles)

        {
            hypotheses.resize( num_particles );
        
            sampled_swathe = boost::make_shared< PointCloud<T> >();

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

