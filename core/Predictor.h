#ifndef L3_PREDICTOR_H
#define L3_PREDICTOR_H

#include "Iterator.h"
#include "Integrator.h"
#include "Datatypes.h"

namespace L3
{

    class Predictor : public L3::TemporalObserver
    {

        public:

            Predictor( L3::Iterator<L3::LHLV>* iterator ) 
                : LHLV_iterator(iterator),
                    previous_update(0.0)
            {
            }

            bool update( double t );

            bool predict( const L3::SE3& current );

        protected:

            L3::Iterator<L3::LHLV>*      LHLV_iterator;

            double previous_update;

    };


    class ParticleFilter : public Predictor
    {

        public:
            ParticleFilter( L3::Iterator<L3::LHLV>* iterator, int num_particles = 100 ) 
                : Predictor(iterator), num_particles(num_particles)
            {
                particles.resize( num_particles ); 
            }

            int num_particles;

            std::vector< L3::SE3 > particles;

            bool predict( const L3::SE3& current );

            bool update( double t );
            
    };

}

#endif
