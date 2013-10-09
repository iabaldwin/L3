#ifndef L3_POSE_WINDOWER_H
#define L3_POSE_WINDOWER_H

#include "Datatypes.h"
#include "PoseProvider.h"
#include "VelocityProvider.h"

namespace L3
{

    /*
     *  Pose Windower
     */
    struct PoseWindower : PoseProvider, TemporalObserver
    {
        std::deque< std::pair< double, boost::shared_ptr<L3::SE3> > >* window;
    };

    /*
     *  Constant time INS windower
     */

    struct Inverter
    {
        std::deque< std::pair< double, boost::shared_ptr<L3::SE3> > > chain;
        
        template <typename InputIterator >
            bool invert( InputIterator start, InputIterator end );
    };


    template <typename T>
        class ConstantTimeWindower : public PoseWindower, Lockable
    {
        public:

            ConstantTimeWindower( L3::ConstantTimeIterator<T>* iterator ) 
                : constant_time_iterator (iterator)
            {
                this->window = &(iterator->window);
            }

            Inverter inverter;
            L3::ConstantTimeIterator<T>* constant_time_iterator;

            bool update( double t)
            {
                L3::WriteLock lock( this->mutex );
                return constant_time_iterator->update(t);
            }

            L3::SE3 operator()( void )
            {
                L3::ReadLock lock( this->mutex );

                if ( !this->constant_time_iterator->window.empty() )
                    return *(this->constant_time_iterator->window.back().second);
                else
                    return L3::SE3::ZERO();
            }
    };

    /*
     *  Constant distance windowers
     */
    class ConstantDistanceWindower : public PoseWindower
    {
        public:

            ConstantDistanceWindower( L3::VelocityProvider* provider, double swathe_length = 10.0  ) 
                : velocity_provider(provider),
                    swathe_length(swathe_length),
                    previous_update(0.0)
            {
                // Reflection
                this->window = &_constant_distance_window;
            }

            // Provider base
            L3::VelocityProvider* velocity_provider;

            // Core window
            std::deque< std::pair< double, boost::shared_ptr<L3::SE3> > > _constant_distance_window;

            // Window buffer
            VELOCITY_WINDOW _window_buffer;
            
            // Search structure
            Comparator< std::pair< double, std::vector< double > > > comparator;

            double swathe_length;

            double previous_update;

            bool update( double time );
    
    };

}

#endif

