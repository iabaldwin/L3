#ifndef L3_POSE_WINDOWER_H
#define L3_POSE_WINDOWER_H

#include "Datatypes.h"
#include "PoseProvider.h"

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
                L3::WriteLock( this->mutex );
                return constant_time_iterator->update(t);
            }

            L3::SE3 operator()( void )
            {
                L3::ReadLock( this->mutex );

                if ( this->constant_time_iterator->window.size() > 0 )
                    return *(this->constant_time_iterator->window.back().second);
                else
                    return L3::SE3::ZERO();
            }
    };

    /*
     *  LHLV Windower
     */
    template <>
        class ConstantTimeWindower<L3::LHLV> : public PoseWindower
        {
            public:

                ConstantTimeWindower( L3::ConstantTimeIterator<L3::LHLV>* iterator ) 
                    : constant_time_iterator(iterator)
                {
                    // Poses from velocity
                    chain_builder.reset( new L3::ChainBuilder( iterator ) );

                    // Reflection
                    this->window = &chain_builder->window;
                }

                // Iterator base
                L3::ConstantTimeIterator<L3::LHLV>* constant_time_iterator;

                // Chain builder - poses from velocity
                boost::shared_ptr< L3::ChainBuilder > chain_builder;

                // Trajectory inverter
                Inverter inverter;

                bool update( double t)
                {
                    // Update the chain
                    chain_builder->update();

                    // Reorient poses
                    return inverter.invert( chain_builder->window.begin(), chain_builder->window.end() );
                }

        };

    /*
     *  Constant distance windowers
     */
    class ConstantDistanceWindower : public PoseWindower
    {
        public:

            ConstantDistanceWindower( L3::ConstantTimeIterator<L3::LHLV>* iterator, double swathe_length = 10.0  ) 
                : constant_time_iterator(iterator),
                    swathe_length(swathe_length),
                    previous_update(0.0)
            {
                // Reflection
                this->window = &_constant_distance_window;
            }


            // Iterator base
            L3::ConstantTimeIterator<L3::LHLV>* constant_time_iterator;

            // Core window
            std::deque< std::pair< double, boost::shared_ptr<L3::SE3> > > _constant_distance_window;

            // Window buffer
            std::deque< std::pair< double, boost::shared_ptr<L3::LHLV> > > _window_buffer;
            
            // Window buffer::Incremental distances
            std::deque < double > _window_incremental_distances;
           
            // Sink
            std::deque < double > _sink;

            // Pose inverter
            Inverter inverter;
        
            // Search structure
            Comparator< std::pair< double, boost::shared_ptr<L3::LHLV> > > comparator;

            double swathe_length;

            double previous_update;

            bool update( double time );
    
    };

}

#endif

