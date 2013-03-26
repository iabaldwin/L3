#ifndef L3_SWATHE_BIULDER_H
#define L3_SWATHE_BIULDER_H

#include "Datatypes.h"
#include "Definitions.h"
#include "Core.h"
#include "PoseProvider.h"

namespace L3
{
    class SwatheBuilder : public TemporalObserver
    {
        public:
            //SwatheBuilder( L3::Iterator<L3::SE3> * pose_it, L3::Iterator<L3::LMS151>* LIDAR_it ) :
            SwatheBuilder( L3::PoseWindower* pose_it, L3::Iterator<L3::LMS151>* LIDAR_it ) :
                pose_windower( pose_it ), 
                LIDAR_iterator( LIDAR_it ), 
                window_duration(0.0) 
            {
            }
                
            Comparator< std::pair< double, boost::shared_ptr<L3::SE3> > > _comparator;

            bool update( double time )
            {
#ifndef NDEBUG
                L3::Tools::Timer t;
                t.begin();
#endif
                if (!pose_windower->update( time ))
                    throw POSE_end();

                if (!LIDAR_iterator->update( time ))
                    throw LIDAR_end(); 

#ifndef NDEBUG
#endif
                /*
                 *  For each LIDAR scan, find the time
                 *  and associated pose
                 */
         
                L3::Iterator<L3::LMS151>::WINDOW_ITERATOR it = LIDAR_iterator->window.begin() ;

                swathe.clear();

                // For each lidar scan, find the nearest pose
                while( it != LIDAR_iterator->window.end() )
                {
                    // Nearest time
                    L3::Iterator<L3::SE3>::WINDOW_ITERATOR index = std::lower_bound( pose_windower->window->begin(), 
                                                                                        pose_windower->window->end(), 
                                                                                        it->first, 
                                                                                        _comparator );
                    
                    if ( index == pose_windower->window->end() ) 
                        index--; 

                    //assert( index-

                    swathe.push_back( std::make_pair( index->second, it->second ) );

                    it++;
                }

                window_duration = LIDAR_iterator->window.back().first - LIDAR_iterator->window.front().first;
               
                return true;
                    
            }

            SWATHE swathe;

        private:

            L3::PoseWindower*            pose_windower;
            L3::Iterator<L3::LMS151>*   LIDAR_iterator;

            double window_duration;
    };

}

#endif

