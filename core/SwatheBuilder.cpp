#include "SwatheBuilder.h"

namespace L3
{

    bool SwatheBuilder::update( double )
            {
                // Rebuild swathe completely
                swathe.clear();

                /*
                 *  For each LIDAR scan, find the time
                 *  and associated pose
                 */
                if ( LIDAR_iterator->window.empty() )
                    return false;

                L3::Iterator<L3::LMS151>::WINDOW_ITERATOR it = LIDAR_iterator->window.begin() ;

                // For each lidar scan, find the nearest pose
                while( it != LIDAR_iterator->window.end() )
                {
                    // Nearest time
                    L3::Iterator<L3::SE3>::WINDOW_ITERATOR index = std::lower_bound( pose_windower->window->begin(), 
                                                                                        pose_windower->window->end(), 
                                                                                        it->first, 
                                                                                        comparator );
                    
                    if ( index == pose_windower->window->end() ) 
                        break;
                    
                    if ((index->first - it->first) == 0)
                        swathe.push_back( std::make_pair( index->second, it->second ) );
                    //else
                        //{} 

                    it++;
                }

                window_duration = LIDAR_iterator->window.back().first - LIDAR_iterator->window.front().first;
               
                return true;
                    
            }



}
