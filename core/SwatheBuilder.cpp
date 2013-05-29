#include "SwatheBuilder.h"

namespace L3
{

    bool SwatheBuilder::update( double )
    {
        // Rebuild swathe completely
        swathe.clear();

        t.begin();

        if( LIDAR_iterator->window.size() < pose_windower->window->size() )
        {
            L3::Iterator<L3::LMS151>::WINDOW_ITERATOR it = LIDAR_iterator->window.begin() ;

            /*
             *For each lidar scan, find the nearest pose
             */
            while( it != LIDAR_iterator->window.end() )
            {
                // Nearest time
                L3::Iterator<L3::SE3>::WINDOW_ITERATOR index = std::lower_bound( pose_windower->window->begin(), 
                        pose_windower->window->end(), 
                        it->first, 
                        pose_comparator );

                if ( index == pose_windower->window->end() ) 
                    break;

                if ((index->first - it->first) == 0)
                    swathe.push_back( std::make_pair( index->second, it->second ) );

                it++;
            }
        }
        else
        {
            L3::Iterator<L3::SE3>::WINDOW_ITERATOR it = pose_windower->window->begin() ;

            // For each lidar pose, find the nearest scan
            while( it != pose_windower->window->end() )
            {
                // Nearest time
                L3::Iterator<L3::LMS151>::WINDOW_ITERATOR index = std::lower_bound( LIDAR_iterator->window.begin(), 
                        LIDAR_iterator->window.end(), 
                        it->first, 
                        LIDAR_comparator );

                if ((index->first - it->first) == 0)
                    //swathe.push_back( std::make_pair( index->second, it->second ) );
                    swathe.push_back( std::make_pair( it->second, index->second ) );

                it++;
            }

            //window_duration = LIDAR_iterator->window.back().first - LIDAR_iterator->window.front().first;

        }

        return true;

    }

}
