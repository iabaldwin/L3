#ifndef L3_SWATHE_BIULDER_H
#define L3_SWATHE_BIULDER_H

#include "Datatypes.h"
#include "Definitions.h"
#include "Core.h"

namespace L3
{

    class SwatheBuilder : public Observer
    {
        public:
            SwatheBuilder( L3::Iterator<L3::Pose>* poses, L3::Iterator<L3::LIDAR>* LIDARs ) :
                pose_window( poses ), LIDAR_window( LIDARs ) 
            {



            }

            L3::Tools::Timer t;
            void update( double time )
            {

                t.begin();
                pose_window->update( time ); 
                LIDAR_window->update( time ); 
                std::cout << t.end() << std::endl;

                /*
                 *  For each LIDAR scan, find the time
                 *  and associated pose
                 */
            
            }


            SWATHE swathe;
        
        private:

            L3::Iterator<L3::Pose>* pose_window;
            L3::Iterator<L3::LIDAR>* LIDAR_window;

    };

}

#endif

