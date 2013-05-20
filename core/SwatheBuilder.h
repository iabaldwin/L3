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
            SwatheBuilder( L3::PoseWindower* pose_it, L3::Iterator<L3::LMS151>* LIDAR_it ) :
                pose_windower( pose_it ), 
                LIDAR_iterator( LIDAR_it ), 
                window_duration(0.0) 
            {
            }
                
            Comparator< std::pair< double, boost::shared_ptr<L3::SE3> > > comparator;

            SWATHE swathe;

            bool update( double );
            

        private:

            L3::PoseWindower*           pose_windower;
            L3::Iterator<L3::LMS151>*   LIDAR_iterator;

            double window_duration;
    };

}

#endif

