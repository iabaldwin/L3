#ifndef L3_PERFORMANCE_H
#define L3_PERFORMANCE_H

#include "Core.h"
#include "Timing.h"
#include "Experience.h"

namespace L3
{
    struct RelativeDisplacement : Updateable
    {
        RelativeDisplacement( Experience* experience, boost::shared_ptr< SE3 > pose, float update_frequency = 2  ) :
            pose( pose ),
            experience(experience),
            update_frequency( update_frequency ),
            previous_update( 0.0 ),
            displacement(0)
        {
            timer.begin();
        }

        L3::Timing::ChronoTimer timer;
        double previous_update, update_frequency, displacement;

        Experience* experience;
        boost::weak_ptr< SE3 >pose;

        void update();
    };

}




#endif

