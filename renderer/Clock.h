#ifndef L3_VISUAL_CLOCK_H
#define L3_VISUAL_CLOCK_H

#include <boost/timer.hpp>

namespace L3
{
namespace Visualisers
{

    struct Clock
    {

        Clock() : running(false), sf(1.0f), start_time(0.0)
        {
        }

        boost::timer t;

        bool running;
        float sf; 
        double start_time;

        void start( double start_time )
        {
            this->start_time = start_time;

            t.restart();

            running = true;
        }

        void stretch( float sf )
        {
            this->sf = sf;
        }

        void stop()
        {
            running = false;
        }

        double operator()()
        {
            return (t.elapsed()*sf)+start_time;;
        }

    };

}
}


#endif
