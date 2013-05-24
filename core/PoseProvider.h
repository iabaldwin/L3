#ifndef L3_POSE_PROVIDER_H
#define L3_POSE_PROVIDER_H

#include <Eigen/LU>

#include <boost/timer.hpp>
#include <Poco/Thread.h>

#include "Iterator.h"
#include "ChainBuilder.h"

namespace L3
{

/*
 *  Provider mixin
 */
struct PoseProvider : std::unary_function< L3::SE3, void >
{
    virtual L3::SE3 operator()() 
    {
        return L3::SE3::ZERO();
    }
};

/*
 *  Circular pose provider (testing)
 */
struct CircularPoseProvider : PoseProvider, Poco::Runnable
{
    CircularPoseProvider( double frequency=10.0) : counter(0), 
                                                    x(0), y(0), 
                                                    angle(0.0), 
                                                    frequency(frequency),
                                                    running(true), 
                                                    update(false)
    {
        // Go
        thread.start( *this );
    }

    ~CircularPoseProvider()
    {
        running = false;
        thread.join();
    }

    Poco::Thread    thread;
    Poco::Mutex     mutex;
    int             counter;
    double          x, y, angle, range, frequency;
    bool            running, update;
     

    void run()
    {
        boost::timer t; 
        
        while ( running )
        {
            if ( t.elapsed() > 1.0/frequency )
            {
                t.restart();

                update = true;
            }
        }
    }

    /*
     *  Is single here, but really should be a chain
     */
    L3::SE3 operator()() 
    {
        L3::SE3 pose(x,y,0,0,0,0);
            
        range = 100;

        if (update)
        {
            angle+=( M_PI/180.0 )* 5;

            x = range*cos(angle);
            y = range*sin(angle);

            update = false;
        }

        return pose;
    }
};

}
#endif
