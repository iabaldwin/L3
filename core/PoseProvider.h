#ifndef L3_POSE_PROVIDER_H
#define L3_POSE_PROVIDER_H

#include <boost/timer.hpp>

namespace L3
{

struct PoseProvider : std::unary_function< L3::SE3, void >
{
    virtual L3::SE3 operator()() = 0;
};


struct CircularPoseProvider : PoseProvider, Poco::Runnable
{
    CircularPoseProvider() : counter(0), x(0), y(0), angle(0.0), running(true), frequency(10), update(false)
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
    bool            running, update;
    double          x, y, range, angle, frequency;
     

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

    L3::SE3 operator()() 
    {
        L3::SE3 pose(x,y,0,0,0,0);

        if (update)
        {
            angle+=( M_PI/180.0 )* 5;

            range = 100;

            x = range*cos(angle);
            y = range*sin(angle);

            update = false;
        }

        return pose;
    }
};

}
#endif
