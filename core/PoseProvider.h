#ifndef L3_POSE_PROVIDER_H
#define L3_POSE_PROVIDER_H

#include <boost/timer.hpp>
#include <Poco/Thread.h>

#include "Iterator.h"
#include "ChainBuilder.h"

namespace L3
{

struct PoseProvider : std::unary_function< L3::SE3, void >
{
    virtual L3::SE3 operator()() 
    {
        return L3::SE3::ZERO();
    }
};

/*
 *Circular pose provider (testing)
 */
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
template <typename T>
class ConstantTimeWindower : public PoseWindower
{
    public:
        
        ConstantTimeWindower( L3::ConstantTimeIterator<T>* iterator ) 
            : constant_time_iterator (iterator)
        {
            this->window = &(iterator->window);
        }
        
        L3::ConstantTimeIterator<T>* constant_time_iterator;

        bool update( double t)
        {
            constant_time_iterator->update(t);
        }

        L3::SE3 operator()( void )
        {
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

        boost::shared_ptr< L3::ChainBuilder > chain_builder;

        bool update( double t)
        {
            chain_builder->update(t);
        }

};




}
#endif
