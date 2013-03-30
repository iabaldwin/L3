#ifndef L3_DATASET_RUNNER_H
#define L3_DATASET_RUNNER_H

#include <Poco/Runnable.h>
#include <Poco/Thread.h>

#include <boost/timer.hpp>

#include "Core.h"
#include "Iterator.h"
#include "Dataset.h"

namespace L3
{

struct Runner
{
    virtual void operator << ( L3::Observer* observer )
    {
        observables.push_front( observer ); 
    }

    std::list< L3::Observer* > observables;
};

struct TemporalRunner : Runner, TemporalObserver
{
    virtual bool update( double t )
    {
        for( std::list< L3::Observer* >::iterator it = observables.begin(); 
                it != observables.end();
                it++ )
            dynamic_cast<L3::TemporalObserver*>( *it )->update( t );
    }

};

struct ThreadedTemporalRunner : TemporalRunner, Poco::Runnable
{
    Poco::Thread thread;

    bool running;

    ThreadedTemporalRunner() : running( true )
    {
    }

    ~ThreadedTemporalRunner()
    {
        running = false;
        if ( thread.isRunning() )
            thread.join();
    }

    void start( double start_time, bool threaded=true )
    {
        thread.start( *this );
    }

    virtual void run()
    {
        while( running )
        {

        }
    }
};
    
struct DatasetRunner : ThreadedTemporalRunner
{
    DatasetRunner( const L3::Dataset* d ) : dataset(d), running(true), frequency(1.0)
    {
        // Constant time iterator over poses
        pose_iterator.reset( new L3::ConstantTimeIterator<L3::SE3>( dataset->pose_reader ) );
        LIDAR_iterator.reset( new L3::ConstantTimeIterator<L3::LMS151>( dataset->LIDAR_readers.back() ) );
        LHLV_iterator.reset( new L3::ConstantTimeIterator<L3::LHLV> ( dataset->LHLV_reader ) );  
    
   
        (*this)<< &*pose_iterator;
        (*this)<< &*LIDAR_iterator;
        (*this)<< &*LHLV_iterator;
    
    }

    std::auto_ptr<L3::ConstantTimeIterator< L3::SE3 > >     pose_iterator;
    std::auto_ptr<L3::ConstantTimeIterator< L3::LHLV > >    LHLV_iterator;
    std::auto_ptr<L3::ConstantTimeIterator< L3::LMS151 > >  LIDAR_iterator;

    Poco::Thread    thread;
    const Dataset*  dataset;
    bool            running;
    float           frequency;
    double          current_time; 

    ~DatasetRunner()
    {
        stop();
      
        if ( thread.isRunning() )
            thread.join();
    }

    void start( double start_time )
    {
        current_time = start_time;
        thread.start( *this );
    }

    void stop()
    {
        running = false;
    }

    void step( double dt )
    {
        current_time += dt;
           
        this->update( current_time );
    }

    void run()
    {
        boost::timer t;

        while( running )
        {
            if ( t.elapsed() > 1.0/frequency )
            {
                step( .5 );

                // Restart timer
                t.restart();
            }
        }

    }
};

} // L3
#endif

