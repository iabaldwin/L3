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

    virtual void operator<< ( L3::Observer* observer )
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

struct ThreadedRunner : Runner, Poco::Runnable
{
    virtual void run() = 0;
};
    
struct DatasetRunner : ThreadedRunner
{
    DatasetRunner( const L3::Dataset* d ) : dataset(d), running(false), frequency(1.0)
    {
        // Constant time iterator over poses
        pose_iterator.reset( new L3::ConstantTimeIterator<L3::SE3>( dataset->pose_reader, 60.0 ) );
        LIDAR_iterator.reset( new L3::ConstantTimeIterator<L3::LMS151>( dataset->LIDAR_readers.back(), 10.0 ) );
        LHLV_iterator.reset( new L3::ConstantTimeIterator<L3::LHLV> ( dataset->LHLV_reader, 30.0 ) );  
    }

    std::auto_ptr<L3::ConstantTimeIterator< L3::SE3 > >     pose_iterator;
    std::auto_ptr<L3::ConstantTimeIterator< L3::LHLV> >     LHLV_iterator;
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

    void start( double start_time, bool threaded=true )
    {
        current_time = start_time;
        running = true;
        
        if (threaded)
            thread.start( *this );
    }

    void stop()
    {
        running = false;
    }

    void step( double dt )
    {
        current_time += dt;

        // Update Poses
        if ( !pose_iterator->update( current_time ) )
            throw std::exception();
    
        // Update LHLV
        if ( !LHLV_iterator->update( current_time ) )
            throw std::exception();

        // Update LIDAR
        if ( !LIDAR_iterator->update( current_time ))
            throw std::exception();
    
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

