#ifndef L3_DATASET_RUNNER_H
#define L3_DATASET_RUNNER_H

#include <Poco/Runnable.h>
#include <Poco/Thread.h>

#include <boost/timer.hpp>

#include "Core.h"
#include "Iterator.h"
#include "Dataset.h"
#include "Experience.h"
#include "Estimator.h"
#include "SwatheBuilder.h"
#include "Predictor.h"
#include "ScanMatching.h"

namespace L3
{

/*
 *  Runner
 */
struct Runner
{
    virtual Runner& operator << ( L3::Observer* observer )
    {
        observables.push_front( observer ); 
        return (*this);
    }

    std::list< L3::Observer* > observables;
};

/*
 *  Time-based runner
 */
struct TemporalRunner : Runner, TemporalObserver
{
    virtual bool update( double t )
    {
        for( std::list< L3::Observer* >::iterator it = observables.begin(); 
                it != observables.end();
                it++ )
            
            // Dangerous, fast
            static_cast<L3::TemporalObserver*>( *it )->update( t);
            // Safe, slow 
            //L3::TemporalObserver* observer = dynamic_cast<L3::TemporalObserver*>( *it );
    }

};

/*
 *  Threaded runner
 */
struct ThreadedTemporalRunner : TemporalRunner, Poco::Runnable
{

    ThreadedTemporalRunner() : running( true )
    {
    }

    Poco::Thread    thread;
    bool            running;
    double          current_time;

    ~ThreadedTemporalRunner()
    {
        running = false;
        if ( thread.isRunning() )
            thread.join();
    }

    void start( double start_time )
    {
        current_time = start_time;
        thread.start( *this );
    }

    virtual void run()
    {
        while( running )
        {

        }
    }
};

/*
 *  Implementation specific
 */
struct EstimatorRunner : ThreadedTemporalRunner
{

    L3::Experience*                         experience;
    L3::PoseProvider*                       provider;
    L3::ConstantTimeWindower<L3::LHLV>*     windower;
    L3::SwatheBuilder*                      swathe_builder;
    L3::Projector<double>*                  projector;
    //L3::Estimator::Estimator<double>*       estimator;
    L3::Estimator::Algorithm<double>*       estimator;

    std::list < Dumpable* >  dumps;
    std::list < TemporalObserver* >         observers;
    
    L3::ConstantTimeIterator< L3::LMS151 >*     vertical_LIDAR;
    L3::ConstantTimeIterator< L3::LMS151 >*     horizontal_LIDAR;
  
    boost::shared_ptr< L3::ScanMatching::ScanMatcher >  scan_matcher;

    boost::shared_ptr< L3::SE3 > current;

    float speedup;

    EstimatorRunner( float speedup=5.0) : speedup(speedup)
    {
        current.reset( new L3::SE3( L3::SE3::ZERO() ) );
        scan_matcher.reset( new L3::ScanMatching::ICP() );
    }

    void run();
    bool update( double time );

    EstimatorRunner& setPoseWindower( L3::ConstantTimeWindower<L3::LHLV>* windower )
    {
        this->windower = windower;
        (*this) << dynamic_cast<L3::TemporalObserver*>(windower);
        (*this) << dynamic_cast<L3::Dumpable*>(windower);
        return *this;
    }

    
    EstimatorRunner& setPoseProvider( L3::PoseProvider* provider )
    {
        this->provider = provider;
        (*this) << dynamic_cast<L3::TemporalObserver*>(provider);
        (*this) << dynamic_cast<L3::Dumpable*>(provider);
        return *this;
    }

    EstimatorRunner& setSwatheBuilder( L3::SwatheBuilder* swathe_builder )
    {
        // Not temporally updateable
        this->swathe_builder = swathe_builder;
        (*this) << dynamic_cast<L3::Dumpable*>(swathe_builder);
        return *this;
    }

    EstimatorRunner& setExperience( L3::Experience* experience )
    {
        // Spatially updateable
        this->experience = experience;
        (*this) << dynamic_cast<L3::Dumpable*>(experience);
        return *this;
    }

    EstimatorRunner& setProjector( L3::Projector<double>* projector )
    {
        // Not temporally updateable
        this->projector = projector;
        return *this;
    }

    //EstimatorRunner& setEstimator( L3::Estimator::Estimator<double>* estimator )
    EstimatorRunner& setAlgorithm( L3::Estimator::Algorithm<double>* estimator )
    {
        this->estimator = estimator;
        return *this;
    }

    EstimatorRunner& setHorizontalLIDAR( L3::ConstantTimeIterator< L3::LMS151 >* windower )
    {
        this->horizontal_LIDAR = windower;
        (*this) << dynamic_cast<L3::Dumpable*>(windower);
        return *this;
    }

    EstimatorRunner& setVerticalLIDAR( L3::ConstantTimeIterator< L3::LMS151 >* windower )
    {
        this->vertical_LIDAR = windower;
        (*this) << dynamic_cast<L3::Dumpable*>(windower);
        return *this;
    }

    EstimatorRunner& operator<<( L3::TemporalObserver* observer )
    {
        if ( observer )
            observers.push_front( observer ); 
        return *this;
    }

    EstimatorRunner& operator<<( L3::Dumpable* dumpable)
    {
        if ( dumpable )
            dumps.push_front( dumpable ); 
        return *this;
    }

};


struct DatasetRunner : ThreadedTemporalRunner, Lockable
{
    DatasetRunner( const L3::Dataset* d ) 
        : dataset(d), 
            running(true), 
            frequency(10.0)
    {
        // Constant time iterator over poses
        pose_iterator.reset( new L3::ConstantTimeIterator<L3::SE3>( dataset->pose_reader ) );
        LIDAR_iterator.reset( new L3::ConstantTimeIterator<L3::LMS151>( dataset->LIDAR_readers.begin()->second ) );
        LHLV_iterator.reset( new L3::ConstantTimeIterator<L3::LHLV> ( dataset->LHLV_reader ) );  
   
        (*this)<< pose_iterator.get() << LIDAR_iterator.get() << LHLV_iterator.get();
    
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
                L3::WriteLock( this->mutex );
                
                step( .5 );

                // Restart timer
                t.restart();
            }
        }

    }
};

} // L3
#endif

