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

namespace L3
{

struct Runner
{
    virtual Runner& operator << ( L3::Observer* observer )
    {
        observables.push_front( observer ); 
        return (*this);
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
   
        return postUpdate();
    }

    virtual bool postUpdate()
    {
        return true;
    }
};

struct ThreadedTemporalRunner : TemporalRunner, Poco::Runnable
{

    ThreadedTemporalRunner() : running( true )
    {
    }

    Poco::Thread    thread;
    bool            running;

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

/*
 *  Implementation specific
 */
struct EstimatorRunner : TemporalRunner
{

    L3::Experience*                     experience;
    L3::PoseProvider*                   provider;
    L3::Projector<double>*              projector;
    L3::Estimator::Estimator<double>*   estimator;
    SWATHE*                             swathe;

    /*
     *    All time-sensitive iterables have been
     *    updated
     *
     */

    bool postUpdate()
    {
#ifndef  NDEBUG
        boost::timer t;
#endif

        boost::shared_ptr<L3::PointCloud<double> > experience_cloud;
        
        L3::SE3 pose = (*provider)();
        experience->update( pose.x, pose.y );
        experience->getExperienceCloud( experience_cloud );

#ifndef  NDEBUG
        std::cout << "Experience\t" << t.elapsed() << std::endl;
#endif
        projector->project( *swathe );

#ifndef  NDEBUG
        std::cout << "Projection\t" << t.elapsed() << std::endl;
#endif

        (*estimator)( &*experience_cloud, projector->cloud, L3::SE3::ZERO() );

#ifndef  NDEBUG
        std::cout << "Estimation\t" << t.elapsed() << std::endl;
#endif
    
    }

    bool setPoseProvider( L3::PoseProvider* provider )
    {
        this->provider = provider;
    }

    bool setExperience( L3::Experience* experience )
    {
        this->experience = experience;
    }

    bool setProjector( L3::Projector<double>* projector )
    {
        this->projector = projector;
    }

    bool setEstimator( L3::Estimator::Estimator<double>* estimator )
    {
        this->estimator = estimator;
    }

    bool setSwathe( SWATHE* swathe )
    {
        this->swathe = swathe;
    }

};


struct DatasetRunner : ThreadedTemporalRunner
{
    DatasetRunner( const L3::Dataset* d ) : dataset(d), running(true), frequency(1.0)
    {
        // Constant time iterator over poses
        pose_iterator.reset( new L3::ConstantTimeIterator<L3::SE3>( dataset->pose_reader ) );
        LIDAR_iterator.reset( new L3::ConstantTimeIterator<L3::LMS151>( dataset->LIDAR_readers.begin()->second ) );
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

