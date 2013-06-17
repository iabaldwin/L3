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
#include "ScanMatching.h"
#include "Predictor.h"
#include "VelocityProvider.h"

#include <Poco/Task.h>
#include <Poco/TaskManager.h>

namespace L3
{

struct TemporalRunner 
{
    std::list < TemporalObserver* > observers;

    virtual bool update( double time )
    {
        std::for_each( observers.begin(), observers.end(), std::bind2nd( std::mem_fun( &TemporalObserver::update ), time ) );
   
        return true;
    }

};


/*
 *  Threaded runner
 */
struct ThreadedRunner : TemporalRunner, Poco::Runnable
{
    ThreadedRunner() 
        : running( true ),
        paused(false),
        current_time(0.0)
    {
    }

    Poco::Thread    thread;
    bool            running, paused;
    double          start_time, current_time;


    void stop()
    {
        running = false;
    }

    void start()
    {
        thread.start( *this );
    }
    
    };

struct DatasetRunner : ThreadedRunner
{
    DatasetRunner( L3::Dataset* dataset, L3::Configuration::Mission* mission, float speedup=5.0 );

    ~DatasetRunner()
    {
        running = false;

        if ( thread.isRunning() )
            thread.join();
    }

    std::ofstream   output;

    Configuration::Mission*     mission;
    Dataset*                    dataset;
    
    float           speedup;
    double          current_time, start_time;  
    bool            stand_alone;
   
    std::list < Updater* > updaters;
    
    boost::shared_ptr< L3::SE3 > current;
    

    boost::shared_ptr< L3::SE3 >                projection;
    boost::shared_ptr< L3::Projector<double> >  projector;
    boost::shared_ptr< L3::PointCloud<double> > point_cloud;
    boost::shared_ptr< L3::SwatheBuilder >      swathe_builder;
    boost::shared_ptr< L3::Predictor >          predictor;

    boost::shared_ptr< L3::PoseProvider >       provider;
    boost::shared_ptr< L3::PoseWindower >       pose_windower;
    
    boost::shared_ptr< L3::VelocityProvider >   lhlv_velocity_provider;
    boost::shared_ptr< L3::VelocityProvider >   icp_velocity_provider;
    boost::shared_ptr< L3::VelocityProvider >   ics_velocity_provider;


    boost::shared_ptr< L3::ConstantTimeWindower< L3::SE3 > >    oracle;
    boost::shared_ptr< L3::ConstantTimeIterator< L3::SE3 > >    pose_iterator;
    boost::shared_ptr< L3::ConstantTimeIterator< L3::LHLV > >   LHLV_iterator;
    boost::shared_ptr< L3::ConstantTimeIterator< L3::LMS151 > > vertical_LIDAR;
    boost::shared_ptr< L3::ConstantTimeIterator< L3::LMS151 > > horizontal_LIDAR;
    
    boost::shared_ptr< L3::ConstantTimeIterator< L3::SMVelocity > >    velocity_source;
     
    boost::shared_ptr< L3::ScanMatching::Engine > engine;
        
    std::vector<double> timings;
        
    void run();
    
    virtual bool update( double time )
    {
        return true;
    }

    DatasetRunner& operator<<( L3::TemporalObserver* observer)
    {
        if ( !observer )
        {
            std::cout << "Erroneouse observer passed!" << std::endl;
            exit(-1);
        }
        observers.push_back( observer ); 
        return *this;
    }

    DatasetRunner& operator<<( L3::Updater* updater )
    {
        if ( !updater)
        {
            std::cout << "Erroneouse updater passed!" << std::endl;
            exit(-1);
        }
        updaters.push_back( updater ); 
        return *this;
    }


};

/*
 *  Implementation specific
 */
struct EstimatorRunner : DatasetRunner, Lockable
{
    EstimatorRunner( L3::Dataset* dataset, L3::Configuration::Mission* mission, L3::Experience* experience, float speedup=5.0 ) 
        : DatasetRunner( dataset, mission, speedup ),
            experience(experience)
    {
    }


    ~EstimatorRunner()
    {
        running = false;

        if ( thread.isRunning() )
            thread.join();
    }
    
    L3::Experience*                                         experience;
    boost::shared_ptr< L3::Estimator::Algorithm<double> >   algorithm;

    bool update( double time );

    EstimatorRunner& setAlgorithm( boost::shared_ptr< L3::Estimator::Algorithm<double> > algo )
    {
        // Remove it if it exista
        std::list< TemporalObserver* >::iterator it = std::find( observers.begin(), observers.end(), dynamic_cast< L3::TemporalObserver* >( this->algorithm.get() ) ); 
        if(  it != observers.end() )
            observers.erase( it );

        this->algorithm = algo;
     
        // Is it updateable
        if( L3::TemporalObserver* observer = dynamic_cast< L3::TemporalObserver* >( algorithm.get() ) )
            (*this) << observer;

        return *this;
    }
};

} // L3
#endif

