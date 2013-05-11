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
    }
};


/*
 *  Threaded runner
 */
struct ThreadedRunner : TemporalRunner, Poco::Runnable
{
    ThreadedRunner() 
        : running( true ),
            current_time(0.0)
    {
    }

    Poco::Thread    thread;
    bool            running;
    double          start_time, current_time;

    void start()
    {
        thread.start( *this );
    }
    
    };

struct DatasetRunner : ThreadedRunner
{
    DatasetRunner( L3::Dataset* dataset, L3::Configuration::Mission* mission, float speedup=5.0 ) 
        : dataset(dataset), 
            speedup(speedup),
            current_time(0.0),
            start_time(dataset->start_time)
    {
        // Constant time iterator over poses
        pose_iterator.reset( new L3::ConstantTimeIterator<L3::SE3>( dataset->pose_reader ) );
        LHLV_iterator.reset( new L3::ConstantTimeIterator<L3::LHLV> ( dataset->LHLV_reader ) );  
       
        // LIDAR iterators
        vertical_LIDAR.reset( new L3::ConstantTimeIterator<L3::LMS151>( dataset->LIDAR_readers.begin()->second ) );
        horizontal_LIDAR.reset( new L3::ConstantTimeIterator<L3::LMS151>( dataset->LIDAR_readers.begin()->second ) );
  
        // Point-clouds
        point_cloud.reset( new L3::PointCloud<double>() );
        projection.reset( new L3::SE3( L3::SE3::ZERO() ) );
        
        L3::Configuration::convert( mission->lidars[ mission->declined], *projection );
    
        projector.reset( new L3::Projector<double>( projection.get(), point_cloud.get() ) );

        engine.reset( new L3::ScanMatching::Engine( horizontal_LIDAR.get() ) );
        
        oracle.reset( new L3::ConstantTimeWindower< L3::SE3 >( pose_iterator.get() ) );

        pose_windower.reset( new L3::ConstantTimeWindower< L3::LHLV>( LHLV_iterator.get() ) );
        
        (*this)<< pose_iterator.get() << LHLV_iterator.get() << vertical_LIDAR.get() << horizontal_LIDAR.get() << engine.get();

        swathe_builder.reset( new L3::SwatheBuilder( pose_windower.get(), vertical_LIDAR.get() ) );
   
        current.reset( new L3::SE3( L3::SE3::ZERO() ) );
    
    }

    ~DatasetRunner()
    {
        running = false;

        if ( thread.isRunning() )
            thread.join();

    }

    Dataset*        dataset;
    float           speedup;
    double          current_time, start_time;  
    
    
    std::list < Dumpable* > dumps;
    
    boost::shared_ptr< L3::SE3 > current;

    boost::shared_ptr< L3::SE3 >                projection;
    boost::shared_ptr< L3::Projector<double> >  projector;
    boost::shared_ptr< L3::PointCloud<double> > point_cloud;
    
    boost::shared_ptr< L3::SwatheBuilder >      swathe_builder;

    boost::shared_ptr< L3::ConstantTimeWindower< L3::SE3 > >    oracle;
    boost::shared_ptr< L3::ConstantTimeWindower< L3::LHLV > >   pose_windower;
    
    boost::shared_ptr< L3::ConstantTimeIterator< L3::SE3 > >    pose_iterator;
    boost::shared_ptr< L3::ConstantTimeIterator< L3::LHLV > >   LHLV_iterator;

    boost::shared_ptr< L3::ConstantTimeIterator< L3::LMS151 > > vertical_LIDAR;
    boost::shared_ptr< L3::ConstantTimeIterator< L3::LMS151 > > horizontal_LIDAR;
     
    boost::shared_ptr< L3::ScanMatching::Engine > engine;
    boost::shared_ptr< L3::ScanMatching::ScanMatcher > scan_matcher;
        
    void run();
    
    virtual bool update( double time )
    {
        return true;
    }

    DatasetRunner& operator<<( L3::TemporalObserver* observer)
    {
        if ( observer )
            observers.push_front( observer ); 
        return *this;
    }

    DatasetRunner& operator<<( L3::Dumpable* dumpable)
    {
        if ( dumpable )
            dumps.push_front( dumpable ); 
        return *this;
    }

};



/*
 *  Implementation specific
 */
struct EstimatorRunner : DatasetRunner
{
    EstimatorRunner( L3::Dataset* dataset, L3::Configuration::Mission* mission, L3::Experience* experience, float speedup=5.0 ) 
        : DatasetRunner( dataset, mission, speedup ),
            experience(experience)
    {
        estimated.reset( new L3::SE3( L3::SE3::ZERO() ) ); 
    
        this->provider = oracle.get();
    }


    ~EstimatorRunner()
    {
        running = false;

        if ( thread.isRunning() )
            thread.join();
    }
    
    boost::shared_ptr< L3::SE3 > estimated;

    L3::Experience*                         experience;
    L3::PoseProvider*                       provider;
    L3::ConstantTimeWindower<L3::LHLV>*     windower;
    L3::Estimator::Algorithm<double>*       algorithm;

    bool update( double time );

    EstimatorRunner& setPoseWindower( L3::ConstantTimeWindower<L3::LHLV>* windower )
    {
        this->windower = windower;
        (*this) << dynamic_cast<L3::TemporalObserver*>(windower);
        (*this) << dynamic_cast<L3::Dumpable*>(windower);

        return *this;
    }

    EstimatorRunner& setAlgorithm( L3::Estimator::Algorithm<double>* algorithm )
    {
        this->algorithm = algorithm;
        return *this;
    }
};


} // L3
#endif

