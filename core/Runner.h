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
            static_cast<L3::TemporalObserver*>( *it )->update( t ) ;
            // Safe, slow 
            //L3::TemporalObserver* observer = dynamic_cast<L3::TemporalObserver*>( *it );
    }

};

/*
 *  Threaded runner
 */
struct ThreadedTemporalRunner : TemporalRunner, Poco::Runnable
{

    ThreadedTemporalRunner() 
        : running( true ),
            current_time(0.0)
    {
    }

    Poco::Thread    thread;
    bool            running;
    double          start_time, current_time;

    void stop()
    {
        running = false;
    }

    ~ThreadedTemporalRunner()
    {
        stop();
        if ( thread.isRunning() )
            thread.join();
    }

    void start( double start_time )
    {
        this->start_time = start_time;
        thread.start( *this );
    }

    virtual void run()
    {
    }
};

struct DatasetRunner : ThreadedTemporalRunner
{
    DatasetRunner( L3::Dataset* dataset, L3::Configuration::Mission* mission, float speedup=5.0 ) 
        : dataset(dataset), 
            speedup(speedup),
            current_time(0.0),
            start_time(dataset->start_time),
            running(true)
    {
        // Constant time iterator over poses
        pose_iterator.reset( new L3::ConstantTimeIterator<L3::SE3>( dataset->pose_reader ) );
        LHLV_iterator.reset( new L3::ConstantTimeIterator<L3::LHLV> ( dataset->LHLV_reader ) );  
        
        vertical_LIDAR.reset( new L3::ConstantTimeIterator<L3::LMS151>( dataset->LIDAR_readers.begin()->second ) );
        horizontal_LIDAR.reset( new L3::ConstantTimeIterator<L3::LMS151>( dataset->LIDAR_readers.begin()->second ) );
   
        point_cloud.reset( new L3::PointCloud<double>() );
   
        projection.reset( new L3::SE3( L3::SE3::ZERO() ) );
        
        L3::Configuration::convert( mission->lidars[ mission->declined], *projection );
    
        projector.reset( new L3::Projector<double>( projection.get(), point_cloud.get() ) );

        engine.reset( new L3::ScanMatching::Engine( horizontal_LIDAR.get() ) );
        (*this) << dynamic_cast<L3::TemporalObserver*>(engine.get());

        (*this)<< pose_iterator.get() << LHLV_iterator.get() << vertical_LIDAR.get() << horizontal_LIDAR.get();
  
        // Pose Windower
        pose_windower.reset( new L3::ConstantTimeWindower< L3::LHLV>( LHLV_iterator.get() ) );
        swathe_builder.reset( new L3::SwatheBuilder( pose_windower.get(), vertical_LIDAR.get() ) );
   
    }

    std::list < Dumpable* >         dumps;
    std::list < TemporalObserver* > observers;

    boost::shared_ptr< L3::ConstantTimeWindower<L3::SE3 > >     oracle;
    boost::shared_ptr< L3::ConstantTimeWindower<L3::LHLV > >    pose_windower;

    boost::shared_ptr< L3::SwatheBuilder > swathe_builder;
    
    boost::shared_ptr< L3::SE3 >                projection;
    boost::shared_ptr< L3::Projector<double> >  projector;
    boost::shared_ptr< L3::PointCloud<double> > point_cloud;

    boost::shared_ptr< L3::ConstantTimeIterator< L3::SE3 > >    pose_iterator;
    boost::shared_ptr< L3::ConstantTimeIterator< L3::LHLV > >   LHLV_iterator;

    boost::shared_ptr< L3::ConstantTimeIterator< L3::LMS151 > > vertical_LIDAR;
    boost::shared_ptr< L3::ConstantTimeIterator< L3::LMS151 > > horizontal_LIDAR;
     
    boost::shared_ptr< L3::ScanMatching::Engine > engine;
    boost::shared_ptr< L3::ScanMatching::ScanMatcher > scan_matcher;
    
    Poco::Thread    thread;
    Dataset*        dataset;
    bool            running;
    float           speedup;
    double          start_time, current_time; 

    virtual ~DatasetRunner()
    {
        stop();
      
        if ( thread.isRunning() )
            thread.join();
    }

    void start()
    {
        thread.start( *this );
    }

    void stop()
    {
        running = false;
    }

    void run();
    
    virtual bool update( double time )
    {
        std::cout << "HI" << std::endl;
    }

    DatasetRunner& operator<<( L3::TemporalObserver* observer )
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
    EstimatorRunner( L3::Dataset* dataset, L3::Configuration::Mission* mission, float speedup=5.0 ) :
        DatasetRunner( dataset, mission, speedup )
    {
        current.reset( new L3::SE3( L3::SE3::ZERO() ) );
        estimated.reset( new L3::SE3( L3::SE3::ZERO() ) ); 
    }


    L3::Experience*                         experience;
    L3::PoseProvider*                       provider;
    L3::ConstantTimeWindower<L3::LHLV>*     windower;
    L3::Estimator::Algorithm<double>*       estimator;

    boost::shared_ptr< L3::SE3 > current;
    boost::shared_ptr< L3::SE3 > estimated;


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
        //this->swathe_builder = swathe_builder;
        //(*this) << dynamic_cast<L3::Dumpable*>(swathe_builder);
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
        //this->projector = projector;
        return *this;
    }

    EstimatorRunner& setAlgorithm( L3::Estimator::Algorithm<double>* algorithm )
    {
        //this->algorithm = algorithm;
        return *this;
    }


};


} // L3
#endif

