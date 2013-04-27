#ifndef L3_EXPERIENCE_H
#define L3_EXPERIENCE_H

#include <Poco/Thread.h>

#include "ChainBuilder.h"
#include "Datatypes.h"
#include "PointCloud.h"
#include "Dataset.h"
#include "Reader.h"
#include "Projector.h"
#include "Configuration.h"
#include "Histogram.h"
#include "Smoother.h"

#include <map>

struct LengthEstimatorInterface : L3::LengthEstimator
{
    L3::LengthEstimator estimator;

    double operator()( std::pair< double, boost::shared_ptr<L3::SE3> > element )
    {
        return estimator( *element.second );
    }

};

namespace L3
{

struct experience_section
{
    int id;
    double x,y;
    unsigned int stream_position;
    unsigned int payload_size;
};

/*
 *Core experience
 */
struct Experience : SpatialObserver, Poco::Runnable
{

    Experience( std::deque<experience_section>  SECTIONS, std::string& fname, unsigned int WINDOW=10 );
    
    double                          _x,_y;
    std::deque<experience_section>  sections;
    std::ifstream                   data;
    Poco::Thread                    thread;
    
    unsigned int                    window;
    bool                            running;
    std::map< unsigned int, std::pair< bool, boost::shared_ptr<L3::PointCloud<double> > > > resident_sections;
  
    boost::shared_ptr< L3::PointCloud<double> >  resident_point_cloud;
    boost::shared_ptr< L3::Histogram<double> >   experience_histogram;

    ~Experience();

    void            initialise();
    virtual void    run();
    bool            update( double x, double y );
    
    std::pair< long unsigned int, L3::Point<double>* > load( unsigned int id );
    
};

/*
 *  Experience loader
 */
struct ExperienceLoader
{
    std::deque<experience_section> sections;

    ExperienceLoader( const L3::Dataset& dataset )
    {
        load( dataset.path() );
    }

    ExperienceLoader( const std::string& target )
    {
        load( target );
    }

    void load( const std::string& target )
    {
        std::cout << target + "/experience.index" << std::endl;

        std::ifstream experience_index( (target + "/experience.index").c_str(), std::ios::binary );

        if ( !experience_index.good() )
            throw L3::no_such_file();

        std::string experience_name( target + "/experience.dat");

        experience_section section;

        while( true )
        {
            experience_index.read( (char*)(&section.id),                sizeof(int) ); 
            experience_index.read( (char*)(&section.x),                 sizeof(double) );
            experience_index.read( (char*)(&section.y),                 sizeof(double) );
            experience_index.read( (char*)(&section.stream_position),   sizeof(unsigned int) );
            experience_index.read( (char*)(&section.payload_size),      sizeof(unsigned int) );

            if ( experience_index.good() )
                sections.push_back( section );
            else
                break;
        }

        experience_index.close();

        experience.reset( new Experience( sections, experience_name ) );
    }
        
    boost::shared_ptr<Experience> experience;

};

/*
 *  Build an experience from a dataset
 */
struct ExperienceBuilder
{
    ExperienceBuilder( L3::Dataset& dataset, double start_time, double end_time, double threshold=10.0 )
    {
        std::cout.precision(15);
       
        // Pose reader
        pose_reader.reset( new L3::IO::BinaryReader<L3::SE3>() );
        if (!pose_reader->open( dataset.path() + "/OxTS.ins" ) )
            throw std::exception();

        // Scan reader
        LIDAR_reader.reset( new L3::IO::SequentialBinaryReader<L3::LMS151>() );
        if (!LIDAR_reader->open( dataset.path() + "/LMS1xx_10420002_192.168.0.50.lidar" ) )
            throw std::exception();
        
        std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > >      poses;
        std::vector< std::pair< double, boost::shared_ptr<L3::LMS151> > >   scans;

        // Read *all* the poses
        pose_reader->read();
        pose_reader->extract( poses );

        // Structure for calculating pose chain length
        LengthEstimatorInterface length_estimator;

        // Matched swathe
        SWATHE swathe;

        // Calibration/projection
        L3::Configuration::Mission mission( dataset );   
        L3::SE3 calibration = L3::SE3::ZERO();
        L3::Configuration::convert( mission.lidars[ mission.declined ], calibration );

        std::cout << calibration << std::endl;

        boost::shared_ptr< L3::PointCloud<double> > point_cloud( new L3::PointCloud<double>() );
        boost::shared_ptr< L3::Projector<double> > projector( new L3::Projector<double>( &calibration, point_cloud.get() ) );

        int id = 0;
        double accumulate = 0.0;
     
        // Experience data
        std::ofstream experience_data( (dataset.path() + "/experience.dat").c_str(), std::ios::binary );
        std::ofstream experience_index( (dataset.path() + "/experience.index").c_str(), std::ios::binary );

        unsigned int stream_position = 0;

        double absolute_start_time = 0;

        // Read LIDAr data, element at a time
        while( LIDAR_reader->read() )
        {
            // Extract scan
            LIDAR_reader->extract( scans );

            //exit(-1);

            if (absolute_start_time == 0)
                absolute_start_time = scans[0].first;

            double current_relative_time = scans[0].first - absolute_start_time;

            // Still to go?
            if ( current_relative_time < start_time )
                continue;

            // Done
            if ( current_relative_time > end_time )
                break;

            // Match pose
            std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > > matched;
            
            L3::Utils::matcher< L3::SE3, L3::LMS151 > m( &poses, &matched );
            
            m = std::for_each( scans.begin(), scans.end(),  m );

            accumulate += length_estimator( matched[0] );
  
            swathe.push_back( std::make_pair( matched[0].second, scans[0].second ) );

            //yaw, is 0
            //std::cout << *matched[0].second << std::endl;

            if ( accumulate > threshold )
            {
                // Reset
                accumulate = 0.0;
#ifndef NDEBUG
                L3::Timing::SysTimer t;
                t.begin();
#endif
                // Project the points 
                projector->project( swathe );

#ifndef NDEBUG
                std::cout << point_cloud->num_points << " points in " << t.elapsed() << "s" << std::endl;
#endif
                // Compute mean
                std::pair<double,double> means = mean( point_cloud.get()  );

                std::cout << means.first << " " << means.second << std::endl;
              
                //  Writing : DATA
                //  1. Write points 
                unsigned int payload_size = point_cloud->num_points*sizeof(L3::Point<double>);
                experience_data.write( (char*)( point_cloud->points ), payload_size );
                //  Writing : INDEX
                //  1. Write the ID
                experience_index.write( (char*)(&id), sizeof(int) ); id++;
                //  2. write the x,y location
                experience_index.write( (char*)(&means.first), sizeof(double) );
                experience_index.write( (char*)(&means.second), sizeof(double) );
                //  3. write the start position
                experience_index.write( (char*)(&stream_position), sizeof(unsigned int) );
                //  4. write the payload size
                experience_index.write( (char*)(&payload_size), sizeof(unsigned int) );

                stream_position += payload_size;
                                
                // Reset swathe
                swathe.clear();
            }
        }
    }

    std::auto_ptr<L3::IO::BinaryReader< L3::SE3 > >                 pose_reader;
    std::auto_ptr<L3::IO::SequentialBinaryReader< L3::LMS151 > >    LIDAR_reader;

};

}

#endif

