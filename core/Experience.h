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

#include <flann/flann.hpp>

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

struct SelectionPolicy
{
    virtual bool operator()( std::deque< experience_section>* sections, double x, double y, std::list<unsigned int>& required_sections, const int window ) 
    {
        return false;
    }
};

struct KNNPolicy : SelectionPolicy
{
    bool operator()( std::deque< experience_section>* sections, double x, double y, std::list<unsigned int>& required_sections, const int window );
};

struct StrictlyRetrospectivePolicy : SelectionPolicy
{
    bool operator()( std::deque< experience_section>* sections, double x, double y, std::list<unsigned int>& required_sections, const int window );
};

struct RetrospectiveWithLookaheadPolicy: SelectionPolicy
{
    bool operator()( std::deque< experience_section>* sections, double x, double y, std::list<unsigned int>& required_sections, const int window );
};

/*
 *Core experience
 */
struct Experience : SpatialObserver, Poco::Runnable, Lockable
{
    Experience( std::deque<experience_section> sections, 
            std::string& fname, 
            boost::shared_ptr< SelectionPolicy > policy, 
            int WINDOW=2, 
            boost::shared_ptr< flann::Index< flann::L2<float> > > index = boost::shared_ptr< flann::Index< flann::L2<float> > >(),
            boost::shared_ptr< std::deque< L3::SE3 > > poses  = boost::shared_ptr< std::deque< L3::SE3 > >()) ;
    
    int                                     window;
    std::ifstream                           data;
    Poco::Thread                            thread;
    std::deque<experience_section>          sections;
    boost::shared_ptr< SelectionPolicy >    policy; 
    bool                                    running;
    double                                  _x,_y;

    boost::shared_ptr< std::deque< L3::SE3 > > poses; 
    boost::shared_ptr< flann::Index< flann::L2<float> > > pose_lookup;

    std::map< unsigned int, std::pair< bool, boost::shared_ptr<L3::PointCloud<double> > > > resident_sections;
  
    boost::shared_ptr< L3::PointCloud<double> >  resident_point_cloud;
    boost::shared_ptr< L3::HistogramPyramid<double> > experience_pyramid;

    ~Experience();

    virtual void    run();
    void            initialise();
    bool            update( double x, double y );
    void            createHistograms( const std::vector< double >& densities  );
  
    L3::SE3         getClosestPose( const L3::SE3& input );

    std::pair< long unsigned int, L3::Point<double>* > load( unsigned int id );
    
};

/*
 *  Experience loader
 */
struct ExperienceLoader
{
    std::deque<experience_section> sections;

    ExperienceLoader( const L3::Dataset& dataset, int window_sections = 3 ) : window_sections(window_sections)
    {
        load( dataset.path() );
    }

    ExperienceLoader( const std::string& target, int window_sections = 3 ) : window_sections(window_sections)
    {
        load( target );
    }

    int window_sections;
    boost::shared_ptr<Experience> experience;

    void load( const std::string& target )
    {
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


        // Read experience poses
        std::ifstream experience_poses( (target + "/experience.pose").c_str(), std::ios::binary );

        boost::shared_ptr< std::deque< L3::SE3 > > poses = boost::make_shared< std::deque< L3::SE3 > >();  
        std::vector<float> pose_stream;

        int counter = 0;
        std::vector<double> tmp(4);
        while( experience_poses.good() )
        {
            double datum;

            experience_poses.read( (char*)(&datum), sizeof(double) );

            pose_stream.push_back( float(datum) );
  
            tmp[counter++] = datum; 

            if( counter == 4 )
            {
                poses->push_back( L3::SE3( tmp[0], tmp[1], tmp[2] , 0, 0, tmp[3] ) );
           
                counter = 0;
            }
        
        }

        experience_poses.close();

        flann::Matrix<float> flann_dataset(new float[pose_stream.size()], pose_stream.size()/4, 4 );

        float* ptr = flann_dataset[0];

        std::copy( pose_stream.begin(), pose_stream.end(), ptr );

        boost::shared_ptr< flann::Index< flann::L2<float> > > index = boost::make_shared< flann::Index< flann::L2<float> > >(flann_dataset, flann::KDTreeIndexParams(4));
        index->buildIndex();

        experience.reset( new Experience( sections, experience_name, boost::dynamic_pointer_cast< SelectionPolicy >( boost::make_shared< KNNPolicy >() ), window_sections, index, poses ) );
        //experience.reset( new Experience( sections, experience_name, boost::dynamic_pointer_cast< SelectionPolicy >( boost::make_shared< StrictlyRetrospectivePolicy >() ), window_sections ) );
        //experience.reset( new Experience( sections, experience_name, boost::dynamic_pointer_cast< SelectionPolicy >( boost::make_shared< RetrospectiveWithLookaheadPolicy>() ), window_sections ) );
    }
        
};

/*
 *  Build an experience from a dataset
 */
struct ExperienceBuilder
{
    ExperienceBuilder( L3::Dataset& dataset, double start_time, double end_time, double experience_section_threshold=10.0, double scan_spacing_threshold=0.2  );

    boost::shared_ptr<L3::IO::BinaryReader< L3::SE3 > >                 pose_reader;
    boost::shared_ptr<L3::IO::SequentialBinaryReader< L3::LMS151 > >    LIDAR_reader;

};

}

#endif

