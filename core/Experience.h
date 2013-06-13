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
struct Experience : SpatialObserver, Poco::Runnable
{
    Experience( std::deque<experience_section> sections, std::string& fname, boost::shared_ptr< SelectionPolicy > policy, int WINDOW=10 );
    
    int                                     window;
    std::ifstream                           data;
    Poco::Thread                            thread;
    std::deque<experience_section>          sections;
    boost::shared_ptr< SelectionPolicy >    policy; 
    bool                                    running;
    double                                  _x,_y;

    std::map< unsigned int, std::pair< bool, boost::shared_ptr<L3::PointCloud<double> > > > resident_sections;
  
    boost::shared_ptr< L3::PointCloud<double> >  resident_point_cloud;
    boost::shared_ptr< L3::HistogramPyramid<double> > experience_pyramid;

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

    ExperienceLoader( const L3::Dataset& dataset, int window_sections = 10 ) : window_sections(window_sections)
    {
        load( dataset.path() );
    }

    ExperienceLoader( const std::string& target, int window_sections = 10 ) : window_sections(window_sections)
    {
        load( target );
    }

    int window_sections;
    boost::shared_ptr<Experience> experience;

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

        experience.reset( new Experience( sections, experience_name, boost::dynamic_pointer_cast< SelectionPolicy >( boost::make_shared< KNNPolicy >() ), window_sections ) );
        //experience.reset( new Experience( sections, experience_name, boost::dynamic_pointer_cast< SelectionPolicy >( boost::make_shared< StrictlyRetrospectivePolicy >() ), window_sections ) );
        //experience.reset( new Experience( sections, experience_name, boost::dynamic_pointer_cast< SelectionPolicy >( boost::make_shared< RetrospectiveWithLookaheadPolicy>() ), window_sections ) );
    }
        
};

/*
 *  Build an experience from a dataset
 */
struct ExperienceBuilder
{
    ExperienceBuilder( L3::Dataset& dataset, double start_time, double end_time, double experience_section_threshold=10.0, double scan_spacing_threshold=0.5  );

    boost::shared_ptr<L3::IO::BinaryReader< L3::SE3 > >                 pose_reader;
    boost::shared_ptr<L3::IO::SequentialBinaryReader< L3::LMS151 > >    LIDAR_reader;

};

}

#endif

