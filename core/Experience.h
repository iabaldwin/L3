#ifndef L3_EXPERIENCE_H
#define L3_EXPERIENCE_H

#include "PointCloud.h"
#include "Dataset.h"
#include "Reader.h"
#include "Projector.h"
#include "Configuration.h"
#include "Histogram.h"
#include "Smoother.h"
#include "ChainBuilder.h"
#include "SpatialData.h"

#include <flann/flann.hpp>

namespace L3
{

/*
 *Core experience
 */
struct Experience : SpatialQuery
    {
        Experience( std::deque< spatial_data > sections, 
                std::string fname, 
                boost::shared_ptr< SelectionPolicy > policy, 
                int window_size=2, 
                boost::shared_ptr< flann::Index< flann::L2<float> > > index = boost::shared_ptr< flann::Index< flann::L2<float> > >(),
                boost::shared_ptr< std::deque< L3::SE3 > > poses  = boost::shared_ptr< std::deque< L3::SE3 > >()) ;

        boost::shared_ptr< std::deque< L3::SE3 > > poses; 
        boost::shared_ptr< flann::Index< flann::L2<float> > > pose_lookup;

        std::map< unsigned int, std::pair< bool, boost::shared_ptr<L3::PointCloud<double> > > > resident_sections;

        boost::shared_ptr< L3::PointCloud<double> >  resident_point_cloud;
        boost::shared_ptr< L3::HistogramPyramid<double> > experience_pyramid;

        ~Experience();

        virtual void    run();
        void            initialise();
        void            createHistograms( const std::vector< double >& densities  );

        L3::SE3         getClosestPose( const L3::SE3& input );

        std::pair< long unsigned int, L3::Point<double>* > load( unsigned int id );

    };

/*
 *  Experience loader
 */
struct ExperienceLoader
{
    ExperienceLoader( const L3::Dataset& dataset, int window_sections = 3 ) : window_sections(window_sections)
    {
        load( dataset.path() );
    }

    ExperienceLoader( const std::string& target, int window_sections = 3 ) : window_sections(window_sections)
    {
        load( target );
    }

    int window_sections;
    std::deque<spatial_data> sections;
    boost::shared_ptr<Experience> experience;

    void load( const std::string& target )
    {
        std::ifstream experience_index( (target + "/experience.index").c_str(), std::ios::binary );

        if ( !experience_index.good() )
            throw L3::no_such_file();

        std::string experience_name( target + "/experience.dat");

        spatial_data section;

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

        int counter = 0;
        std::vector<double> tmp(4);
        
        std::vector<float> pose_stream;
        
        while( experience_poses.good() )
        {
            double datum;

            experience_poses.read( (char*)(&datum), sizeof(double) );

            tmp[counter++] = datum; 

            if( counter == 4 )
            {
                poses->push_back( L3::SE3( tmp[0], tmp[1], tmp[2] , 0, 0, tmp[3] ) );
                counter = 0;
            
                pose_stream.push_back( float(tmp[0] )  );   // Only search over x and y
                pose_stream.push_back( float(tmp[1] )  );
  
            }
        
        }

        experience_poses.close();

        // Only query over x an dy
        flann::Matrix<float> flann_dataset(new float[pose_stream.size()], pose_stream.size()/2, 2 );

        float* ptr = flann_dataset[0];

        std::copy( pose_stream.begin(), pose_stream.end(), ptr );

        boost::shared_ptr< flann::Index< flann::L2<float> > > index = boost::make_shared< flann::Index< flann::L2<float> > >(flann_dataset, flann::KDTreeIndexParams(4));
        index->buildIndex();

        experience.reset( new Experience( sections, experience_name, boost::dynamic_pointer_cast< SelectionPolicy >( boost::make_shared< KNNPolicy >() ), window_sections, index, poses ) );
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

