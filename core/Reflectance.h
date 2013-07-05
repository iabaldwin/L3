#ifndef L3_REFLECTANCE_BUILDER_H
#define L3_REFLECTANCE_BUILDER_H

#include "Datatypes.h"
#include "PointCloud.h"
#include "Dataset.h"
#include "Reader.h"
#include "Projector.h"
#include "Configuration.h"
#include "Integrator.h"
#include "SpatialData.h"

namespace L3
{
    struct Reflectance : SpatialQuery
    {
        Reflectance( std::deque< spatial_data > sections, 
                std::string fname, 
                boost::shared_ptr< SelectionPolicy > policy, 
                int window_size=2 );

        std::map< unsigned int, std::pair< bool, boost::shared_ptr<L3::PointCloudE<double> > > > resident_sections;

        boost::shared_ptr< L3::PointCloudE<double> >  resident_point_cloud;

        ~Reflectance();

        virtual void    run();
        void            initialise();
        void            createHistograms( const std::vector< double >& densities  );

        std::pair< long unsigned int, L3::PointE<double>* > load( unsigned int id );

    };

    /*
     *  Builder
     */
    struct ReflectanceBuilder
    {
        ReflectanceBuilder( L3::Dataset& dataset, double start_time, double end_time, double experience_section_threshold=10.0, double scan_spacing_threshold=0.2  );    

        boost::shared_ptr<L3::IO::BinaryReader< L3::SE3 > >                 pose_reader;
        boost::shared_ptr<L3::IO::SequentialBinaryReader< L3::LMS151 > >    LIDAR_reader;
    };

    /*
     *  Loader
     */
    struct ReflectanceLoader
    {

        ReflectanceLoader( const L3::Dataset& dataset, int window_sections = 3 ) : window_sections(window_sections)
        {
            load( dataset.path() );
        }

        int window_sections;
        std::deque<spatial_data> sections;
        boost::shared_ptr<Reflectance> reflectance;

        void load( const std::string& target );
    };
}

#endif

