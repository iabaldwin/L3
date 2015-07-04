#include "Reflectance.h"
#include "ChainBuilder.h"

#include <flann/flann.hpp>

namespace L3
{

    ReflectanceBuilder::ReflectanceBuilder( L3::Dataset& dataset, double start_time, double end_time, double section_threshold, double scan_spacing_threshold )
    {
        std::cout.precision(15);

        // Pose reader
        pose_reader = boost::make_shared< L3::IO::BinaryReader<L3::SE3> >();

        if (!pose_reader->open( dataset.path() + "/OxTS.ins" ) )
        {
            std::cerr << "No poses available" << std::endl;
            throw std::exception();
        }

        // Scan reader
        LIDAR_reader.reset( new L3::IO::SequentialBinaryReader<L3::LMS151>() );

        if (!LIDAR_reader->open( dataset.path() + "/LMS1xx_10420002_192.168.0.50.lidar" ) )
        {
            std::cerr << "No LIDAR available" << std::endl;
            throw std::exception();
        }

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

        boost::shared_ptr< L3::PointCloudE<double> > point_cloud( new L3::PointCloudE<double>() );
        boost::shared_ptr< L3::ReflectanceProjector<double> > projector( new L3::ReflectanceProjector<double>( &calibration, point_cloud.get() ) );

        int id = 0;
        double accumulate = 0.0;

        // reflectance data
        std::ofstream reflectance_data( (dataset.path() + "/reflectance.dat").c_str(), std::ios::binary );
        std::ofstream reflectance_index( (dataset.path() + "/reflectance.index").c_str(), std::ios::binary );

        unsigned int stream_position = 0;

        double absolute_start_time = 0;

        double spacing = scan_spacing_threshold;
            
        MaskPolicy<L3::LMS151> policy( .1 );

        // Read LIDAR data, element at a time
        while( LIDAR_reader->read() )
        {
            // Extract scan
            LIDAR_reader->extract( scans, policy );

            // Initialisatino condition
            if (absolute_start_time == 0)
                absolute_start_time = scans[0].first;

            // Compute relative time
            double current_relative_time = scans[0].first - absolute_start_time;

            // Still to go, i.e. t_k < t_L?
            if ( current_relative_time < start_time )
                continue;

            // Done, i.e. t_k >= t_U?
            if ( current_relative_time > end_time )
                break;

            // Match pose
            std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > > matched;

            L3::Utils::matcher< L3::SE3, L3::LMS151 > m( &poses, &matched );

            m = std::for_each( scans.begin(), scans.end(),  m );

            double increment = length_estimator( matched[0] );

            accumulate += increment;

            // Add pose
            swathe.push_back( std::make_pair( matched[0].second, scans[0].second ) );

            if ( accumulate > section_threshold )
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
                boost::tuple<double,double,double> means = mean( point_cloud.get()  );

#ifndef NDEBUG
                std::cout << means.get<0>() << " " << means.get<1>() << std::endl;
#endif
                //Data format:
                //XYZ Alpha

                //  Writing : DATA
                //  1. Write points 
                unsigned int payload_size = point_cloud->num_points*sizeof(L3::PointE<double>);
                reflectance_data.write( (char*)( point_cloud->points ), payload_size );
                //  Writing : INDEX
                //  1. Write the ID
                reflectance_index.write( (char*)(&id), sizeof(int) ); id++;
                //  2. write the x,y location
                reflectance_index.write( (char*)(&means.get<0>()), sizeof(double) );
                reflectance_index.write( (char*)(&means.get<1>()), sizeof(double) );
                //  3. write the start position
                reflectance_index.write( (char*)(&stream_position), sizeof(unsigned int) );
                //  4. write the payload size
                reflectance_index.write( (char*)(&payload_size), sizeof(unsigned int) );

                stream_position += payload_size;

                // Reset swathe
                swathe.clear();
            }
        }
    }


    void ReflectanceLoader::load( const std::string& target )
    {
        std::ifstream reflectance_index( (target + "/reflectance.index").c_str(), std::ios::binary );

        if ( !reflectance_index.good() )
        {
            std::cerr << "No reflectance data <" <<  target << ">" << std::endl;
            return; 
        }

        std::string reflectance_data( target + "/reflectance.dat");

        spatial_data section;

        while( true )
        {
            reflectance_index.read( (char*)(&section.id),                sizeof(int) ); 
            reflectance_index.read( (char*)(&section.x),                 sizeof(double) );
            reflectance_index.read( (char*)(&section.y),                 sizeof(double) );
            reflectance_index.read( (char*)(&section.stream_position),   sizeof(unsigned int) );
            reflectance_index.read( (char*)(&section.payload_size),      sizeof(unsigned int) );

            if ( reflectance_index.good() )
                sections.push_back( section );
            else
                break;
        }

        reflectance_index.close();

        std::vector<double> pose_stream;

        for ( std::deque<spatial_data>::iterator it =  sections.begin();
                it != sections.end();
                it++ )
        {
            pose_stream.push_back( it->x );
            pose_stream.push_back( it->y );
        }

        flann::Matrix<float> flann_dataset(new float[pose_stream.size()], pose_stream.size()/2, 2 );

        float* ptr = flann_dataset[0];

        std::copy( pose_stream.begin(), pose_stream.end(), ptr );

        boost::shared_ptr< flann::Index< flann::L2<float> > > index = boost::make_shared< flann::Index< flann::L2<float> > >(flann_dataset, flann::KDTreeIndexParams(4));
        index->buildIndex();

        reflectance.reset( new Reflectance( sections, reflectance_data, boost::dynamic_pointer_cast< SelectionPolicy >( boost::make_shared< KNNPolicy >() ), window_sections ) );
    }


    /*
     *  Reflectance structure
     */
    Reflectance::Reflectance( std::deque< spatial_data > sections, 
            std::string fname, 
            boost::shared_ptr< SelectionPolicy > policy,
            int window )
        : SpatialQuery( sections, fname, policy, window ),
        resident_point_cloud( new L3::PointCloudE<double>() )
    {
        // Open 
        data.open( fname.c_str(), std::ios::binary );

        // Go
        thread.start( *this );
    }

    Reflectance::~Reflectance()
    {
        data.close();          
        
        running = false;        
        
        if( thread.isRunning() )
            thread.join();          
    }

    void Reflectance::run()
    {
        while( running )
        {
            L3::WriteLock master( this->mutex );

            /*
             *  Choose the sections required according to some policy
             */
            std::list<unsigned int> required_sections;
            if ( !policy->operator()( &sections, _x, _y, required_sections, window ) )  // Catastrophe
            {
                std::cerr << "Unable to apply selection policy, cannot continue..." << std::endl;
                exit(-1);
            }

            /*
             *  Mark everything as *NOT* required
             */
            for( std::map< unsigned int, std::pair< bool, boost::shared_ptr< L3::PointCloudE<double> > > >::iterator map_it = resident_sections.begin();
                    map_it != resident_sections.end();
                    map_it++ )
                map_it->second.first = false;

            /*
             *  Search
             */
            std::list< boost::shared_ptr<L3::PointCloudE<double> > > clouds;

            bool update_required = false;

            for ( std::list<unsigned int>::iterator it = required_sections.begin(); it != required_sections.end(); it++ )
            {
                std::map< unsigned int, std::pair< bool, boost::shared_ptr< L3::PointCloudE<double> > > >::iterator map_it = resident_sections.find( *it );

                //We need it, and don't have it
                if ( map_it == resident_sections.end() )
                {
                    update_required = true; 

                    //Load 
                    std::pair< unsigned int, L3::PointE<double>* > load_result = load(*it);

                    L3::PointCloudE<double>* cloud = new L3::PointCloudE<double>();

                    cloud->num_points = load_result.first;
                    cloud->points = load_result.second;

                    // Insert, mark it as required by default, transfer ownership
                    resident_sections.insert( std::make_pair( *it, std::make_pair( true, boost::shared_ptr<L3::PointCloudE<double> >( cloud ) ) ) );
                }
                else
                {
                    // We need it, and we have it
                    map_it->second.first = true;
                }
            }

            /*
             *  Erase everything *NOT* required
             */
            std::map< unsigned int, std::pair< bool, boost::shared_ptr< L3::PointCloudE<double> > > >::iterator map_it = resident_sections.begin();
            while( map_it != resident_sections.end() )
            {
                if( !map_it->second.first )
                {
                    update_required = true; 
                    map_it->second.second.reset(); 
                    resident_sections.erase( map_it++ );
                }
                else
                    clouds.push_back( map_it++->second.second ); 

            }

            /*
             *Assign the resident cloud
             */
            if (update_required)
            {
                // Join clouds
                join( clouds, resident_point_cloud );
            }

            master.unlock();

            // Play nice
            usleep( .15*1e6 );

        }
    }

    std::pair< long unsigned int, L3::PointE<double>* > Reflectance::load( unsigned int id )
    {
        if( id >= sections.size() )
        {
            throw std::exception();
        }

        // Seek
        data.seekg( sections[id].stream_position, std::ios_base::beg );
        char* tmp = new char[sections[id].payload_size];

        // Read 
        data.read( tmp, sections[id].payload_size );

        // DBG checks
        assert( data.good() ); 
        assert( sections[id].payload_size == data.gcount() );

        return std::make_pair( sections[id].payload_size/sizeof(L3::PointE<double>), reinterpret_cast<L3::PointE<double>*>( tmp ) );
    }

}
