#include "Experience.h"

namespace L3
{
    std::ostream& operator<<( std::ostream& o, spatial_data section )
    {
        o << section.id << ":" << section.x << "," << section.y << "(" << section.stream_position << "," << section.payload_size << ")";
        return o;
    }

    bool operator<( std::pair< double, unsigned int > a, std::pair< double, unsigned int > b )
    {
        return a.first < b.first;
    }

    /*
     *  Builder
     */
    ExperienceBuilder::ExperienceBuilder( L3::Dataset& dataset, double start_time, double end_time, double experience_section_threshold, double scan_spacing_threshold )
    {
        std::cout.precision(15);

        // Pose reader
        pose_reader.reset( new L3::IO::BinaryReader<L3::SE3>() );
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

        boost::shared_ptr< L3::PointCloud<double> > point_cloud( new L3::PointCloud<double>() );
        boost::shared_ptr< L3::Projector<double> > projector( new L3::Projector<double>( &calibration, point_cloud.get() ) );

        int id = 0;
        double accumulate = 0.0;

        // Experience data
        std::ofstream experience_data( (dataset.path() + "/experience.dat").c_str(), std::ios::binary );
        std::ofstream experience_index( (dataset.path() + "/experience.index").c_str(), std::ios::binary );
        std::ofstream experience_poses( (dataset.path() + "/experience.pose").c_str(), std::ios::binary );

        unsigned int stream_position = 0;

        double absolute_start_time = 0;

        double spacing = scan_spacing_threshold;

        // Read LIDAR data, element at a time
        while( LIDAR_reader->read() )
        {
            // Extract scan
            LIDAR_reader->extract( scans );

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
            spacing -= increment;

            //So, here we have estimated the accumulate. Once we have gone X metres, add it back
            std::cout << accumulate / scan_spacing_threshold << "[" << scan_spacing_threshold << "," << accumulate << "," << experience_section_threshold << "]" << std::endl;

            if( spacing <= 0.0 )
            {
                swathe.push_back( std::make_pair( matched[0].second, scans[0].second ) );
                spacing = scan_spacing_threshold;

                double val = matched[0].second->X();
                experience_poses.write( (char*)(&val ), sizeof(double) );
                val = matched[0].second->Y();
                experience_poses.write( (char*)(&val ), sizeof(double) );
                val = matched[0].second->Z();
                experience_poses.write( (char*)(&val ), sizeof(double) );
                val = matched[0].second->Q();
                experience_poses.write( (char*)(&val ), sizeof(double) );


            }

            if ( accumulate > experience_section_threshold )
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

                //  Writing : DATA
                //  1. Write points 
                unsigned int payload_size = point_cloud->num_points*sizeof(L3::Point<double>);
                experience_data.write( (char*)( point_cloud->points ), payload_size );
                //  Writing : INDEX
                //  1. Write the ID
                experience_index.write( (char*)(&id), sizeof(int) ); id++;
                //  2. write the x,y location
                experience_index.write( (char*)(&means.get<0>()), sizeof(double) );
                experience_index.write( (char*)(&means.get<1>()), sizeof(double) );
                //  3. write the start position
                experience_index.write( (char*)(&stream_position), sizeof(unsigned int) );
                //  4. write the payload size
                experience_index.write( (char*)(&payload_size), sizeof(unsigned int) );

                stream_position += payload_size;

                // Reset swathe
                swathe.clear();
            }
        }
        std::cout << "Done. Wrote " <<  scans.back().first - absolute_start_time << "s" << std::endl;
    }

    /*
     *  Experience
     */

    Experience::Experience( std::deque< spatial_data > sections, 
            std::string fname, 
            boost::shared_ptr< SelectionPolicy > policy,
            int window_size, 
            boost::shared_ptr< flann::Index< flann::L2<float> > > index ,
            boost::shared_ptr< std::deque< L3::SE3 > > poses ) 
        : SpatialQuery( sections, fname, policy, window_size ),
        resident_point_cloud( new L3::PointCloud<double>() ),
        pose_lookup( index ),
        poses(poses)
    {
        // Open 
        data.open( fname.c_str(), std::ios::binary );

        // Allocate
        std::vector<double> densities;

        densities.push_back( 1 );
        densities.push_back( 2 );
        densities.push_back( 4 );

        createHistograms( densities );

        // Go
        thread.start( *this );

    }

    Experience::~Experience()
    {
        data.close();          
        running = false;        
        if( thread.isRunning() )
            thread.join();          
    }

    void Experience::createHistograms( const std::vector< double >& densities  )
    {
        experience_pyramid.reset( new L3::HistogramPyramid<double>( densities ) );
    }

    L3::SE3 Experience::getClosestPose( const L3::SE3& input )
    {
        // Get the single closest pose
        flann::Matrix<int> indices(new int[1*1], 1, 1 );
        flann::Matrix<float> dists(new float[1*1], 1, 1);

        //flann::Matrix<float> query( new float[1*4], 1, 4 );
        flann::Matrix<float> query( new float[1*2], 1, 2 );

        float* ptr = query[0];

        *ptr++ = input.X();
        *ptr++ = input.Y();
        //*ptr++ = input.Z();
        //*ptr++ = input.Q();

        pose_lookup->knnSearch( query, indices, dists, 1, flann::SearchParams(128));

        L3::SE3 closest = (*poses)[*indices[0]];

        delete [] indices.ptr();
        delete [] dists.ptr();

        return closest;
    }

    void Experience::run()
    {
        while( running )
        {
            L3::ReadLock master( this->mutex );

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
            for( std::map< unsigned int, std::pair< bool, boost::shared_ptr< L3::PointCloud<double> > > >::iterator map_it = resident_sections.begin();
                    map_it != resident_sections.end();
                    map_it++ )
                map_it->second.first = false;

            /*
             *  Search
             */
            std::list< boost::shared_ptr<L3::PointCloud<double> > > clouds;

            bool update_required = false;

            for ( std::list<unsigned int>::iterator it = required_sections.begin(); it != required_sections.end(); it++ )
            {
                std::map< unsigned int, std::pair< bool, boost::shared_ptr< L3::PointCloud<double> > > >::iterator map_it = resident_sections.find( *it );

                //We need it, and don't have it
                if ( map_it == resident_sections.end() )
                {
                    update_required = true; 

                    //Load 
                    std::pair< unsigned int, L3::Point<double>* > load_result = load(*it);

                    L3::PointCloud<double>* cloud = new L3::PointCloud<double>();

                    cloud->num_points = load_result.first;
                    cloud->points = load_result.second;

                    // Insert, mark it as required by default, transfer ownership
                    resident_sections.insert( std::make_pair( *it, std::make_pair( true, boost::shared_ptr<L3::PointCloud<double> >( cloud ) ) ) );
                }
                else
                {
                    // We need it, and we have it
                    map_it->second.first = true;
                }
            }

            /*
             *Erase everything *NOT* required
             */
            std::map< unsigned int, std::pair< bool, boost::shared_ptr< L3::PointCloud<double> > > >::iterator map_it = resident_sections.begin();
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

                if ( resident_point_cloud->num_points != 0 )
                {
                    // Compute histogram
                    boost::tuple<double,double,double> min_bound = L3::min<double>( &*resident_point_cloud );
                    boost::tuple<double,double,double> max_bound = L3::max<double>( &*resident_point_cloud );
                    boost::tuple<double,double,double> means     = L3::mean( &*resident_point_cloud );

                    //L3::BoxSmoother< double, 3 > smoother; 
                    L3::LogisticSmoother< double > logistic_smoother; 
                    L3::GaussianSmoother< double > gaussian_smoother; 

                    for( L3::HistogramPyramid<double>::PYRAMID_ITERATOR it = this->experience_pyramid->begin();
                            it != this->experience_pyramid->end();
                            it++ )
                    {

                        boost::shared_ptr<L3::HistogramUniformDistance<double> > current_histogram 
                            = boost::dynamic_pointer_cast<L3::HistogramUniformDistance<double> >(*it);

                        WriteLock lock( current_histogram->mutex );

                        current_histogram->create(  means.get<0>(), 
                                min_bound.get<0>(), 
                                max_bound.get<0>(),
                                means.get<1>(),                   
                                min_bound.get<1>(), 
                                max_bound.get<1>());

                        current_histogram->operator()( resident_point_cloud.get() );
                        logistic_smoother.smooth( current_histogram.get() );
                        gaussian_smoother.smooth( current_histogram.get() );

                        lock.unlock();

                    }
                }
            }

            master.unlock();

            // Play nice
            usleep( .15*1e6 );

        }
    }

    
    std::pair< long unsigned int, L3::Point<double>* > Experience::load( unsigned int id )
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

        return std::make_pair( sections[id].payload_size/sizeof(L3::Point<double>), reinterpret_cast<L3::Point<double>*>( tmp ) );
    }

}
