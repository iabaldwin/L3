#include "RoadBuilder.h"
#include "ChainBuilder.h"

namespace L3
{

    RoadBuilder::RoadBuilder( L3::Dataset& dataset, double start_time, double end_time, double experience_section_threshold, double scan_spacing_threshold )
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
        //LIDAR_reader.reset( new L3::IO::SequentialBinaryReader<L3::LMS151>( RoadMaskPolicy<L3::LMS151>() ) );
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

        // Road data
        std::ofstream road_data( (dataset.path() + "/road.dat").c_str(), std::ios::binary );

        unsigned int stream_position = 0;

        double absolute_start_time = 0;

        double spacing = scan_spacing_threshold;

        // Read LIDAR data, element at a time
        while( LIDAR_reader->read() )
        {
            // Extract scan
            LIDAR_reader->extract( scans, RoadMaskPolicy<L3::LMS151>() );

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

                //Data format:
                //XYZ Alpha

                //  Writing : DATA
                //  1. Write points 
                //unsigned int payload_size = point_cloud->num_points*sizeof(L3::Point<double>);
                //experience_data.write( (char*)( point_cloud->points ), payload_size );
                ////  Writing : INDEX
                ////  1. Write the ID
                //experience_index.write( (char*)(&id), sizeof(int) ); id++;
                ////  2. write the x,y location
                //experience_index.write( (char*)(&means.get<0>()), sizeof(double) );
                //experience_index.write( (char*)(&means.get<1>()), sizeof(double) );
                ////  3. write the start position
                //experience_index.write( (char*)(&stream_position), sizeof(unsigned int) );
                ////  4. write the payload size
                //experience_index.write( (char*)(&payload_size), sizeof(unsigned int) );

                //stream_position += payload_size;

                // Reset swathe
                swathe.clear();
            }
        }
    }



}
