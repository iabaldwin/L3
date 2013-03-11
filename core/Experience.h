#ifndef L3_EXPERIENCE_H
#define L3_EXPERIENCE_H

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


/*
 *Build an experience from a dataset
 */
struct ExperienceBuilder
{

    ExperienceBuilder( L3::Dataset& dataset )
    {
        std::cout.precision(15);
        
        pose_reader.reset( new L3::IO::BinaryReader<L3::SE3>() );
        if (!pose_reader->open( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/L3/OxTS.ins" ))
            throw std::exception();

        LIDAR_reader.reset( new L3::IO::SequentialBinaryReader<L3::LMS151>() );
        if (!LIDAR_reader->open( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/L3/LMS1xx_10420001_192.168.0.51.lidar" ) )
            throw std::exception();
        
        std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > >      poses;
        std::vector< std::pair< double, boost::shared_ptr<L3::LMS151> > >   scans;

        // Read *all* the poses
        pose_reader->read();
        pose_reader->extract( poses );

        std::cout << poses.size() << std::endl;

      
        LengthEstimatorInterface length_estimator;

        SWATHE swathe;

        L3::SE3 projection(0,0,0,.1,.2,.3);
        L3::PointCloud<double>* point_cloud = new L3::PointCloud<double>();
        std::auto_ptr< L3::Projector<double> > projector( new L3::Projector<double>( &projection, point_cloud ) );

        // Read data
        double accumulate = 0.0;
        while( LIDAR_reader->read() )
        {
            // Extract scan
            LIDAR_reader->extract( scans );
            
            // Match pose
            std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > > matched;
            
            L3::Utils::matcher< L3::SE3, L3::LMS151 > m( &poses, &matched );
            m = std::for_each( scans.begin(), scans.end(),  m );

            accumulate += length_estimator( matched[0] );
  
            swathe.push_back( std::make_pair( matched[0].second, scans[0].second ) );

            if ( accumulate > 10.0 )
            {
                // Reset
                accumulate = 0.0;

                L3::Tools::Timer t;
                t.begin();
                projector->project( swathe );
                std::cout << point_cloud->num_points << " points in " << t.end() << "s" << std::endl;

                // Reset swathe
                swathe.clear();
            }

        }

    }

    std::auto_ptr<L3::IO::BinaryReader< L3::SE3 > >                 pose_reader;
    std::auto_ptr<L3::IO::SequentialBinaryReader< L3::LMS151 > >    LIDAR_reader;

};


struct Experience
{

    Experience( L3::Dataset& dataset ) : point_cloud(NULL)
    {

        L3::SE3 calibration = L3::SE3::ZERO();
        
        try
        {
            calibration = L3::Utils::loadCalibration( "dataset/" + dataset.name()  + ".config", "LMS1xx_10420002" );
        }
        catch( L3::calibration_failure )
        {
            std::cout << "No/erroneous calibration file for " <<  dataset.name() << std::endl;
            throw std::exception(); 
        }

        std::cout << calibration << std::endl;

        // Load the poses
        INS_reader.reset( new L3::IO::BinaryReader<L3::SE3>() );
        INS_reader->open( dataset.path() + "/OxTS.ins" );
        INS_reader->read();
        INS_reader->extract( poses );

        // Load the lidar data 
        LIDAR_reader.reset( new L3::IO::BinaryReader<L3::LMS151>() );
        LIDAR_reader->open( dataset.path() + "/LMS1xx_10420002_192.168.0.50.lidar" );
        LIDAR_reader->read();
        LIDAR_reader->extract( LIDAR_data );

        // Match
        std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > > matched;
        L3::Utils::matcher< L3::SE3, L3::LMS151 > m( &poses, &matched );
        m = std::for_each( LIDAR_data.begin(), LIDAR_data.end(),  m );

        // Thread into swathe
        L3::Utils::threader<L3::SE3, L3::LMS151>  t;
        std::transform( matched.begin(), 
                        matched.end(), 
                        LIDAR_data.begin(), 
                        std::back_inserter( swathe ),
                        t );

        //Project
        L3::SE3 calib( 0, 0, 0, -1.57, 0, 0 ); 
        point_cloud = new L3::PointCloud<double>();
        projector = new L3::Projector<double>( &calib, point_cloud );
        projector->project( swathe );
    }
    
    ~Experience()
    {
        delete point_cloud;
        delete projector;
    }

    L3::Projector<double>*  projector;
    L3::PointCloud<double>* point_cloud;

    SWATHE swathe;

    std::auto_ptr<L3::IO::BinaryReader< L3::SE3 > > INS_reader;
    std::auto_ptr<L3::IO::BinaryReader< L3::LMS151 > > LIDAR_reader;
        
    std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > > poses;
    std::vector< std::pair< double, boost::shared_ptr<L3::LMS151> > > LIDAR_data;

};

}

#endif

