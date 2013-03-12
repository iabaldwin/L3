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

struct experience_section
{
    int id;
    double x,y;
    unsigned int stream_position;
    unsigned int payload_size;
};

std::ostream& operator<<( std::ostream& o, experience_section section )
{
    o << section.id << ":" << section.x << "," << section.y << "(" << section.stream_position << "," << section.payload_size << ")";

    return o;
}


/*
 *Load an experience from file
 */
struct ExperienceLoader
{

    std::vector<experience_section> sections;

    ExperienceLoader()
    {
 
        std::ifstream experience_data( "experience.dat", std::ios::binary );
        std::ifstream experience_index( "experience.index", std::ios::binary );

        experience_section section;

        while( experience_index.good() )
        {
            experience_index.read( (char*)(&section.id), sizeof(int) ); 
            experience_index.read( (char*)(&section.x), sizeof(double) );
            experience_index.read( (char*)(&section.y), sizeof(double) );
            experience_index.read( (char*)(&section.stream_position), sizeof(unsigned int) );
            experience_index.read( (char*)(&section.payload_size), sizeof(unsigned int) );

            sections.push_back( section );
        }

        experience_data.close();
        experience_index.close();

    }

};

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

        LengthEstimatorInterface length_estimator;

        SWATHE swathe;

        // Calibration Point/generation
        L3::SE3 projection(0,0,0,.1,.2,.3);
        L3::PointCloud<double>* point_cloud = new L3::PointCloud<double>();
        std::auto_ptr< L3::Projector<double> > projector( new L3::Projector<double>( &projection, point_cloud ) );

        int id = 0;
        double accumulate = 0.0;
      
        std::ofstream experience_data( "experience.dat", std::ios::binary );
        std::ofstream experience_index( "experience.index", std::ios::binary );

        unsigned int stream_position=0;

        // Read data
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
                //t.begin();
                projector->project( swathe );
                //std::cout << point_cloud->num_points << " points in " << t.end() << "s" << std::endl;

                std::pair<double,double> means = mean( point_cloud );

                std::cout << means.first << " " << means.second << std::endl;
              
                //  Writing : DATA
                //  1. Write points 
                unsigned int payload_size = point_cloud->num_points*sizeof(L3::Point<double>);
                experience_data.write( (char*)( point_cloud->points ), payload_size );

                std::cout << stream_position << std::endl;

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

