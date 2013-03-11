#include <iostream>
#include "L3.h"

int main()
{
    std::auto_ptr<L3::IO::BinaryReader< L3::SE3 > > INS_reader( new L3::IO::BinaryReader<L3::SE3>() );
    INS_reader->open( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/L3/OxTS.ins" );
    INS_reader->read();
    std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > > poses;
    INS_reader->extract( poses );

    std::auto_ptr<L3::IO::BinaryReader< L3::LMS151 > > LIDAR_reader( new L3::IO::BinaryReader<L3::LMS151>() );
    LIDAR_reader->open( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/L3/LMS1xx_10420001_192.168.0.51.lidar" );
    LIDAR_reader->read();
    std::vector< std::pair< double, boost::shared_ptr<L3::LMS151> > > LIDAR_data;
    LIDAR_reader->extract( LIDAR_data );

    std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > > matched;

    L3::Utils::matcher< L3::SE3, L3::LMS151 > m( &poses, &matched );

    m = std::for_each( LIDAR_data.begin(), LIDAR_data.end(),  m );

    std::cout << matched.size() << ":" << LIDAR_data.size() << std::endl;

    L3::Utils::threader<L3::SE3, L3::LMS151>  t;

    SWATHE s;

    std::transform( matched.begin(), 
                    matched.end(), 
                    LIDAR_data.begin(), 
                    std::back_inserter( s ),
                    t );
    
    L3::SE3 projection(0,0,0,.1,.2,.3);
    L3::PointCloud<double>* point_cloud = new L3::PointCloud<double>();
    std::auto_ptr< L3::Projector<double> > projector( new L3::Projector<double>( &projection, point_cloud ) );

    projector->project( s );

    L3::PointCloud<double> sampled_cloud = L3::samplePointCloud( point_cloud, 10000 );

    L3::writePCLASCII( "test.pcd", sampled_cloud );

}
