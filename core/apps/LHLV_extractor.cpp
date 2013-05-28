#include <sstream>
#include "L3.h"

int main( int argc, char** argv )
{
    if ( argc < 2 ) 
    {
        std::cerr << "Usage: " << argv[0] << " <char::dataset>" << std::endl;
        exit(-1);
    }

    char* dataset_directory = argv[1];

    /*
     *  L3
     */
    L3::Dataset dataset( dataset_directory );
    std::cout << dataset << std::endl;

    // Configuration
    L3::Configuration::Mission* mission = new L3::Configuration::Mission( dataset ) ;
    std::cout << mission->horizontal << std::endl;


    // Read all the velocities
    std::cout.precision(15);

    // Pose reader
    boost::shared_ptr<L3::IO::BinaryReader< L3::LHLV > > LHLV_reader;
    boost::shared_ptr<L3::IO::SequentialBinaryReader< L3::LMS151 > > LIDAR_reader;
    LHLV_reader.reset( new L3::IO::BinaryReader<L3::LHLV>() );

    if (!LHLV_reader->open( dataset.path() + "/OxTS.lhlv" ) )
        throw std::exception();

    LIDAR_reader.reset( new L3::IO::SequentialBinaryReader<L3::LMS151>() );
    if (!LIDAR_reader->open( dataset.path() + "/LMS1xx_10420002_192.168.0.50.lidar" ) )
        throw std::exception();

    std::vector< std::pair< double, boost::shared_ptr<L3::LHLV > > >    velocities;
    std::vector< std::pair< double, boost::shared_ptr<L3::LMS151> > > scans;
    
    // Read *all* the velocities
    LHLV_reader->read();
    LHLV_reader->extract( velocities );
    
    std::vector< std::pair< double, boost::shared_ptr<L3::LHLV > > >    matched_velocities;

    // Read LIDAR data, element at a time
    while( LIDAR_reader->read() )
    {
        LIDAR_reader->extract( scans );
        // Match velocities
        std::vector< std::pair< double, boost::shared_ptr<L3::LHLV > > > matched;

        L3::Utils::matcher< L3::LHLV, L3::LMS151 > m( &velocities, &matched );

        m = std::for_each( scans.begin(), scans.end(),  m );

        matched_velocities.push_back( matched[0] );
    }

    std::cout << velocities.size() << std::endl;
    std::cout << matched_velocities.size() << std::endl;

    L3::IO::BinaryWriter< L3::LHLV > writer;

    if( !writer.open( "/Users/ian/code/test.dat" ) )
        std::cerr << "Could not open!" << std::endl;

    std::cout << writer.write( matched_velocities ) << std::endl;


}
