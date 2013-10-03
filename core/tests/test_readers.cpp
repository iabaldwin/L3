#include <iostream>
#include "Reader.h"
#include "Writer.h"
#include "Configuration.h"
#include "Utils.h"

int main( int argc, char* argv[] )
{

    if ( argc < 2 )
    {
        std::cerr << "Usage: " << argv[0]  << " <pose_file>" << std::endl;
        return -1;
    }

    std::auto_ptr<L3::IO::BinaryReader< L3::SE3 > > reader( new L3::IO::BinaryReader<L3::SE3>() );
    
    reader->open( std::string(argv[1]) );
    
    reader->read();

    std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > > poses;
    if ( reader->extract( poses ) )
    {
        std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > >::iterator it =  poses.begin();
        
        while( it != poses.end() )
        {
            std::cout << it->first << ", " << *it->second << std::endl;

            it++;
        }
    }

}
