#include <iostream>
#include "Reader.h"

int main()
{
    std::auto_ptr<L3::IO::LIDARReader> reader( new L3::IO::LIDARReader() );
    reader->open( "/Users/ian/code/python/tools/LMS1xx_10420001_192.168.0.51/ranges.txt" );
   
    std::vector<L3::LMS151*> scans;
    reader->read();

    if ( reader->extract( scans ) )
        for( std::vector<L3::LMS151*>::iterator it=scans.begin(); it!= scans.end(); it++ )
            std::cout << *(*it) << std::endl;
}

