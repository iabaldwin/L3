#include <iostream>
#include "Reader.h"
#include "Utils.h"

int main()
{
    std::auto_ptr<L3::IO::BulkReader< L3::Pose > > reader( new L3::IO::BulkReader<L3::Pose>() );
    reader->open( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/L3/OxTS.ins" );
   
    std::vector< std::pair< double, boost::shared_ptr<L3::Pose> > > poses;
    reader->read();

    if ( reader->extract( poses ) )
    {
        L3::Utils::localisePoseChain( poses.begin(), poses.end(), L3::Utils::BEGBROKE() );
   
        L3::Utils::localisePoseChainToOrigin( poses );

        //L3::IO::Writer w;
        //if( w.open( "test.txt" ))
            //w << poses; 
    }
}
