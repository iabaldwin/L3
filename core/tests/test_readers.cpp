#include <iostream>
#include "Reader.h"
#include "Utils.h"

int main()
{
    std::auto_ptr<L3::IO::BinaryReader< L3::SE3 > > reader( new L3::IO::BinaryReader<L3::SE3>() );
    reader->open( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/L3/OxTS.ins" );
    
    reader->read();
    reader->extract();

    //std::vector< std::pair< double, boost::shared_ptr<L3::Pose> > > poses;
    //if ( reader->extract( poses ) )
    //{
        //L3::Utils::localisePoseChain( poses.begin(), poses.end(), L3::Utils::BEGBROKE() );
   
        //L3::Utils::localisePoseChainToOrigin( poses );

        ////L3::IO::Writer w;
        ////if( w.open( "test.txt" ))
            ////w << poses; 
    //}
}
