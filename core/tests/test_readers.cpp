#include <iostream>
#include "Reader.h"
#include "Writer.h"
#include "Configuration.h"
#include "Utils.h"

int main()
{
    std::auto_ptr<L3::IO::BinaryReader< L3::SE3 > > reader( new L3::IO::BinaryReader<L3::SE3>() );
    reader->open( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/L3/OxTS.ins" );
    
    reader->read();

    std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > > poses;
    if ( reader->extract( poses ) )
    {
        L3::Configuration::Begbroke b;

        //L3::Utils::localisePoseChain( poses.begin(), poses.end(), b );
   
        L3::IO::Writer w;
        if( w.open( "test.txt" ))
            w << poses; 
    }
}
