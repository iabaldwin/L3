#include <iostream>
#include "Reader.h"
#include "Writer.h"
#include "Datatypes.h"
#include "Utils.h"

int main()
{
    std::auto_ptr<L3::IO::PoseReader> reader( new L3::IO::PoseReader() );
    reader->open( "/Users/ian/code/python/tools/test/OxTS.ins" );
   
    std::vector<L3::Pose*> poses;
    reader->read();

    if ( reader->extract( poses ) )
    {
        //L3::UTILS::localisePoseChain( poses, L3::UTILS::BEGBROKE() );
   
        L3::UTILS::localisePoseChainToOrigin( poses );

        L3::IO::Writer w;
        if( w.open( "test.txt" ))
            w << poses; 
    }
}