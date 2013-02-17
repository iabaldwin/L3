#include <iostream>
#include "Reader.h"
#include "Definitions.h"

int main()
{
    std::auto_ptr<L3::IO::PoseReader> reader( new L3::IO::PoseReader() );
    reader->open( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/L3/OxTS.ins" );
   
    POSE_SEQUENCE poses;
    reader->read();

    if ( reader->extract( poses ) )
        for( POSE_SEQUENCE_ITERATOR it=poses.begin(); it!= poses.end(); it++ )
            std::cout << *(*it).second << std::endl << (*it).second->getHomogeneous() << std::endl;
    else
        std::cout << "Failure" << std::endl;
}

