#include <iostream>
#include "Reader.h"

int main()
{
    std::auto_ptr<L3::IO::PoseReader> reader( new L3::IO::PoseReader() );
    reader->open( "/Users/ian/code/python/tools/test/OxTS.ins" );
   
    std::vector<L3::Pose*> poses;
    reader->read();

    if ( reader->extract( poses ) )
        for( std::vector<L3::Pose*>::iterator it=poses.begin(); it!= poses.end(); it++ )
            std::cout << *(*it) << std::endl << (*it)->getHomogeneous() << std::endl;
    else
        std::cout << "Failure" << std::endl;
}

