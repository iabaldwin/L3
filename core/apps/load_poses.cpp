#include <iostream>
#include "Reader.h"

int main()
{
    std::auto_ptr<L3::IO::PoseReader> reader( new L3::IO::PoseReader() );
    reader->open( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/L3/OxTS.ins" );
   
    std::vector<L3::Pose*> poses;
    reader->read();

    std::cout.precision(10);

    if ( reader->extract( poses ) )
        for( std::vector<L3::Pose*>::iterator it=poses.begin(); it!= poses.end(); it++ )
            std::cout << *(*it) << std::endl;
}

