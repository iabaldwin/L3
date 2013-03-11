#include <iostream>
#include "Reader.h"
#include "Utils.h"

int main()
{
    std::auto_ptr<L3::IO::SequentialBinaryReader< L3::SE3 > > reader( new L3::IO::SequentialBinaryReader<L3::SE3>() );
    if (!reader->open( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/L3/OxTS.ins" ))
        throw std::exception();
   
    std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > > poses;
   
    int counter = 0;
    std::cout.precision(15);
    while( reader->read() )
    {
        reader->extract( poses );
        std::cout << counter++ << ":" << poses[0].first << ":" <<  poses.size() << std::endl;     
    }
}
