#include "Dataset.h"
#include "Iterator.h"
#include "ChainBuilder.h"

int main()
{
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );

    if( !(dataset.validate() && dataset.load() ) )
        throw std::exception();

    // Constant time iterator over poses
    L3::ConstantTimeIterator< L3::LHLV > iterator( dataset.LHLV_reader, 100.0 );

    double time = dataset.start_time;
        
    std::cout.precision(15);

    L3::ChainBuilder builder;
    
    // Run
    while (true)
    {
        usleep( .1*1e6 );
        if ( !iterator.update( time += 1 ) )
            throw std::exception();

        builder.build( iterator.window );

        std::cout << time << "-->" << iterator.window.front().first << ":" << iterator.window.back().first << ":" << iterator.window.back().first - iterator.window.front().first <<  "(" << iterator.window.size() << ")" << std::endl;
    
    } 

}
