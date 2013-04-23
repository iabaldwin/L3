#include "L3.h"


int main()
{
    try
    {
        L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/" );

        if( !(dataset.validate() && dataset.load() ) )
            throw std::exception();

        // Constant time iterator over LHLV data
        L3::ConstantTimeIterator< L3::LHLV > iterator( dataset.LHLV_reader );

        double time = dataset.start_time;

        std::cout.precision(15);

        L3::ChainBuilder builder( &iterator );

        L3::Timing::SysTimer t;

        // Run
        while (true)
        {
            usleep( .1*1e6 );
            t.begin();
            if ( !builder.update( time += 1 ) )
                throw std::exception();
            
            std::cout << time << "-->" << iterator.window.front().first << ":" << iterator.window.back().first << ":" << iterator.window.back().first - iterator.window.front().first <<  "(" << iterator.window.size() << ")" << std::endl;
        } 

    }
    catch( L3::no_such_folder& e )
    {
        std::cerr << "No such folder" << std::endl;
    }

}
