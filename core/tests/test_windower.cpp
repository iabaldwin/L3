#include "Windower.h"
#include "WindowerFactory.h"
#include "Datatypes.h"
#include "Definitions.h"

int main()
{
    std::cout.precision( 12 );
    
    Poco::Thread thread;

    //boost::shared_ptr< L3::SlidingWindow<L3::SE3> > w = L3::WindowerFactory<L3::SE3>::constantTimeWindow( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/L3/OxTS.ins", 100  );
    boost::shared_ptr< L3::SlidingWindow<L3::LMS151> > w = L3::WindowerFactory<L3::LMS151>::constantTimeWindow( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/L3/LMS1xx_10420001_192.168.0.51.lidar", 100  );

    w->initialise();

    thread.start( *w );

    double time = 1328534146.40;

    double increment = 1;
    while (true)
    {
        //usleep( .1*1e06) ;
        assert( w->update( time += increment ) );
        std::cout << time << "-->" << (*w->window.begin()).first << ":" << (*(w->window.end()-1)).first << "(" << w->window.size() << ")" << std::endl;
    } 
    
    thread.join();
}

