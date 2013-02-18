#include "Windower.h"
#include "Datatypes.h"
#include "Definitions.h"

int main()
{
    std::cout.precision( 12 );
    
    Poco::Thread thread;

    L3::SlidingWindow<L3::Pose> w( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/L3/OxTS.ins", 100  );

    thread.start( w );

    double start = 1328534146.406440019607543945;

    for( double i=start; i<(start+10000); i+= 1.0 )
    {
        w.update( i );
        usleep( 100000 );
        std::cout << i << "-->" << (*w.window.begin()).first << ":" << (*(w.window.end()-1)).first << "(" << w.window.size() << ")" << std::endl;
    }
    
    thread.join();
}

