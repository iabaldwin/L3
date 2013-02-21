#include "Windower.h"
#include "Datatypes.h"
#include "Definitions.h"

int main()
{
    std::cout.precision( 12 );
    
    Poco::Thread thread;

    L3::SlidingWindow<L3::Pose> w( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/L3/OxTS.ins", 100  );

    thread.start( w );

    double time = 1328534146.40;
   
    double increment = 1;
    while (true)
    {
        usleep( .1*increment*1e6 );
        w.update( time += increment );
        std::cout << time << "-->" << (*w.window.begin()).first << ":" << (*(w.window.end()-1)).first << "(" << w.window.size() << ")" << std::endl;
    } 
    
    thread.join();
}

