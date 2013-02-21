#include "Windower.h"
#include "Datatypes.h"
#include "Definitions.h"

int main()
{
    std::cout.precision( 12 );
    

    L3::Tools::Timer t;

    for ( int i=100; i < 2000; i+= 50 )
    {
        double time = 1328534146.40;

        L3::SlidingWindowBinary< L3::LMS151 > w( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/L3/LMS1xx_10420001_192.168.0.51.lidar", 100  );
        w.STACK_SIZE = i;

        w.initialise();

        t.begin();
      
        int j;
        for ( j=0; j<20; j++ )
        {
            w.read();
        }
        
        std::cout << w.STACK_SIZE << " entries in an average of " << t.end()/(double)j << "s" << std::endl;
    }
}

