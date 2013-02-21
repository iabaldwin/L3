#include "WindowerFactory.h"
#include "Datatypes.h"
#include "Definitions.h"

int main()
{
    std::cout.precision( 12 );
    

    L3::Tools::Timer t;

    for ( int i=100; i < 2000; i+= 50 )
    {
        double time = 1328534146.40;

        boost::shared_ptr<L3::SlidingWindow< L3::LMS151 > > window = L3::WindowerFactory<L3::LMS151>::constantTimeWindow( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/L3/LMS1xx_10420001_192.168.0.51.lidar", 100  );
        
        window->STACK_SIZE = i;
        window->initialise();

        t.begin();
      
        int j;
        for ( j=0; j<20; j++ )
        {
            window->read();
        }
        
        std::cout << window->STACK_SIZE << " entries in an average of " << t.end()/(double)j << "s" << std::endl;
    }
}

