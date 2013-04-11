#ifndef L3_TOOLS_H
#define L3_TOOLS_H

#include <sys/time.h>
#include <ctime>

namespace L3
{
namespace Tools
{

    struct Timer
    {
        clock_t tStart;
        
        void begin()
        {
            tStart = clock();
        }

        double end()
        {
            return (double)(clock() - tStart)/CLOCKS_PER_SEC;
        }

        double elapsed()
        {
            return end();
        }
    };

    
} // Tools  
} // L3

#endif
