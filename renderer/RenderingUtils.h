#ifndef L3_RENDERING_UTILS_H
#define L3_RENDERING_UTILS_H

#include <GLV/glv.h>

namespace L3
{
namespace Visualisers
{

struct ColorCycler
{
    ColorCycler() : counter(0)
    {
        colors.push_back( glv::Color( 1, 0, 0 ) );
        colors.push_back( glv::Color( 0, 1, 0 ) );
        colors.push_back( glv::Color( 0, 0, 1 ) );
    }

    int counter;

    std::deque< glv::Color > colors;

    glv::Color operator()()
    {
        return colors[counter++%3];
    }

};

}
}
#endif

