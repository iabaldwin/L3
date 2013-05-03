#ifndef L3_RENDERING_UTILS_H
#define L3_RENDERING_UTILS_H

#include <deque>

#include <GLV/glv.h>

#include "glut.h"

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

    void drawBitmapText(char *string,float x,float y,float z) ;

}
}
#endif

