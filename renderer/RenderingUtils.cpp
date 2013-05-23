#include "RenderingUtils.h"

#include <iostream>

namespace L3
{
namespace Visualisers
{

    void drawBitmapText(char *string,float x,float y,float z) 
    {  
        char *c;
        //glRasterPos3f(x, y,z);

        glv::draw::enter2D( 1000, 600);
        
        glLoadIdentity();
        for (c=string; *c != '\0'; c++) 
        {
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, *c);
        }
    }

    glv::Color ColorInterpolator::operator()( double value )
    {
        
        std::pair< glv::Color, glv::Color > colors = map->getBounds( value );

        double r = ( 1-value)*colors.first.r + value*colors.second.r;
        double g = ( 1-value)*colors.first.g + value*colors.second.g;
        double b = ( 1-value)*colors.first.b + value*colors.second.b;

        return glv::Color(r,g,b,value);
    }

}
}

