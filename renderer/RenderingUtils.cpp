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


}
}

