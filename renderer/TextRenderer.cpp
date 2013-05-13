#include "TextRenderer.h"

namespace L3
{
namespace Visualisers
{
    Text3D::Text3D()
    {
        glfInit();
        
        font_descriptor = glfLoadFont( (char*)"/Users/ian/code/thirdparty/glf_distr/fonts/courier1.glf" );
   
        glfSetCurrentFont(font_descriptor);
    }

    void Text3D::setText( std::string text )
    {
        this->text = text;
    }

    void Text3D::onDraw3D( glv::GLV& g )
    {
        
        float xmin,ymin,xmax,ymax;
        
        glfGetStringBounds( const_cast<char*>(text.c_str()), &xmin, &ymin, &xmax, &ymax);
        
        glColor3f(1.0,0.2,0.3);
        glPushMatrix();
        glScalef(5.0,5.0,1.0);
        glfDrawSolidString( const_cast<char*>(text.c_str() ) );
        glPopMatrix();
        

    }



}
}
