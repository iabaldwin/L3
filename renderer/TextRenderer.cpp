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
   
        float currentColor[4];
        glGetFloatv(GL_CURRENT_COLOR,currentColor);

        glColor3f(1.0,0.2,0.3);
        glPushMatrix();
        glScalef(5.0,5.0,1.0);
        glfDrawSolidString( const_cast<char*>(text.c_str() ) );
        glPopMatrix();
       
        glColor4f( currentColor[0], currentColor[1], currentColor[2], currentColor[3] );

    }

    /*
     *Label renderer
     */

    void LeafLabel::onDraw3D( glv::GLV& g )
    {
        if ( leaf->tag.x == 0.0  || leaf->tag.y == 0.0 )
            return;

        glPushMatrix();
        glRotatef( 90, 1, 0, 0 );
        //glTranslatef( leaf->tag.x, 20, leaf->tag.y);
        glTranslatef( leaf->tag.x, 20, -1*leaf->tag.y);
        Text3D::setText( leaf->tag.text );
        Text3D::onDraw3D( g );
        glPopMatrix();

        // Draw a bounding box, transparency, etc.


    }

}
}
