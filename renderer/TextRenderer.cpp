#include "TextRenderer.h"

#include <FTGL/ftgl.h>

#define FONT_FILE "/Library/Fonts/Times New Roman.ttf"
//#define FONT_FILE "/Library/Fonts/AppleMyungjo.ttf"
//#define FONT_FILE "/Library/Fonts/Courier New.ttf"
//#define FONT_FILE "/Library/Fonts/Microsoft Sans Serif.ttf"

namespace L3
{
namespace Visualisers
{
    Text3D::Text3D()
    {

        // Create a pixmap font from a TrueType file.

    }

    void Text3D::onDraw3D( glv::GLV& g )
    {
        //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        //glEnable(GL_DEPTH_TEST);
        //////float front_ambient[4]  = { 0.7, 0.7, 0.7, 0.0 };
        //////glMaterialfv(GL_FRONT, GL_AMBIENT, front_ambient);
        //////glColorMaterial(GL_FRONT, GL_DIFFUSE);
        //glColor3f(0.0, 1.0, 0.0);
        ////font[1]->Render("Hello FTGL!");

        ////void glutStrokeCharacter(void *font, int character);
        ////face->draw( 0, 0, "Hello, World!" );


        //std::string scoreString( "Duck" );

        //glutBitmapTimesRoman10(GLUT_BITMAP_HELVETICA_18, scoreString.c_str());


        //glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ////glMatrixMode(GL_MODELVIEW);
        ////glLoadIdentity();

        //glColor3ub(255,0,0);
        //glPushMatrix();
        //glScalef(5,5,5);
        //glBegin(GL_QUADS);
        //glVertex2f(-1,-1);
        //glVertex2f(1,-1);
        //glVertex2f(1,1);
        //glVertex2f(-1,1);
        //glEnd();
        //glPopMatrix();

        //glRasterPos2i(0,0);     // B

        //std::string tmp( "wha-hey!" );
        //for( size_t i = 0; i < tmp.size(); ++i )
        //{
            //glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, tmp[i]);
        //}
        
        //FTGLPixmapFont font("/home/user/Arial.ttf");
        //FTGLPixmapFont font( FONT_FILE );
       
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glPushMatrix();
        glColor3ub(0,255,0);    

        float front_ambient[4]  = { 0.7, 0.7, 0.7, 0.0 };
        glMaterialfv(GL_FRONT, GL_AMBIENT, front_ambient);
        glColorMaterial(GL_FRONT, GL_DIFFUSE);

        glTranslatef( 10, 10, 10 );

        FTGLTextureFont font( FONT_FILE );

        // If something went wrong, bail out.
        if(font.Error())
            throw std::exception();

        // Set the font size and render a small text.
        font.FaceSize(12);
        font.Render("Hello World!");


        glPopMatrix();
    }





}
}
