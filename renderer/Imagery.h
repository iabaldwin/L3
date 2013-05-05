#ifndef L3_IMAGERY_H
#define L3_IMAGERY_H

#include "Common/Imagery.h"
#include "Visualisers.h"

namespace L3
{
namespace Visualisers
{
    
    struct LocaleImage : L3::Visualisers::Leaf
    {

        LocaleImage( const std::string& locale )
        {
            IAB::Imagery::IMAGE img;
            try
            {
                img = IAB::Imagery::loadImage( locale );
            }
            catch (...)
            {
                std::cerr << "Could not load " << locale << std::endl;
            }

            GLubyte image_texture[img->height][img->width][4];

            for(int j=img->height-1; j> 0; j--)
            {
                for(int i=0; i< img->width;  i++)
                {    
                    GLubyte* b = &((GLubyte*)(img->imageData + img->widthStep*j))[i*3];
                    GLubyte* g = &((GLubyte*)(img->imageData + img->widthStep*j))[i*3+1];
                    GLubyte* r = &((GLubyte*)(img->imageData + img->widthStep*j))[i*3+2];

                    image_texture[j][i][0] = (GLubyte) *r;
                    image_texture[j][i][1] = (GLubyte) *g;
                    image_texture[j][i][2] = (GLubyte) *b;
                    image_texture[j][i][3] = (GLubyte) 128;

                }
            }

            
            glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

            glGenTextures(1, &texName);
            glBindTexture(GL_TEXTURE_2D, texName);

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, 
                    GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, 
                    GL_NEAREST);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, img->width, 
                    img->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, 
                    image_texture);  
        }

        GLuint texName;

        void onDraw3D( glv::GLV& g )
        {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            glEnable(GL_TEXTURE_2D);
            glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
            glBindTexture(GL_TEXTURE_2D, texName);
            glBegin(GL_QUADS);

            glTexCoord2f(0.0, 0.0); glVertex3f(-100.0, 100.0, 0.0);
            glTexCoord2f(1.0, 0.0); glVertex3f(100.0, 100.0, 0.0);
            glTexCoord2f(1.0, 1.0); glVertex3f(100.0, -100.0, 0.0);
            glTexCoord2f(0.0, 1.0); glVertex3f(-100.0, -100.0, 0.0);

            glEnd();
            glFlush();
            glDisable(GL_TEXTURE_2D);

        }
    };

}
}

#endif

