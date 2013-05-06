#ifndef L3_IMAGERY_H
#define L3_IMAGERY_H

#include "Common/Imagery.h"
#include "Visualisers.h"
#include "L3.h"

namespace L3
{
namespace Visualisers
{
    
    struct ImageRenderer : L3::Visualisers::Leaf
    {

        ImageRenderer( ) 
            : lower_bound_x( -100.0f), upper_bound_x( 100.0f ),
            lower_bound_y( -100.0f), upper_bound_y( 100.0f ),
            z_bound(-6.0)
        {
        }

        GLuint texName;

        double lower_bound_x, upper_bound_x,
                lower_bound_y, upper_bound_y,
                z_bound;

        bool load(const std::string& image )
        {
            // Load the image
            IAB::Imagery::IMAGE img;
            
            try
            {
                img = IAB::Imagery::loadImage( image );
            }
            catch (...)
            {
                std::cerr << "Could not load " << image << std::endl;
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

        virtual ~ImageRenderer()
        {
        }

        virtual void onDraw3D( glv::GLV& g )
        {
            glEnable(GL_TEXTURE_2D);
            glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
            glBindTexture(GL_TEXTURE_2D, texName);
            glBegin(GL_QUADS);

            glTexCoord2f(0.0, 0.0); glVertex3f(lower_bound_x, upper_bound_y, z_bound);
            glTexCoord2f(1.0, 0.0); glVertex3f(upper_bound_x, upper_bound_y, z_bound);
            glTexCoord2f(1.0, 1.0); glVertex3f(upper_bound_x, lower_bound_y, z_bound);
            glTexCoord2f(0.0, 1.0); glVertex3f(lower_bound_x, lower_bound_y, z_bound);

            glEnd();
            glFlush();
            glDisable(GL_TEXTURE_2D);

        }
    };

    struct LocaleRenderer : ImageRenderer
    {
        bool load( L3::Configuration::Locale& locale )        
        {
            std::string image = "/Users/ian/Documents/begbroke.png" ;

            lower_bound_x = locale.x_lower;
            upper_bound_x = locale.x_upper;

            lower_bound_y = locale.y_lower;
            upper_bound_y = locale.y_upper;

            double mean_x= ((lower_bound_x + upper_bound_x)/2.0);
            double mean_y= ((lower_bound_y + upper_bound_y)/2.0);

            lower_bound_x -= mean_x;
            lower_bound_y -= mean_y;

            upper_bound_x -= mean_x;
            upper_bound_y -= mean_y;

            std::cout << lower_bound_x <<":"<< lower_bound_y << " " << upper_bound_x << ":" <<upper_bound_y;

            return ImageRenderer::load( image );
        }

    };
}
}

#endif

