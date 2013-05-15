#ifndef L3_IMAGERY_H
#define L3_IMAGERY_H

#include "Common/Imagery.h"
#include "Visualisers.h"
#include "Components.h"
#include "QueryInterface.h"
#include "L3.h"

#include "boost/multi_array.hpp"

namespace L3
{
namespace Visualisers
{

    struct ImageData
    {
        IAB::Imagery::IMAGE img;
        //boost::shared_array< GLubyte > texture;
        boost::multi_array<GLubyte, 3> texture; 
        GLuint name;
    };

    struct ImageFactory
    {
        static bool Image(const std::string& image_target, ImageData& data  )
        {
            // Load the image
            //IAB::Imagery::IMAGE img;
            
            try
            {
                data.img = IAB::Imagery::loadImage( image_target );
            }
            catch (...)
            {
                std::cerr << "Could not load " << image_target << std::endl;
                return false;
            }

            //GLubyte image_texture[data.img->height][data.img->width][4];
            //data.texture.reset( new GLubyte[data.img->height][data.img->width][4] );

            boost::multi_array<GLubyte, 3>::extent_gen extents;
            data.texture.resize( extents[data.img->height][data.img->width][4] );

            for(int j=data.img->height-1; j> 0; j--)
            {
                for(int i=0; i< data.img->width;  i++)
                {    
                    GLubyte* b = &((GLubyte*)(data.img->imageData + data.img->widthStep*j))[i*3];
                    GLubyte* g = &((GLubyte*)(data.img->imageData + data.img->widthStep*j))[i*3+1];
                    GLubyte* r = &((GLubyte*)(data.img->imageData + data.img->widthStep*j))[i*3+2];

                    //image_texture[j][i][0] = (GLubyte) *r;
                    //image_texture[j][i][1] = (GLubyte) *g;
                    //image_texture[j][i][2] = (GLubyte) *b;
                    //image_texture[j][i][3] = (GLubyte) 128;

                    data.texture[j][i][0] = (GLubyte) *r;
                    data.texture[j][i][1] = (GLubyte) *g;
                    data.texture[j][i][2] = (GLubyte) *b;
                    data.texture[j][i][3] = (GLubyte) 128;


                }
            }

            glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

            //glGenTextures(1, &texName);
            //glBindTexture(GL_TEXTURE_2D, texName);

            glGenTextures(1, &data.name );
            glBindTexture(GL_TEXTURE_2D, data.name );

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, 
                    GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, 
                    GL_NEAREST);
            //glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, img->width, 
                    //img->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, 
                    //image_texture);  
       
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, data.img->width, 
                    data.img->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, 
                    data.texture.data() );  

            return true;
        }


    };

    struct ImageRenderer : SelectableLeaf
    {
        ImageRenderer( ImageData& data ) 
            : SelectableLeaf( data.img->width, data.img->height,1),
                lower_bound_x(.0f), upper_bound_x( data.img->width ),
                lower_bound_y(.0f), upper_bound_y( data.img->height ),
                z_bound(-6.0),
                data(data)
        {
        }

        ImageData& data;

        double lower_bound_x, upper_bound_x,
                lower_bound_y, upper_bound_y,
                z_bound;


        virtual ~ImageRenderer()
        {
        }

        virtual void onDraw3D( glv::GLV& g )
        {
            glEnable(GL_TEXTURE_2D);
            glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
            //glBindTexture(GL_TEXTURE_2D, texName);
            glBindTexture(GL_TEXTURE_2D, data.name );
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

    //struct LocaleRenderer : ImageRenderer, Controllable
    //{
        //LocaleRenderer( L3::Configuration::Locale& locale )
        //{



        //}

        //bool load( )
        //{
            ////std::string image = "/Users/ian/Documents/begbroke_high_res.jpg";
            //std::string image = "/Users/ian/Documents/begbroke_med_res.png" ;

            //lower_bound_x = locale.x_lower;
            //upper_bound_x = locale.x_upper;

            //lower_bound_y = locale.y_lower;
            //upper_bound_y = locale.y_upper;

            ////upper_bound_x = upper_bound_x - lower_bound_x;
            //upper_bound_x = 1000*.125*4; 
            //lower_bound_x = 0.0;

            ////upper_bound_y = upper_bound_y - lower_bound_y;
            //upper_bound_y = 1000*.125*4; 
            //lower_bound_y = 0.0;

            ////return ImageRenderer::load( image );
        //}

    //};
}
}

#endif

