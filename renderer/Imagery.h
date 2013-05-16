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
        //GLuint name;
    };

    struct ImageFactory
    {
        static bool Image(const std::string& image_target, ImageData& data  )
        {
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
            
            ////boost::shared_ptr< boost::multi_array<GLubyte, 3> > texture; 
            ////data.texture.reset( new boost::multi_array<GLubyte, 3> );
            ////boost::multi_array<GLubyte, 3>::extent_gen extents;
            ////data.texture->resize( extents[data.img->height][data.img->width][4] );

            //for(int j=data.img->height-1; j> 0; j--)
            //{
                //for(int i=0; i< data.img->width;  i++)
                //{    
                    //GLubyte* b = &((GLubyte*)(data.img->imageData + data.img->widthStep*j))[i*3];
                    //GLubyte* g = &((GLubyte*)(data.img->imageData + data.img->widthStep*j))[i*3+1];
                    //GLubyte* r = &((GLubyte*)(data.img->imageData + data.img->widthStep*j))[i*3+2];

                    //image_texture[j][i][0] = (GLubyte) *r;
                    //image_texture[j][i][1] = (GLubyte) *g;
                    //image_texture[j][i][2] = (GLubyte) *b;
                    //image_texture[j][i][3] = (GLubyte) 128;

                    ////(*data.texture)[j][i][0] = (GLubyte) *r;
                    ////(*data.texture)[j][i][1] = (GLubyte) *g;
                    ////(*data.texture)[j][i][2] = (GLubyte) *b;
                    ////(*data.texture)[j][i][3] = (GLubyte) 128;
                //}
            //}

            //glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

            ////glGenTextures(1, &texName);
            ////glBindTexture(GL_TEXTURE_2D, texName);

            //glGenTextures(1, &data.name );
            //glBindTexture(GL_TEXTURE_2D, data.name );

            //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
            //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            
            ////glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, img->width, 
                    ////img->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, 
                    ////image_texture);  

            //glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, data.img->width, 
                    //data.img->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, 
                    //image_texture);  


            ////glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, data.img->width, 
                    ////data.img->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, 
                    ////data.texture.data() );  

            return true;
        }


    };

    struct image_bounds
    {
       
        image_bounds() : lower_x(.0f), upper_x( .0f),
                    lower_y(.0f), upper_y( .0f ),
                    z_bound(-6.0)
        {
        }


        double lower_x, upper_x, 
                  lower_y, upper_y,
                  z_bound;
    };


    struct ImageRenderer 
    {
        ImageRenderer( ImageData& data, image_bounds b )  : b(b), data(data)
        {
            GLubyte image_texture[data.img->height][data.img->width][4];
            
            //boost::shared_ptr< boost::multi_array<GLubyte, 3> > texture; 
            //data.texture.reset( new boost::multi_array<GLubyte, 3> );
            //boost::multi_array<GLubyte, 3>::extent_gen extents;
            //data.texture->resize( extents[data.img->height][data.img->width][4] );

            for(int j=data.img->height-1; j> 0; j--)
            {
                for(int i=0; i< data.img->width;  i++)
                {    
                    GLubyte* b = &((GLubyte*)(data.img->imageData + data.img->widthStep*j))[i*3];
                    GLubyte* g = &((GLubyte*)(data.img->imageData + data.img->widthStep*j))[i*3+1];
                    GLubyte* r = &((GLubyte*)(data.img->imageData + data.img->widthStep*j))[i*3+2];

                    image_texture[j][i][0] = (GLubyte) *r;
                    image_texture[j][i][1] = (GLubyte) *g;
                    image_texture[j][i][2] = (GLubyte) *b;
                    image_texture[j][i][3] = (GLubyte) 128;

                    //(*data.texture)[j][i][0] = (GLubyte) *r;
                    //(*data.texture)[j][i][1] = (GLubyte) *g;
                    //(*data.texture)[j][i][2] = (GLubyte) *b;
                    //(*data.texture)[j][i][3] = (GLubyte) 128;
                }
            }

            glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

            //glGenTextures(1, &texName);
            //glBindTexture(GL_TEXTURE_2D, texName);

            //glGenTextures(1, &data.name );
            //glBindTexture(GL_TEXTURE_2D, data.name );

            glGenTextures(1, &name );
            glBindTexture(GL_TEXTURE_2D, name );


            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            
            //glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, img->width, 
                    //img->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, 
                    //image_texture);  

            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, data.img->width, 
                    data.img->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, 
                    image_texture);  


            //glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, data.img->width, 
                    //data.img->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, 
                    //data.texture.data() );  
        }
        
        GLuint name;

        image_bounds b;
        ImageData& data;

        virtual ~ImageRenderer()
        {
        }

        virtual void onDraw3D( glv::GLV& g )
        {
            glEnable(GL_TEXTURE_2D);
            glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
            //glBindTexture(GL_TEXTURE_2D, texName);
            //glBindTexture(GL_TEXTURE_2D, data.name );
            glBindTexture(GL_TEXTURE_2D, name );
            glBegin(GL_QUADS);

            glTexCoord2f(0.0, 0.0); glVertex3f(b.lower_x, b.upper_y, b.z_bound);
            glTexCoord2f(1.0, 0.0); glVertex3f(b.upper_x, b.upper_y, b.z_bound);
            glTexCoord2f(1.0, 1.0); glVertex3f(b.upper_x, b.lower_y, b.z_bound);
            glTexCoord2f(0.0, 1.0); glVertex3f(b.lower_x, b.lower_y, b.z_bound);

            glEnd();
            glFlush();
            glDisable(GL_TEXTURE_2D);
      
            glv::Point3 vertices[4];
            glv::Color  colors[4];


            vertices[0]( b.lower_x, b.upper_y, b.z_bound );
            vertices[1]( b.upper_x, b.upper_y, b.z_bound );
            vertices[2]( b.upper_x, b.lower_y, b.z_bound );
            vertices[3]( b.lower_x, b.lower_y, b.z_bound );


            glv::draw::paint( glv::draw::LineLoop, vertices, colors, 4 );
        
        }
    };


    struct LocaleRenderer : SelectableLeaf, Controllable 
    {
        LocaleRenderer( ImageData data, image_bounds b ) : SelectableLeaf( 100,100,100)
        {
            image_renderer.reset( new ImageRenderer( data, b ) );
        }

        boost::shared_ptr< ImageRenderer > image_renderer;

        void onDraw3D(glv::GLV& g )
        {
            //if( !image_renderer->data.initalised )
                //image_renderer->data.initalise();

            image_renderer->onDraw3D(g);
        }

    };

    struct LocaleRendererFactory
    {

        static boost::shared_ptr< LocaleRenderer > build( L3::Configuration::Locale& locale )        
        {
            // This should be embedded in the configuration     
            std::string image_target = "/Users/ian/Documents/begbroke_med_res.png" ;

            ImageData data;
            ImageFactory::Image(image_target, data);

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

            image_bounds b;

            b.upper_x = 1000.0;
            b.upper_y = 1000.0;

            return boost::make_shared<LocaleRenderer>( data, b  );
        }

    };

}
}

#endif

