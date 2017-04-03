#include <iostream>

#include <glv.h>
#include <glv_binding.h>
#include <glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "Components.h"

#include "libglf/glf.h"

#include <boost/filesystem.hpp>
#include <boost/shared_array.hpp>

#include <deque>

struct test_leaf : L3::Visualisers::Leaf
{

    test_leaf()
    {
        glfInit();

        boost::filesystem::directory_iterator itr( "/Users/ian/code/thirdparty/glf_distr/fonts/" );

        while ( itr != boost::filesystem::directory_iterator() )
        {
            std::string str = itr->path().string();
            fonts.push_back( std::string(str) );
            
            itr++;
        }


        font_descriptor.reset( new int[ fonts.size() ]  );
      
        for( std::deque< std::string >::iterator it = fonts.begin();
                it != fonts.end();
                it++ )
        {
            std::string load_string = (*it);

            font_descriptor[ (int)(std::distance( fonts.begin(), it )) ] = glfLoadFont( const_cast< char* >( load_string.c_str() ) );

        }

        current_font = 0;


    }
        
    std::deque < std::string > fonts;

    
    boost::shared_array< int > font_descriptor;

    int current_font;

    void onDraw3D(glv::GLV& g)
    {

        //glfSetCurrentFont(font_descriptor[0]);
        glfSetCurrentFont(font_descriptor[current_font]);
      
        static int k = 0;

        if ( (k++ % 20 ) == 0 )
        {

            std::cout << fonts[current_font] << std::endl;

            current_font++;

            if ( current_font == fonts.size() )
                current_font = 0;
        }

        
        char* THEMSG = static_cast<char*>("Hello, World\0");
        
        glPushMatrix();
        
        float xmin,ymin,xmax,ymax;
        
        glfGetStringBounds(THEMSG,&xmin,&ymin,&xmax,&ymax);
        glColor3f(1.0,0.2,0.3);
        //glScalef(10.0,10.0,10.0);
        glScalef(2.0,2.0,1.0);
        glfDrawSolidString(THEMSG);
        
        glPopMatrix();


    }
};

int main (int argc, char ** argv)
{

    glv::GLV top;
    glv::Window win(1400, 800, "Soaring");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    L3::Visualisers::Composite composite;
    L3::Visualisers::BasicPanController         controller(composite.position);
    L3::Visualisers::Grid                       grid;
    
    test_leaf leaf;
  
    top << ( composite << leaf << grid );

    composite.stretch(1,1);

    win.setGLV(top);
    glv::Application::run();
}


