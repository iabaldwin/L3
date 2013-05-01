#include <iostream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>


#include "L3.h"
#include "Visualisers.h"
#include "VisualiserRunner.h"
#include "Components.h"

struct Cube : L3::Visualisers::Leaf
{

    
    void onDraw3D( glv::GLV& g)
    {
        glv::Point3 points[1*4*6];
        glv::Color  colors[1*4*6];

        float x_delta = -10.0;
        float y_delta = -10.0;

        float x = 10.0;
        float y = 10.0;

        int counter = 0;

        // Bottom
        float z_val = 1;
        colors[counter].set( 0, 0, 1, 1 );
        points[counter++]( x+x_delta, y+y_delta, z_val );
        colors[counter].set( 0, 0, 1, 1 );
        points[counter++]( x+x_delta, y-y_delta, z_val );
        colors[counter].set( 0, 0, 1, 1 );
        points[counter++]( x-x_delta, y-y_delta, z_val );
        colors[counter].set( 0, 0, 1, 1 );
        points[counter++]( x-x_delta, y+y_delta, z_val );

        // Top
        z_val = 20;
        colors[counter].set( 0, 0, 1, 1 );
        points[counter++]( x+x_delta, y+y_delta, z_val );
        colors[counter].set( 0, 0, 1, 1 );
        points[counter++]( x+x_delta, y-y_delta, z_val );
        colors[counter].set( 0, 0, 1, 1 );
        points[counter++]( x-x_delta, y-y_delta, z_val );
        colors[counter].set( 0, 0, 1, 1 );
        points[counter++]( x-x_delta, y+y_delta, z_val );

       
        // Side 1
        float x_val = x+x_delta;
         
        colors[counter].set( 0, 0, 1, .5 );
        points[counter++]( x_val, y-y_delta, 1 );
        colors[counter].set( 0, 0, 1, .5 );
        points[counter++]( x_val, y+y_delta, 1 );
        colors[counter].set( 0, 0, 1, .5 );
        points[counter++]( x_val, y+y_delta, 20 );
        colors[counter].set( 0, 0, 1, .5 );
        points[counter++]( x_val, y-y_delta, 20 );
        
        // Side 2
        x_val = x-x_delta;
        colors[counter].set( 0, 0, 1, .5 );
        points[counter++]( x_val, y-y_delta, 1 );
        colors[counter].set( 0, 0, 1, .5 );
        points[counter++]( x_val, y+y_delta, 1 );
        colors[counter].set( 0, 0, 1, .5 );
        points[counter++]( x_val, y+y_delta, 20 );
        colors[counter].set( 0, 0, 1, .5 );
        points[counter++]( x_val, y-y_delta, 20 );
        

  
        // Front 
        float y_val = y-x_delta;
         
        colors[counter].set( 0, 0, 1, .5 );
        points[counter++]( x-x_delta, y_val, 1 );
        colors[counter].set( 0, 0, 1, .5 );
        points[counter++]( x+x_delta, y_val, 1 );
        colors[counter].set( 0, 0, 1, .5 );
        points[counter++]( x+x_delta, y_val, 20 );
        colors[counter].set( 0, 0, 1, .5 );
        points[counter++]( x-x_delta, y_val, 20 );
        
        // Back
        y_val = y+x_delta;
         
        colors[counter].set( 0, 0, 1, .5 );
        points[counter++]( x-x_delta, y_val, 1 );
        colors[counter].set( 0, 0, 1, .5 );
        points[counter++]( x+x_delta, y_val, 1 );
        colors[counter].set( 0, 0, 1, .5 );
        points[counter++]( x+x_delta, y_val, 20 );
        colors[counter].set( 0, 0, 1, .5 );
        points[counter++]( x-x_delta, y_val, 20 );
        



        glv::draw::enable( glv::draw::Blend );
        //glv::draw::paint( glv::draw::Quads, points, colors, counter );
        glv::draw::paint( glv::draw::TriangleStrip, points, colors, counter );
        glv::draw::disable( glv::draw::Blend );

    }

};

int main (int argc, char ** argv)
{
    /*
     *Visualisation
     */
    glv::GLV top;
    glv::Window win(1400, 800, "Visualisation::PointCloud");

    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    L3::Visualisers::Grid                       grid;
    L3::Visualisers::Composite                  composite;
    L3::Visualisers::BasicPanController         controller( composite.position );
  
    Cube c;

    composite.addController( dynamic_cast<L3::Visualisers::Controller*>( &controller ) ).stretch(1,1);

    top << (composite << grid << c );

    // Go
    win.setGLV(top);
    glv::Application::run();

}


