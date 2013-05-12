#include <iostream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "Components.h"
#include "GLVInterface.h"
#include "CommandInterface.h"


struct test_leaf : L3::Visualisers::Leaf
{
    void onDraw3D(glv::GLV& g)
    {
        glv::draw::translateZ( -400 );
        glv::Point3 pts[1000];
        glv::Color colors[1000];

        for( int i=0; i<1000; i++ )
            pts[i]( random()%100-50, random()%100-50, random()%100-50 );

        glv::draw::paint( glv::draw::Points, pts, colors, 1000 );
    
    }
};

/*
 *  Custom GLV view
 */
struct CustomGLV : glv::GLV
{
    bool onEvent( glv::Event::t e, glv::GLV& g)
    {
        glv::space_t a =0.0f;
        glv::space_t b =0.0f;
        
        if ( e == glv::Event::KeyDown )
        {
            const glv::Keyboard& k = g.keyboard();
            switch (k.key())
            {
                case 96:
                    this->broadcastEvent( static_cast< glv::Event::t>( 20 ) );
                    // This is quite grim
                    this->setMouseDown(a, b, 1, 1);
                    this->setMouseUp(a, b, 1, 1);
                default:
                    break; 
            }

        }
    }
};

int main (int argc, char ** argv)
{
    boost::shared_ptr< glv::View >  glv_interface( new L3::Visualisers::GLVInterface( glv::Rect(1200,800,200,150) ) ) ;

    L3::Interface* lua = new L3::LuaInterface();
    L3::Interface* l3 = new L3::CommandInterface( NULL, boost::shared_ptr< L3::Container >());

    (*dynamic_cast< L3::Visualisers::GLVInterface* >( glv_interface.get() ) ) << lua << l3;

    //glv::GLV top;
    CustomGLV top;
    glv::Window win(1400, 800, "L3::Component");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    L3::Visualisers::Composite                  composite;
    L3::Visualisers::BasicPanController         controller(composite.position);
    L3::Visualisers::Grid                       grid;

    composite.addController( dynamic_cast<L3::Visualisers::Controller*>( &controller ) ).stretch(1,1);

    test_leaf leaf;

    top << ( composite << grid << leaf  ) << (*glv_interface);

    win.setGLV(top);
    glv::Application::run();
}


