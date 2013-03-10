#ifndef L3_COMPONENTS_H
#define L3_COMPONENTS_H

#include <iostream>

#include <GLV/glv.h>

#include "L3.h"
#include "Controller.h"

namespace L3
{
namespace Visualisers
{

/*
 *Single entity for 3D rendering
 */
struct Component : glv::View3D
{

    Component()
    {
        /*
         *If you don't have an active
         *    view, then the onDraw
         *    methods are not called. 
         *    However, this stretches
         *    to the full size of the
         *    element.
         */
        stretch(1,1);
    }

    virtual void onDraw3D(glv::GLV& g)
    {
    }
    virtual void onDraw2D(glv::GLV& g)
    {
    }

};

/*
 *Leaf component for 3D rendering
 */
struct Leaf 
{
    virtual void onDraw3D( glv::GLV& g )
    {}

    virtual void onDraw2D( glv::GLV& g )
    {}
   
    double time;
};

/*
 * Composite renderer
 *  
 *  Render multiple leaf viewers 
 *
 */
struct Composite : glv::View3D 
{

    Composite() : current_time(0.0), 
                    sf(1), 
                    _x(0), _y(0), _z( -240.0 ), 
                    _r(0), _p(0), _q(0)
    {
        stretch(1,1); 
        
        far( 1000 );
    }

    clock_t current, previous;
    unsigned int sf;
    double current_time; 
    double _x, _y, _z;
    double _r, _p, _q;
 
    std::list<Leaf*> components; 

    void apply( control_t t )
    {
        _x += t.x;
        _y += t.y;
        _z += t.z;
        _r += t.r;
        _p += t.p;
        _q += t.q;
    }

    bool onEvent( glv::Event::t type, glv::GLV& g )
    {    
        apply( controller->onEvent( type, g ) );
    
    }

    L3::Visualisers::Controller* controller;
    void addController( L3::Visualisers::Controller* c )
    {
        controller = c; 
    }

    virtual void onDraw3D(glv::GLV& g)
    {
        std::list<Leaf*>::iterator it = components.begin();
        glv::draw::translate( _x, _y, _z );
        glv::draw::rotate( _r, _p, _q );

        // 1. Compute time since last update
        current = clock();
        double elapsed = double(current - previous)/CLOCKS_PER_SEC;

        // Takes care of initial inf.
        elapsed  = (elapsed > 1.0) ? 1.0 : elapsed;

        // 2. Increment the *time* by this value 
        current_time += (elapsed *= sf);

        while( it != components.end() )
        {
            (*it)->time = this->current_time;
            (*it)->onDraw3D( g );
            it++;
        }
 
        previous = current;
    }

    Composite& operator<<( Leaf& leaf )
    {
        components.push_back( &leaf );
        return *this;
    }
    
};

/*
 *Useful components
 *
 *      GRID
 *
 */
struct Grid : Leaf
{
    Grid()
    {
        vertices = new glv::Point3[ 100*4 ];
        colors = new glv::Color[ 100*4 ];

        counter = 0;
        float spacing = 50; 
        float lower = -500;
        float upper = 500;

        // Add horizontal
        for ( int i=lower; i<upper; i+=spacing )
        {
            vertices[counter++]( i, lower, 0);
            vertices[counter++]( i, upper, 0);
        }

        for ( int i=lower; i<upper; i+=spacing )
        {
            vertices[counter++]( lower, i , 0);
            vertices[counter++]( upper, i , 0);
        }

    }

    int counter;

    ~Grid()
    {
        delete [] vertices;
    }
        
    glv::Point3*    vertices;
    glv::Color*     colors;

    void onDraw3D(glv::GLV& g)
    { 
        glv::draw::paint( glv::draw::Lines, vertices, colors, counter );
    }
};

struct CoordinateSystem
{

    CoordinateSystem()  : _pose( L3::SE3::ZERO() )
    {
        _init();
    }

    CoordinateSystem( const L3::SE3& pose ) : _pose( pose )
    {
        _init();
    }
 
    L3::SE3 _pose;

    ~CoordinateSystem()
    {
        delete [] vertices;
    }

    void _init()
    {
        vertices = new glv::Point3[6];
        vertices[0]( 0, 0, 0 ); 
        vertices[1]( 1, 0, 0 ); 
        vertices[2]( 0, 0, 0 ); 
        vertices[3]( 0, 1, 0 ); 
        vertices[4]( 0, 0, 0 ); 
        vertices[5]( 0, 0, 1 ); 
   
        colors = new glv::Color[6];
        colors[0] = glv::Color( 1, 0, 0 ); 
        colors[1] = glv::Color( 1, 0, 0 ); 
        colors[2] = glv::Color( 0, 1, 0 ); 
        colors[3] = glv::Color( 0, 1, 0 ); 
        colors[4] = glv::Color( 0, 0, 1 ); 
        colors[5] = glv::Color( 0, 0, 1 ); 
    }

    void onDraw3D(glv::GLV& g)
    { 
        glv::draw::push();

        glv::draw::translate( _pose.x, _pose.y, _pose.z );
        glv::draw::rotate( _pose.r, _pose.q, _pose.q );
        glv::draw::paint( glv::draw::LineStrip, vertices, colors, 6 );
        
        glv::draw::pop();
    }

    glv::Color* colors;
    glv::Point3* vertices;

};

}
}



#endif
