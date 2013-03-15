#ifndef L3_COMPONENTS_H
#define L3_COMPONENTS_H

#include <iostream>

#include <GLV/glv.h>

#include "Controller.h"
#include "L3.h"

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
                    _x(0), _y(0), _z( -500.0 ), 
                    _r(0), _p(0), _q(0),
                    controller(NULL)
    {
        stretch(1,1); 
        
        far( 1000 );
    }

    
    double current_time; 
    unsigned int sf;
    double _x, _y, _z;
    double _r, _p, _q;
    L3::Visualisers::Controller* controller;
 
    clock_t current, previous;
    
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
        if (controller)
            apply( controller->onEvent( type, g ) );
  
        return true;
    }

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
 */
struct Grid : Leaf
{

    int counter;
    glv::Point3*    vertices;
    glv::Color*     colors;

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
            glv::Color c = (counter % 4 == 0 ) ? glv::Color( .82 ) : glv::Color( 112.f/255.f, 138.f/255.f, 144.f/255.f ) ; 

            colors[counter] = c; 
            vertices[counter++]( i, lower, 0);
            colors[counter] = c; 
            vertices[counter++]( i, upper, 0);
        }

        for ( int i=lower; i<upper; i+=spacing )
        {
            glv::Color c = (counter % 4 == 0 ) ? glv::Color( .82 ) : glv::Color( 112.f/255.f, 138.f/255.f, 144.f/255.f ) ; 
            
            colors[counter] = c; 
            vertices[counter++]( lower, i , 0);
            colors[counter] = c; 
            vertices[counter++]( upper, i , 0);
        }

    }

    ~Grid()
    {
        delete [] vertices;
    }


    void onDraw3D(glv::GLV& g)
    { 
        glv::draw::lineWidth( .01 );
        glv::draw::paint( glv::draw::Lines, vertices, colors, counter );
    }
};

/*
 *Histogram Renderer
 */
struct HistogramRenderer : Leaf
{
    L3::histogram<double>* hist;

    void operator()( L3::histogram<double>* HIST )
    {
        hist = HIST;
    }

    void onDraw3D(glv::GLV& g)
    {
        glv::Point3 quad_vertices[4];
        glv::Color quad_colors[4];

        float delta = hist->delta;

        for( unsigned int i=0; i < hist->num_bins; i++ )
        {
            for( unsigned int j=0; j < hist->num_bins; j++ )
            {
                unsigned int val = hist->bin( i, j );
          
                glv::Color c = glv::Color( val/10.0 );

                std::pair<float,float> coords = hist->coords( i, j);

                quad_colors[0] = c;
                quad_vertices[0]( coords.first-delta/2.5, coords.second-delta/2.5, val );
                quad_colors[1] = c;
                quad_vertices[1]( coords.first-delta/2.5, coords.second+delta/2.5, val );
                quad_colors[2] = c;
                quad_vertices[2]( coords.first+delta/2.5, coords.second+delta/2.5, val );
                quad_colors[3] = c;
                quad_vertices[3]( coords.first+delta/2.5, coords.second-delta/2.5, val );
        
                glv::draw::paint( glv::draw::TriangleFan, quad_vertices, quad_colors, 4 );
            
            }
        }
    }
};

/*
 *CoordinateSystem : XYZ, RGB
 */
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
        delete [] colors;
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

        // Test to see if the model matrix is consistent
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
