#ifndef L3_COMPONENTS_H
#define L3_COMPONENTS_H

#include <iostream>

#include <GLV/glv.h>

#include "L3.h"


namespace L3
{

namespace Visualisers
{

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
    double current_time; 
    unsigned int sf;
    std::list<Leaf*> components; 

    void move( double x, double y, double z )
    {
        _x+=x;
        _y+=y;
        _z+=z;
    }

    void rotate( double r, double p, double q )
    {
        _r+=r;
        _p+=p;
        _q+=q;
    }
    double _x, _y, _z;
    double _r, _p, _q;

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

    virtual void onDraw2D( glv::GLV& g)
    {
    }

    Composite& operator<<( Leaf& leaf )
    {
        components.push_back( &leaf );
        return *this;
    }
    
};

struct Grid : Leaf
{

    Grid()
    {
        vertices = new glv::Point3[ 100*4 ];

        counter = 0;
        // Add horizontal
        for ( int i=0; i<100; i++ )
        {
            vertices[counter++]( i, 0 , 0);
            vertices[counter++]( i, 100 , 0);
        }

        for ( int j=0; j<100; j++ )
        {
            vertices[counter++]( 0, j , 0);
            vertices[counter++]( 100, j , 0);
        }

    }

    int counter;

    ~Grid()
    {
        delete [] vertices;
    }
        
    glv::Point3* vertices;

    void onDraw3D(glv::GLV& g)
    { 
        glv::draw::push();
        glv::draw::identity(); 
        //glv::draw::paint( glv::draw::Points, vertices, counter );
       
        glv::Point3 verts[2];
        verts[0]( 0, 0, 0 );
        verts[1]( 100, 100, 0 );
        glv::draw::paint( glv::draw::Lines, verts, 2 );

        glv::draw::pop();
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
