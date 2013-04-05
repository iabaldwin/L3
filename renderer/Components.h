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

struct Updateable
{

    virtual void update()
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
    Composite() : glv::View3D( glv::Rect(100,100) ),
        current_time(0.0), 
        sf(1), 
        controller(NULL)
    {
        // Extend far clip
        far( 1000 );
        
        // Fill the view
        stretch(1,1); 

        // Appropriate view-point
        position.z = -500; 
    }
    
    double          current_time; 
    control_t       position ;
    unsigned int    sf;
    L3::Visualisers::Controller* controller;
 
    clock_t                 current, previous;
    std::list<Leaf*>        components; 
    std::list<Updateable*>  updateables; 

    void apply( control_t t )
    {
        position += t;
    }

    void addController( L3::Visualisers::Controller* c )
    {
        controller = c; 
    }

    bool onEvent( glv::Event::t type, glv::GLV& g )
    {
        if (controller)
            apply( controller->onEvent( type, g ) );
  
        return true;
    }
        
    virtual void onDraw3D(glv::GLV& g);

    Composite& operator<<( Leaf& leaf )
    {
        components.push_back( &leaf );
        return *this;
    }
  
    Composite& operator<<( Updateable& leaf )
    {
        updateables.push_back( &leaf );
        return *this;
    }
  


};

/*
 *Element runner
 */
struct Runner : Leaf, L3::TemporalRunner
{
    void onDraw3D(glv::GLV& g)
    {
        //this->update( time );
        L3::TemporalRunner::update( time );
    }

};


/*
 *  Useful components
 *
 *  1.  Grid
 */
struct Grid : Leaf
{

    int counter;
    glv::Point3*    vertices;
    glv::Color*     colors;

    Grid();
    ~Grid();

    void onDraw3D(glv::GLV& g);
    
};

/*
 *  Histograms
 */
struct HistogramRenderer 
{
    L3::Histogram<double>* hist;

    void operator()( L3::Histogram<double>* hist )
    {
        this->hist = hist;
    }

};

std::ostream& operator<<( std::ostream& o, const std::pair<float, float>& input );

/*
 *  Histogram :: Bounds Renderer
 */
struct HistogramBoundsRenderer : HistogramRenderer, Leaf
{
    void onDraw3D(glv::GLV& g);
};

/*
 *  Histogram :: Vertex Renderer
 */
struct HistogramVertexRenderer : HistogramRenderer, Leaf
{
    void onDraw3D(glv::GLV& g);
};

struct HistogramPixelRenderer : glv::Plot, HistogramRenderer, Updateable
{
	HistogramPixelRenderer(const glv::Rect& r )
        : glv::Plot(r)
    {
        // Assign density plot
        this->add(*new glv::PlotDensity( glv::Color(1)) );
    }
   
    void update();
};

/*
 *  CoordinateSystem 
 */
struct CoordinateSystem
{
    L3::SE3 _pose;
    glv::Color* colors;
    glv::Point3* vertices;
 
    CoordinateSystem() ;
    CoordinateSystem( const L3::SE3& pose );

    ~CoordinateSystem();

    void _init();
    
    void onDraw3D(glv::GLV& g);
   
};

struct PointCloudBoundsRenderer : Leaf
{
    L3::PointCloud<double>*  cloud;

    PointCloudBoundsRenderer( L3::PointCloud<double>* point_cloud );
    
    void onDraw3D(glv::GLV& g);

};


}
}

#endif

