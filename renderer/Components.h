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
        
    virtual void onDraw3D(glv::GLV& g)
    {
        glv::draw::translate( position.x, position.y, position.z );
        glv::draw::rotate( position.r, position.p, position.q );

        // 1. Compute time since last update
        current = clock();
        double elapsed = double(current - previous)/CLOCKS_PER_SEC;

        // Takes care of initial inf.
        elapsed  = (elapsed > 1.0) ? 1.0 : elapsed;

        std::list<Updateable*>::iterator updateable_iterator = updateables.begin();
        while( updateable_iterator != updateables.end() )
        {
            (*updateable_iterator)->update();
            updateable_iterator++;
        }

        // 2. Increment the *time* by this value 
        current_time += (elapsed *= sf);

        std::list<Leaf*>::iterator leaf_iterator = components.begin();
        while( leaf_iterator != components.end() )
        {
            (*leaf_iterator)->time = this->current_time;
            (*leaf_iterator)->onDraw3D( g );
            leaf_iterator++;
        }
 
        previous = current;
    }

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

struct HistogramBoundsRenderer : HistogramRenderer, Leaf
{

    void onDraw3D(glv::GLV& g)
    {
        glv::Point3 bound_vertices[4];
        glv::Color  bound_colors[4];

        std::pair<float, float> lower_left = hist->coords(0,0);
        std::cout << "LL " << lower_left << std::endl;

        std::pair<float, float> upper_left = hist->coords( hist->y_bins-1,0);
        std::cout << "UL " << upper_left << std::endl;
        
        std::pair<float, float> upper_right = hist->coords( hist->y_bins-1, hist->x_bins-1 );
        std::cout << "UR " << upper_right << std::endl;

        std::pair<float, float> lower_right = hist->coords( 0,hist->x_bins-1 );
        std::cout << "LR " << lower_right << std::endl;

        bound_vertices[0]( lower_left.first, lower_left.second, 0.0 );
        bound_vertices[1]( upper_left.first, upper_left.second, 0.0 );
        bound_vertices[2]( upper_right.first, upper_right.second, 0.0 );
        bound_vertices[3]( lower_right.first, lower_right.second, 0.0 );

        glv::draw::paint( glv::draw::LineLoop, bound_vertices, bound_colors, 4 );
        std::cout << "---------------" << std::endl;
    
    }


};

struct HistogramVertexRenderer : HistogramRenderer, Leaf
{
    void onDraw3D(glv::GLV& g)
    {
        glv::Point3 quad_vertices[4];
        glv::Color quad_colors[4];

        float x_delta = hist->x_delta;
        float y_delta = hist->y_delta;

        for( unsigned int i=0; i < hist->y_bins; i++ )
        {
            for( unsigned int j=0; j < hist->x_bins; j++ )
            {
                unsigned int val = hist->bin( i, j );
          
                glv::Color c = glv::Color( val/10.0 );

                std::pair<float,float> coords = hist->coords( i, j);

                quad_colors[0] = c;
                quad_vertices[0]( coords.first-x_delta/2.5, coords.second-y_delta/2.5, val );
                quad_colors[1] = c;
                quad_vertices[1]( coords.first-x_delta/2.5, coords.second+y_delta/2.5, val );
                quad_colors[2] = c;
                quad_vertices[2]( coords.first+x_delta/2.5, coords.second+y_delta/2.5, val );
                quad_colors[3] = c;
                quad_vertices[3]( coords.first+x_delta/2.5, coords.second-y_delta/2.5, val );
        
                glv::draw::paint( glv::draw::TriangleFan, quad_vertices, quad_colors, 4 );
            
            }
        }
    }
};

struct HistogramPixelRenderer : glv::Plot, HistogramRenderer, Updateable
{
	HistogramPixelRenderer(const glv::Rect& r )
        : glv::Plot(r)
    {
        // Assign density plot
        this->add(*new glv::PlotDensity( glv::Color(1)) );
    }
   
    void update()
    {
        data().resize( glv::Data::FLOAT, 1, hist->x_bins, hist->y_bins );

        for( unsigned int i=0; i< hist->x_bins; i++ )
        {
            for( unsigned int j=0; j< hist->y_bins; j++ )
            {
                data().assign( hist->bin(i,j)/10.0 , 0, i, j );
            }
        }
   
    }
};

/*
 *CoordinateSystem : XYZ, RGB
 */
struct CoordinateSystem
{
    CoordinateSystem()  : _pose( L3::SE3::ZERO() ), vertices(NULL), colors(NULL)
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
        if ( vertices )
            delete [] vertices;
        if ( colors )
            delete [] colors;
    }

    void _init()
    {
        vertices = new glv::Point3[6];
       
        float scale =10.0f;

        vertices[0]( 0, 0, 0 ); 
        vertices[1]( scale, 0, 0 ); 
        vertices[2]( 0, 0, 0 ); 
        vertices[3]( 0, scale, 0 ); 
        vertices[4]( 0, 0, 0 ); 
        vertices[5]( 0, 0, scale ); 
   
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
        glv::draw::rotate( L3::Utils::Math::radiansToDegrees(_pose.r), 
                            L3::Utils::Math::radiansToDegrees(_pose.p),
                            L3::Utils::Math::radiansToDegrees(_pose.q));
        
        glv::draw::paint( glv::draw::LineStrip, vertices, colors, 6 );
        
        glv::draw::pop();
    }

    glv::Color* colors;
    glv::Point3* vertices;

};


}
}

#endif
