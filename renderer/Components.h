#ifndef L3_COMPONENTS_H
#define L3_COMPONENTS_H

#include <iostream>

#include <GLV/glv.h>

#include "Controller.h"
#include "L3.h"

#include <boost/shared_array.hpp>

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
    Composite( const glv::Rect rect = glv::Rect(100,100) ) : glv::View3D( rect ),
        current_time(0.0), 
        sf(1), 
        controller(NULL)
    {
        // Extend far clip
        far( 1000 );
        
        // Fill the view
        stretch(1,1).anchor(0,0); 

        // Appropriate view-point
        position.z = -500; 
    }
    
    double          current_time; 
    control_t       position ;
    unsigned int    sf;
    L3::Visualisers::Controller* controller;
 
    clock_t                 current, previous;
    std::list<Leaf*>        components; 

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

};

/*
 *  CoordinateSystem 
 */
struct CoordinateSystem
{
    explicit CoordinateSystem( boost::shared_ptr< L3::SE3 > pose  );

    boost::shared_ptr< L3::SE3 >        pose;
    boost::shared_array< glv::Color >   colors;
    boost::shared_array< glv::Point3 >  vertices;

    void onDraw3D(glv::GLV& g);
   
};

template <typename T> 
struct TextRenderer : glv::View
{

    explicit TextRenderer( T& v = 0 ) : t(v)
    {
    }

    T& t;
    
    void onDraw(glv::GLV& g)
    {
        glv::draw::color(1);
        glv::draw::lineWidth(2);
        
        std::stringstream ss;
        ss.precision( 15 );
        
        ss << t;
        glv::draw::text( ss.str().c_str() );
    }

};



/*
 *  Point cloud renderer
 */
struct PointCloudRenderer : Leaf
{
    PointCloudRenderer( L3::PointCloud<double>* cloud );

    L3::PointCloud<double>*                     cloud;
    boost::shared_ptr<L3::PointCloud<double> >  plot_cloud;
    
    boost::shared_array< glv::Color >   colors;
    boost::shared_array< glv::Point3 >  vertices;

    void onDraw3D( glv::GLV& g );
    
};

/*
 *Point cloud bounds renderer
 */
struct PointCloudBoundsRenderer : Leaf
{
    L3::PointCloud<double>*  cloud;

    PointCloudBoundsRenderer( L3::PointCloud<double>* point_cloud );
    
    void onDraw3D(glv::GLV& g);

};



/*
 *  Grid
 */
struct Grid : Leaf
{
    Grid();

    int counter;
    boost::shared_array< glv::Point3 >  vertices;
    boost::shared_array< glv::Color >   colors;

    void onDraw3D(glv::GLV& g);
    
};

/*
 *  Histograms
 */
struct HistogramRenderer 
{
    HistogramRenderer( boost::shared_ptr<L3::Histogram<double> > histogram ) : hist(histogram)
    {
        
    }

    boost::shared_ptr<L3::Histogram<double> > hist;
};

/*
 *  Histogram :: Bounds Renderer
 */
struct HistogramBoundsRenderer : HistogramRenderer, Leaf
{
    HistogramBoundsRenderer( boost::shared_ptr<L3::Histogram<double> > histogram ) : HistogramRenderer(histogram)
    {
    }
    
    void onDraw3D(glv::GLV& g);
};

/*
 *  Histogram :: Vertex Renderer
 */
struct HistogramVertexRenderer : HistogramRenderer, Leaf
{
    HistogramVertexRenderer( boost::shared_ptr<L3::Histogram<double> > histogram ) : HistogramRenderer(histogram)
    {
    }
    
    void onDraw3D(glv::GLV& g);
};

struct HistogramPixelRenderer : glv::Plot, HistogramRenderer, Poco::Runnable
{
	HistogramPixelRenderer(const glv::Rect& r, boost::shared_ptr<L3::Histogram<double> > histogram  )
        : glv::Plot(r), HistogramRenderer(histogram), running(true)
    {
        // Assign density plot
        this->add(*new glv::PlotDensity( glv::Color(1)) );
   
        thread.start( *this );
   
        t.restart();
    }

    ~HistogramPixelRenderer()
    {
        running = false;
   
        if (thread.isRunning())
            thread.join();
    }

    boost::timer    t;
    Poco::Thread    thread;
    bool            running;

    void run();
};

/*
 *  Pose prediction 
 */

struct PoseEstimatesRenderer : Leaf
{

    PoseEstimatesRenderer( boost::shared_ptr<L3::Estimator::PoseEstimates> estimates )  : pose_estimates(estimates)
    {
    }
    
    boost::shared_ptr<L3::Estimator::PoseEstimates> pose_estimates;

    void onDraw3D(glv::GLV& g);
    
};


}
}

#endif

