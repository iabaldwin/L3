#ifndef L3_COMPONENTS_H
#define L3_COMPONENTS_H

#include <iostream>

#include <GLV/glv.h>

#include "L3.h"
#include "Controller.h"

#include "RenderingUtils.h"

#include <boost/shared_array.hpp>

namespace L3
{
namespace Visualisers
{

struct Updateable
{
    virtual void update() = 0;
};

struct Updater : glv::Plot
{
    std::list < Updateable* > updateables;
    
    void onDraw(glv::GLV& g)
    {
        for( std::list< Updateable* >::iterator it = updateables.begin(); it != updateables.end(); it++ )
            (*it)->update();
    }

    Updater& operator<<( Updateable* updateable )
    {
        updateables.push_front( updateable );
    }
};



/*
 *  Single entity for 3D rendering
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

    virtual void onDraw3D( glv::GLV& g ) = 0;

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
    Composite( const glv::Rect rect = glv::Rect(100,100) ) 
        : glv::View3D( rect ), controller(NULL)
    {
        // Extend far clip
        far( 1000 );

        // Appropriate view-point
        position.translateZ( -250 );
    }

    control_t                       position;
    std::list<Leaf*>                components; 
    L3::Visualisers::Controller*    controller;

    Composite& addController( L3::Visualisers::Controller* c )
    {
        controller = c; 
        return *this;
    }

    bool onEvent( glv::Event::t type, glv::GLV& g )
    {
        if (controller)
            controller->onEvent( type, g );

        return true;
    }

    void onDraw3D( glv::GLV& g );
    
    Composite& operator<<( Leaf& leaf )
    {
        components.push_back( &leaf );
        return *this;
    }

};

/*
 *  Coordinate System 
 */
struct CoordinateSystem
{
    explicit CoordinateSystem( L3::SE3& pose , float scale=10.0f );

    L3::SE3&                            pose;
    boost::shared_array< glv::Color >   colors;
    boost::shared_array< glv::Point3 >  vertices;

    float scale;
    
    void onDraw3D(glv::GLV& g);
   
};

template <typename T> 
struct TextRenderer : glv::View
{
    explicit TextRenderer( T& v = 0 ) : t(v), glv::View( glv::Rect(150,25 ) )
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
 *  Single referenced pose renderer
 */
struct PoseRenderer : Leaf
{

    PoseRenderer( L3::SE3& pose ) : pose(pose)
    {
    }

    L3::SE3& pose;

    void onDraw3D( glv::GLV& g )
    {
        CoordinateSystem( pose ).onDraw3D( g ); 
    }

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

/*
 *  Point cloud renderer core
 */
struct PointCloudRenderer 
{
    PointCloudRenderer( boost::shared_ptr< L3::PointCloud<double> > cloud, glv::Color c = glv::Color(.5)  );

    glv::Color color;

    boost::shared_array< glv::Color >   colors;
    boost::shared_array< glv::Point3 >  vertices;
   
    boost::shared_ptr< L3::PointCloud<double> > cloud;
    boost::shared_ptr< L3::PointCloud<double> > point_cloud;
    boost::shared_ptr< L3::PointCloud<double> > plot_cloud;

    void onDraw3D( glv::GLV& g );
    
};

/*
 *  Point cloud renderer
 */
struct PointCloudRendererLeaf : PointCloudRenderer, Leaf
{
    PointCloudRendererLeaf( boost::shared_ptr< L3::PointCloud<double> > cloud ) : PointCloudRenderer(cloud)
    {
    }

    void onDraw3D( glv::GLV& g );
};

/*
 *  Point cloud renderer, view
 */
struct PointCloudRendererView: PointCloudRenderer, glv::View3D, Updateable
{
    PointCloudRendererView( const glv::Rect& r, boost::shared_ptr< L3::PointCloud<double> > cloud, L3::SE3* estimate ) 
        : PointCloudRenderer(cloud),
            glv::View3D(r),
            current_estimate(estimate)

    {
    }

       
    L3::SE3* current_estimate;

    void update();

    void onDraw3D( glv::GLV& g );
   
};

/*
 *  Multi-point cloud renderer
 */
struct CompositeCloudRendererLeaf : Leaf
{
    ColorCycler cycler;

    std::list< PointCloudRendererLeaf* > renderers;

    CompositeCloudRendererLeaf& operator<<( PointCloudRendererLeaf* renderer )
    {
        renderer->color = cycler();

        renderers.push_front( renderer );

        return *this;
    }

    virtual void onDraw3D( glv::GLV& g )
    {
        for( std::list< PointCloudRendererLeaf* >::iterator it = renderers.begin();
                it != renderers.end();
                it++ )
            (*it)->onDraw3D( g );

    }
};

/*
 *Point cloud bounds renderer
 */
struct PointCloudBoundsRenderer : Leaf
{
    boost::shared_ptr< L3::PointCloud<double> > cloud;

    PointCloudBoundsRenderer( boost::shared_ptr< L3::PointCloud<double> > point_cloud ) : cloud(point_cloud)
    {
    }
    
    void onDraw3D(glv::GLV& g);

};

/*
 *  Grid
 */
struct Grid : Leaf
{
    Grid( float lower=-500, float upper=500, float spacing=50);

    int counter;
    boost::shared_array< glv::Point3 >  vertices;
    boost::shared_array< glv::Color >   colors;

    float lower, upper, spacing;

    void onDraw3D(glv::GLV& g);
    
};

/*
 *  DefaultAxes
 */
struct DefaultAxes : Leaf
{
    DefaultAxes();

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
    HistogramBoundsRenderer( boost::shared_ptr<L3::Histogram<double> > histogram ) : HistogramRenderer(histogram) , depth(-5.0)
    {
    }
  
    float depth;

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

/*
 *  Histogram :: Density renderer
 */
struct HistogramDensityRenderer : glv::Plot, HistogramRenderer, Updateable
{
	HistogramDensityRenderer(const glv::Rect& r, boost::shared_ptr<L3::Histogram<double> > histogram )
        : glv::Plot(r), 
        HistogramRenderer(histogram)
    {
        // Assign density plot
        this->add(*new glv::PlotDensity( glv::Color(1)) );
    }

    void update();
};

/*
 *  Histogram :: Voxel Renderer
 */
struct HistogramVoxelRenderer : HistogramRenderer, Updateable
{
	HistogramVoxelRenderer(boost::shared_ptr<L3::Histogram<double> > histogram  )
        : HistogramRenderer(histogram)
    {
        plot_histogram.reset( new L3::Histogram<double>() );
    }

    boost::shared_ptr< L3::Histogram<double> > plot_histogram;

    void onDraw3D( glv::GLV& g );

    virtual void update(){};
};


struct HistogramVoxelRendererView : HistogramVoxelRenderer, glv::View3D
{
	HistogramVoxelRendererView( const glv::Rect& r, boost::shared_ptr<L3::Histogram<double> > histogram  )
        : HistogramVoxelRenderer(histogram), glv::View3D(r)
    {
    }
    
    void onDraw3D( glv::GLV& g )
    {
        glv::draw::translateZ( -50 );
        
        L3::ReadLock lock( hist->mutex );
        L3::clone( hist.get(), plot_histogram.get() );
     
        std::pair<float, float> lower_left = hist->coords(0,0);
        std::pair<float, float> upper_right = hist->coords( hist->x_bins, hist->y_bins );

        float x_delta = (upper_right.first +lower_left.first)/2.0;
        float y_delta = (upper_right.second +lower_left.second)/2.0;

        glv::draw::translate( -1*x_delta, -1*y_delta, 0.0  );

        HistogramVoxelRenderer::onDraw3D(g);    
   
    }

};


struct HistogramVoxelRendererLeaf : HistogramVoxelRenderer, Leaf
{
	HistogramVoxelRendererLeaf(boost::shared_ptr<L3::Histogram<double> > histogram  )
        : HistogramVoxelRenderer(histogram)
    {
    }
    
    void onDraw3D( glv::GLV& g )
    {
        L3::ReadLock lock( hist->mutex );
        L3::clone( hist.get(), plot_histogram.get() );
        lock.unlock();

        HistogramVoxelRenderer::onDraw3D(g);    
    }

};

/*
 *  Single pose orientation renderer
 */
struct DedicatedPoseRenderer : glv::View3D, Updateable
{

    DedicatedPoseRenderer( L3::PoseProvider* provider, const glv::Rect rect = glv::Rect(50,50) ) 
        : glv::View3D( rect ),
            provider(provider)
    {
        pose.reset( new L3::SE3( L3::SE3::ZERO() ) );
    }

    L3::PoseProvider* provider;

    boost::shared_ptr<L3::SE3> pose;
    
    L3::Visualisers::DefaultAxes axes;

    void update();

    void onDraw3D( glv::GLV& g );
};

/*
 *  Raw scan renderer
 */
struct ScanRenderer2D : glv::View3D, Updateable
{

    ScanRenderer2D( L3::ConstantTimeIterator< L3::LMS151 >* windower, const glv::Rect& r, glv::Color color ) 
        : glv::View3D( r ),
            color(color),
            windower(windower)
    {
        scan.reset( new L3::LMS151() );
    }

    virtual ~ScanRenderer2D()
    {
    }

    std::deque< std::pair< double, boost::shared_ptr<L3::LMS151> > > window;
    
    L3::ConstantTimeIterator< L3::LMS151 >* windower;

    boost::shared_ptr<L3::LMS151> scan;
  
    glv::Color color;

    float rotate_z;

    void onDraw3D( glv::GLV& g );

        

    void update();
};


struct HorizontalScanRenderer2D : ScanRenderer2D
{

    HorizontalScanRenderer2D( L3::ConstantTimeIterator< L3::LMS151 >* windower, const glv::Rect& rect )
        : ScanRenderer2D( windower, rect, glv::Color(1,0,0) )
    {
        rotate_z = 0;
    }
};

struct VerticalScanRenderer2D : ScanRenderer2D
{

    VerticalScanRenderer2D( L3::ConstantTimeIterator< L3::LMS151 >* windower, const glv::Rect& rect )
        : ScanRenderer2D( windower, rect, glv::Color(0,0,1) )
    {
        rotate_z = 180;
    }
};

/*
 *  Costs rendering structure
 */
struct CostRenderer 
{

    CostRenderer( L3::Estimator::PoseEstimates& estimates ) 
        : estimates(estimates)
    {
    }

    L3::Estimator::PoseEstimates& estimates;

    void onDraw3D( glv::GLV& g );

};

struct CostRendererLeaf : CostRenderer, Leaf
{
    CostRendererLeaf( L3::Estimator::PoseEstimates& estimates ) 
        : CostRenderer(estimates)    
    {

    }

    void onDraw3D( glv::GLV& g )
    {
        CostRenderer::onDraw3D( g );     
    }

};

struct CostRendererView : CostRenderer, glv::View3D
{
    CostRendererView( L3::Estimator::PoseEstimates& estimates, const glv::Rect rect = glv::Rect(50,50) ) 
        : CostRenderer(estimates), 
            glv::View3D( rect )
    {

    }


    void onDraw3D( glv::GLV& g );
    
};





}   // Visualisers
}   // L3

#endif

