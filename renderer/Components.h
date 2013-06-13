#ifndef L3_COMPONENTS_H
#define L3_COMPONENTS_H

#include <iostream>
#include <GLV/glv.h>

#include <boost/ref.hpp>
#include <boost/any.hpp>
#include <boost/shared_array.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/filesystem.hpp>
#include <boost/shared_array.hpp>
#include <boost/make_shared.hpp>
#include <boost/date_time/local_time/local_time.hpp>

#include "L3.h"
#include "RenderingUtils.h"
#include "ViewController.h"

/*
 *Octree specific
 */
#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_iterator.h>

namespace L3
{
namespace Visualisers
{
    /*
     *  Misc
     */
    void transformCameraToPose( L3::SE3& pose );

    struct Cube
    {
        Cube( float x_lower, float y_lower, float z_lower, 
                float x_upper, float y_upper, float z_upper,
                float opacity ) 
            : x_lower(x_lower), y_lower( y_lower ), z_lower( z_lower ),
            x_upper(x_upper), y_upper( y_upper ), z_upper( z_upper ),
            opacity( opacity )
        {
        }
        float opacity;
        float x_lower, y_lower, z_lower;
        float x_upper, y_upper, z_upper;
    };

    void renderCube( Cube* cube );


    struct VisualUpdater : glv::View, L3::Updater
    {
        void onDraw(glv::GLV& g)
        {
            Updater::update();
        }

    };

    struct SpatialUpdater : glv::View
    {
        SpatialUpdater( boost::shared_ptr< L3::PoseProvider > provider ) : provider(provider)
        {

        }

        boost::weak_ptr< L3::PoseProvider > provider;

        std::list < SpatialObserver* > observers;

        void onDraw(glv::GLV& g)
        {
            boost::shared_ptr< L3::PoseProvider > provider_ptr = provider.lock();

            if( !provider_ptr )
                return;

            L3::SE3 pose = provider_ptr->operator()();

            double x = pose.X();
            double y = pose.Y();
            for( std::list< SpatialObserver* >::iterator it = observers.begin(); it != observers.end(); it++ )
                (*it)->update( x, y );
        }

        SpatialUpdater& operator<<( SpatialObserver* observer )
        {
            observers.push_front( observer );
            return *this;
        }
    };

    struct Controllable
    {
        Controllable() : control_x(0.0), control_y(0.0), control_z(0.0)
        {
        }

        float control_x,control_y,control_z;
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
     *  Leaf component for 3D rendering
     */
    struct Leaf 
    {
        Leaf() 
            : draw_bounds(false),
            visible(true)
        {
            bound_vertices = boost::make_shared< glv::Point3[] >( 24 ); 
            bound_colors   = boost::make_shared< glv::Color[] >( 24 ); 
        }

        struct bounds {
            float x,y,z;
        };

        bool draw_bounds, visible;

        bounds lower,upper;

        boost::shared_ptr< glv::Point3[] >  bound_vertices;
        boost::shared_ptr< glv::Color[] >   bound_colors;

        virtual void onDraw3D( glv::GLV& g ) = 0;

        void drawBounds();

    };


    /*
     *  Text
     */
    template <typename T> 
        struct TextRenderer : glv::TextView
    {
        explicit TextRenderer() : glv::TextView( glv::Rect(150,25 ) )
        {
            this->disable( glv::DrawBorder );
        }

        boost::shared_ptr< variable_lock<T> > lock;

        void setVariable( T& t )
        {
            lock = boost::make_shared< variable_lock<T> >( boost::ref(t) ); 
        }

        void onDraw(glv::GLV& g)
        {
            std::stringstream ss;
            ss.precision( 15 );

            if( lock )
                ss << lock->t;

            mText = ss.str();

            glv::TextView::onDraw(g);

        }

    };


    struct Text3D : Leaf
    {
        Text3D();

        std::string text;
        int font_descriptor;
        float xmin,ymin,xmax,ymax;

        float scale;

        void setText( std::string text );
        void onDraw3D( glv::GLV& g );
    };

    struct label_tag
    {
        float x,y;
        std::string text;
    } ;

    struct LeafLabel : Text3D
    {

        LeafLabel( label_tag* tag ) : tag(tag)
        {
        }

        void onDraw3D( glv::GLV& g );

        label_tag* tag;
    };


    struct LabelledLeaf : Leaf
    {
        LabelledLeaf()
        {
            tag.x = 0.0;
            tag.y = 0.0;

            label = boost::make_shared< L3::Visualisers::LeafLabel >( &this->tag );
        }

        label_tag tag;

        boost::shared_ptr< L3::Visualisers::LeafLabel > label;

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

            this->enable( glv::Property::AlwaysBubble );
        }

        int viewport[4];
        double model[16];
        double projection[16];

        control_t                       position;
        std::list<Leaf*>                components; 
        L3::Visualisers::Controller*    controller;

        virtual void onDraw3D( glv::GLV& g );

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
        explicit CoordinateSystem( L3::SE3& pose , float scale=10.0f, float alpha=1.0f );

        L3::SE3&                            pose;
        boost::shared_ptr< glv::Color[] >   colors;
        boost::shared_ptr< glv::Point3[] >  vertices;

        float scale, alpha;

        void onDraw3D(glv::GLV& g);

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

        virtual void onDraw3D( glv::GLV& g )
        {
            CoordinateSystem( pose, 1.0 ).onDraw3D( g ); 
        }

    };

    struct AnimatedPoseRenderer : PoseRenderer
    {

        AnimatedPoseRenderer( L3::SE3& pose ) 
            : PoseRenderer( pose ), range(1.0)
        {
        }

        float range;

        void onDraw3D( glv::GLV& g );
    };

    struct PoseSequenceRenderer : Leaf
    { 

        PoseSequenceRenderer( boost::shared_ptr< std::vector< std::pair< double, boost::shared_ptr< L3::SE3 > > > > pose_sequence ) : pose_sequence(pose_sequence)
        {
            highlighted_position = 0;
            skip = 100; 
        }

        int highlighted_position, skip ;

        boost::shared_ptr< std::vector< std::pair< double, boost::shared_ptr< L3::SE3 > > > > pose_sequence;

        void onDraw3D(glv::GLV& g);
    };

    /*
     *  Pose prediction 
     */

    struct PoseEstimatesRenderer : Leaf
    {

        PoseEstimatesRenderer( boost::shared_ptr<L3::Estimator::PoseEstimates> estimates )  : estimates(estimates)
        {
        }

        boost::weak_ptr<L3::Estimator::PoseEstimates> estimates;

        void onDraw3D(glv::GLV& g);

    };

    /*
     *Point cloud bounds renderer
     */
    struct PointCloudBoundsRenderer : Leaf
    {
        PointCloudBoundsRenderer( boost::shared_ptr< L3::PointCloud<double> > point_cloud ) : cloud(point_cloud)
        {
        }

        boost::weak_ptr< L3::PointCloud<double> > cloud;

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

        boost::weak_ptr< L3::PointCloud<double> > cloud;
        boost::shared_ptr< L3::PointCloud<double> > plot_cloud;

        void onDraw3D( glv::GLV& g );

    };

    /*
     *  Point cloud renderer
     */
    struct PointCloudRendererLeaf : PointCloudRenderer, Leaf
    {
        PointCloudRendererLeaf( boost::shared_ptr< L3::PointCloud<double> > cloud, boost::shared_ptr< L3::PoseProvider > pose_provider = boost::shared_ptr<L3::PoseProvider>() ) 
            : PointCloudRenderer(cloud), provider(pose_provider)
        {
        }

        boost::weak_ptr< L3::PoseProvider > provider;

        void onDraw3D( glv::GLV& g );
    };

    /*
     *  Point cloud renderer, view
     */
    struct PointCloudRendererView: PointCloudRenderer, glv::View3D, Updateable
    {
        PointCloudRendererView( const glv::Rect& r, boost::shared_ptr< L3::PointCloud<double> > cloud, boost::shared_ptr< L3::SE3 > estimate ) 
            : PointCloudRenderer(cloud),
            glv::View3D(r),
            current_estimate(estimate)

        {
            bounds_renderer = boost::make_shared< PointCloudBoundsRenderer >( cloud );

            label.setValue( "Run-time swathe" );
            label.pos( glv::Place::BL, 0, 0 ).anchor( glv::Place::BL ); 
            (*this) << label;
        }

        boost::shared_ptr< L3::Visualisers::PointCloudBoundsRenderer > bounds_renderer;

        boost::weak_ptr< L3::SE3 > current_estimate;

        std::pair<double,double> centroid;

        glv::Label label;

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
     *  Grid
     */
    struct Grid : Leaf
    {
        Grid( float lower=-500, float upper=500, float spacing=50);

        int counter;
        boost::shared_ptr< glv::Point3[] > vertices;
        boost::shared_ptr< glv::Color[] >  colors;

        float lower, upper, spacing;

        void onDraw3D(glv::GLV& g);

    };

    /*
     *  Spatial grid
     */
    struct DynamicGrid : Grid, SpatialObserver
    {

        DynamicGrid( float lower=-500, float upper=500, float spacing=50)
            : Grid( lower, upper, spacing ),
            current_x(0.0),
            current_y(0.0)
        {

        }

        double current_x;
        double current_y;

        bool update( double x, double y )
        {
            current_x = x;
            current_y = y;

            return false;
        }

        void onDraw3D(glv::GLV& g);

    };

    /*
     *  DefaultAxes
     */
    struct DefaultAxes : Leaf
    {
        DefaultAxes();

        int counter;
        boost::shared_ptr< glv::Point3[] >  vertices;
        boost::shared_ptr< glv::Color[] >   colors;

        void onDraw3D(glv::GLV& g);

    };


    /*
     *  Single pose orientation renderer
     */
    struct DedicatedPoseRenderer : glv::View3D, Updateable
    {
        DedicatedPoseRenderer( boost::shared_ptr< L3::PoseProvider > provider, const glv::Rect rect = glv::Rect(50,50), const std::string& text="" ) 
            : glv::View3D( rect ),
            provider(provider)
        {
            pose = boost::make_shared< L3::SE3 >();

            label.setValue( text );
            label.pos( glv::Place::BL, 0, 0 ).anchor( glv::Place::BL ); 
            *this << label;
        }

        boost::weak_ptr< L3::PoseProvider > provider;

        boost::shared_ptr<L3::SE3> pose;

        L3::Visualisers::DefaultAxes axes;

        glv::Label label;

        void update();

        void onDraw3D( glv::GLV& g );
    };

    /*
     *  Raw scan renderer
     */
    struct ScanRenderer2D : Updateable
    {

        ScanRenderer2D( boost::shared_ptr< L3::ConstantTimeIterator< L3::LMS151 > > windower, glv::Color color, float range_threshold=2.0  ) 
            : windower(windower),
            color(color),
            range_threshold(range_threshold)
        {
            scan = boost::make_shared< LMS151 >();
        }

        virtual ~ScanRenderer2D()
        {
        }

        glv::Label      label;
        glv::Color      color;
        float           rotate_z, range_threshold;
        boost::shared_ptr<L3::LMS151>           scan;

        boost::weak_ptr< L3::ConstantTimeIterator< L3::LMS151 > > windower;

        std::deque< std::pair< double, boost::shared_ptr<L3::LMS151> > > window;

        void onDraw3D( glv::GLV& g );

        void update();
    };

    struct HorizontalScanRenderer2DView : glv::View3D, ScanRenderer2D
    {

        HorizontalScanRenderer2DView( boost::shared_ptr< L3::ConstantTimeIterator< L3::LMS151 > > windower, const glv::Rect& rect )
            : glv::View3D( rect ),
            ScanRenderer2D( windower, glv::Color(1,0,0), 5.0  )

        {
            rotate_z = 0;

            label.setValue( "LMS151::Horizontal" );
            label.pos( glv::Place::BL, 0, 0 ).anchor( glv::Place::BL ); 
            (*this) << label;
        }

        void onDraw3D( glv::GLV& g )
        {
            ScanRenderer2D::onDraw3D(g);
        }
    };

    struct VerticalScanRenderer2DView : glv::View3D, ScanRenderer2D
    {

        VerticalScanRenderer2DView( boost::shared_ptr< L3::ConstantTimeIterator< L3::LMS151 > > windower, const glv::Rect& rect )
            : glv::View3D( rect ),
            ScanRenderer2D( windower, glv::Color(0,0,1),0 )
        {
            rotate_z = 180;

            label.setValue( "LMS151::Vertical" );
            label.pos( glv::Place::BL, 0, 0 ).anchor( glv::Place::BL ); 
            (*this) << label;
        }

        void onDraw3D( glv::GLV& g )
        {
            ScanRenderer2D::onDraw3D(g);
        }

    };

    struct CombinedScanRenderer2D  : glv::View3D
    {

        CombinedScanRenderer2D( boost::shared_ptr< L3::ConstantTimeIterator< L3::LMS151 > > windower_vertical,
                boost::shared_ptr< L3::ConstantTimeIterator< L3::LMS151 > > windower_horizontal, 
                const glv::Rect& rect )
            : glv::View3D(rect)

        {
            scan_renderers.push_back( boost::make_shared<ScanRenderer2D>( windower_vertical, glv::Color(1,0,0) ) );
            scan_renderers.push_back( boost::make_shared<ScanRenderer2D>( windower_vertical, glv::Color(0,0,1) ) );
        }

        void onDraw3D( glv::GLV& g )
        {
            //REFERENCE TO REFERENCE
            //std::for_each( scan_renderers.begin(), scan_renderers.end(), std::bind2nd( std::mem_fun( &ScanRenderer2D::onDraw3D ), g ) );
            //std::for_each( scan_renderers.begin(), scan_renderers.end(), std::bind2nd( std::mem_fun( &ScanRenderer2D::onDraw3D ), boost::reference_wrapper<glv::GLV>(g) ) );
            //std::for_each( scan_renderers.begin(), scan_renderers.end(), std::bind2nd( std::mem_fun( &ScanRenderer2D::onDraw3D ), std::reference_wrapper(g) ) );

            glv::draw::translateZ( -20 );

            Grid().onDraw3D(g);

            for ( std::list< boost::shared_ptr< ScanRenderer2D > >::iterator it = scan_renderers.begin();
                    it != scan_renderers.end();
                    it++ )
                (*it)->onDraw3D(g);

        }

        std::list< boost::shared_ptr< ScanRenderer2D > > scan_renderers;
    };


    /*
     *  Scan matching renderers
     */
    struct ScanMatchingTrajectoryRenderer : Composite
    {
        ScanMatchingTrajectoryRenderer( boost::shared_ptr< L3::ScanMatching::Engine > engine ) : Composite( glv::Rect(175, 175)), engine(engine)
        {
            *(dynamic_cast< glv::View3D* >(this)) << label;
            label.pos( glv::Place::BL, 0, 0 ).anchor( glv::Place::TL ); 
            label.setValue( "Open-loop trajectory");

            this->disable( glv::Property::AlwaysBubble );

            this->operator<< ( grid );

            controller = boost::make_shared< L3::Visualisers::CompositeController >( dynamic_cast< glv::View3D* >(this), boost::ref( this->position ) );
        }

        boost::weak_ptr< L3::ScanMatching::Engine > engine;

        boost::shared_ptr< Controller> controller;

        glv::Label label;

        Grid grid;

        void onDraw3D( glv::GLV& g );

    };

    struct ScanMatchingScanRenderer : glv::View3D
    {
        ScanMatchingScanRenderer( const glv::Rect rect, boost::shared_ptr< L3::ScanMatching::Engine > engine )
            : glv::View3D( rect ),
            engine(engine)
        {

            label.setValue( "SM engine" );
            label.pos( glv::Place::BL, 0, 0 ).anchor( glv::Place::BL ); 
            (*this) << label;

            trajectory = boost::dynamic_pointer_cast< glv::View3D >( boost::make_shared<ScanMatchingTrajectoryRenderer>( engine ) ) ;

            // We are not visible by default
            (*this) << *trajectory;
            trajectory->disable( glv::Visible );
        }

        boost::weak_ptr< L3::ScanMatching::Engine > engine;

        boost::shared_ptr< glv::View3D > trajectory;

        glv::Label label;

        void onDraw3D( glv::GLV& g );

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

    /*
     *  Pyramid cost rendering
     */
    struct AlgorithmCostRenderer  
    {
        AlgorithmCostRenderer( boost::shared_ptr< L3::Estimator::Algorithm<double> > algorithm ) 
            : algorithm(algorithm)
        {

        }

        virtual ~AlgorithmCostRenderer()
        {
        }

        boost::weak_ptr< L3::Estimator::Algorithm<double> > algorithm ;
    };

    struct AlgorithmCostRendererLeaf : AlgorithmCostRenderer, Leaf
    {
        AlgorithmCostRendererLeaf( boost::shared_ptr< L3::Estimator::Algorithm<double> > algorithm ) 
            : AlgorithmCostRenderer(algorithm)

        {

        }

        void onDraw3D( glv::GLV& g );

    };

    /*
     *  Locale: Bounds Renderer
     */

    struct LocaleBoundsRenderer : Leaf
    {
        LocaleBoundsRenderer()
        {
        }

        void onDraw3D(glv::GLV& g);

    };

    /*
     *  Experience views
     */
    struct ExperienceView
    {

        ExperienceView( boost::shared_ptr<L3::Experience> experience, boost::shared_ptr< L3::PoseProvider > provider ) 
            : experience(experience), provider(provider)
        {
        }

        boost::weak_ptr< L3::PoseProvider > provider;
        boost::weak_ptr< L3::Experience >   experience;
        boost::shared_array< glv::Color >   experience_nodes_colors;
        boost::shared_array< glv::Point3 >  experience_nodes_vertices;

    };

    struct ExperienceCloudView : ExperienceView, glv::View3D
    {

        ExperienceCloudView( const glv::Rect& rect, 
                boost::shared_ptr<L3::Experience> experience, 
                boost::shared_ptr< L3::PoseProvider > provider = boost::shared_ptr<L3::PoseProvider>() ) 
            : ExperienceView( experience, provider ), 
            glv::View3D(rect)
        {

            octree = boost::make_shared< pcl::octree::OctreePointCloudDensity<pcl::PointXYZ> >( 0.5f ); 
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

        std::vector< glv::Point3 >  vertices;
        std::vector< glv::Color  >  colors;

        //std::deque< Cube > voxels;

        boost::shared_ptr< PointCloud<double> > master;

        std::list< boost::shared_ptr< PointCloud<double> > > clouds;

        boost::shared_ptr< PointCloudRendererLeaf > point_cloud_renderer_leaf;

        boost::shared_ptr< pcl::octree::OctreePointCloudDensity<pcl::PointXYZ> > octree;

        void load( boost::shared_ptr< L3::Experience > experience );

        void onDraw3D( glv::GLV& g );
    };

    struct ExperienceOverviewView : ExperienceView, glv::View3D
    {
        ExperienceOverviewView( const glv::Rect& rect, 
                boost::shared_ptr<L3::Experience> experience, 
                boost::shared_ptr< L3::PoseProvider > provider = boost::shared_ptr<L3::PoseProvider>() ) 
            : ExperienceView( experience, provider ), 
            glv::View3D(rect)
        {

            label.setValue( "Experience" );
            label.pos( glv::Place::BL, 0, 0 ).anchor( glv::Place::BL ); 
            (*this) << label;

            current = boost::make_shared< L3::SE3 >();

            animation = boost::make_shared< AnimatedPoseRenderer >( boost::ref( *current ) );

            experience_point_cloud = boost::make_shared< ExperienceCloudView >( glv::Rect( 175, 175 ), experience, provider );

            *this << *experience_point_cloud;
            experience_point_cloud->disable( glv::Visible );
        }

        boost::shared_ptr< AnimatedPoseRenderer > animation;
        boost::shared_ptr< ExperienceCloudView > experience_point_cloud;
        boost::shared_ptr< L3::SE3 > current;
        glv::Label label;


        void setProvider( boost::shared_ptr< L3::PoseProvider > provider )
        {
            this->provider = provider;
            this->experience_point_cloud->provider = provider;
        }

        void setExperience( boost::shared_ptr< L3::Experience > experience )
        {
            this->experience = experience;
            this->experience_point_cloud->load( experience );
        }

        void onDraw3D(glv::GLV& g);

    };

    struct DatasetOverviewView : glv::View3D
    {
        DatasetOverviewView( const glv::Rect& rect, boost::shared_ptr< L3::Dataset > dataset, boost::shared_ptr< L3::PoseProvider > provider = boost::shared_ptr< L3::PoseProvider >() ) 
            : glv::View3D(rect), 
            provider( provider )
        {
            boost::scoped_ptr <L3::IO::BinaryReader< L3::SE3 > > pose_reader( ( new L3::IO::BinaryReader<L3::SE3>() ) ) ;

            if (pose_reader->open( dataset->path() + "/OxTS.ins" ) )
            {
                poses.reset ( new std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > > () );
                pose_reader->read();
                pose_reader->extract( *poses );
            }

            current_pose = boost::make_shared< L3::SE3 >();

            renderer = boost::make_shared< L3::Visualisers::AnimatedPoseRenderer >( boost::ref( *current_pose ) );
        }

        boost::shared_ptr< std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > > > poses;
        boost::shared_ptr< L3::Visualisers::AnimatedPoseRenderer > renderer;
        boost::weak_ptr< L3::PoseProvider > provider;
        boost::shared_ptr< L3::SE3 > current_pose;

        void onDraw3D( glv::GLV& g );
    };

    /*
     *  Histograms
     */
    struct HistogramRenderer 
    {
        HistogramRenderer( boost::shared_ptr<L3::Histogram<double> > histogram ) : hist(histogram)
        {

        }

        boost::weak_ptr<L3::Histogram<double> > hist;
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
     *  Histogram :: Density renderer
     */
    struct HistogramDensityRenderer : glv::View, HistogramRenderer, Updateable
    {
        HistogramDensityRenderer(const glv::Rect& rect, boost::shared_ptr<L3::Histogram<double> > histogram )
            : glv::View(rect), 
            HistogramRenderer(histogram),
            mTex(0,0,GL_RGBA,GL_UNSIGNED_BYTE)
        {
            render_histogram = boost::make_shared< L3::Histogram<double> >();
        }

        glv::Texture2 mTex;

        boost::shared_ptr< L3::Histogram<double> > render_histogram;

        void update();
        void onDraw( glv::GLV& g );
    };

    /*
     *  Histogram :: Voxel Renderer
     */
    struct HistogramVoxelRenderer : HistogramRenderer, Updateable
    {
        HistogramVoxelRenderer(boost::shared_ptr<L3::Histogram<double> > histogram  )
            : HistogramRenderer(histogram)
        {
            plot_histogram = boost::make_shared< L3::Histogram<double> >();
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

            // Obtain the pointer
            boost::shared_ptr<L3::Histogram<double> > hist_ptr = hist.lock();

            if ( !hist_ptr)
                return;

            if( hist_ptr->empty() )
                return;

            L3::ReadLock lock( hist_ptr->mutex );
            L3::clone( hist_ptr.get(), plot_histogram.get() );

            std::pair<float, float> lower_left = hist_ptr->coords(0,0);
            std::pair<float, float> upper_right = hist_ptr->coords( hist_ptr->x_bins, hist_ptr->y_bins );

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
            // Obtain the pointer
            boost::shared_ptr<L3::Histogram<double> > hist_ptr = hist.lock();

            if ( !hist_ptr)
                return;

            L3::ReadLock lock( hist_ptr->mutex );

            if ( !hist_ptr->empty() ) 
                L3::clone( hist_ptr.get(), plot_histogram.get() );
            lock.unlock();

            HistogramVoxelRenderer::onDraw3D(g);    
        }

    };


    /*
     *  Histogram :: Vertex Renderer
     */
    struct HistogramVertexRenderer : HistogramRenderer, glv::View
    {
        HistogramVertexRenderer( const glv::Rect rect, boost::shared_ptr<L3::Histogram<double> > histogram ) 
            : HistogramRenderer(histogram), glv::View(rect)
        {
        }

        void onDraw(glv::GLV& g);
    };


    /*
     *  Pyramid renderer
     */
    struct HistogramPyramidRenderer
    {
        HistogramPyramidRenderer( boost::shared_ptr<L3::HistogramPyramid<double> > histogram_pyramid) 
            : pyramid(histogram_pyramid)
        {
        }

        boost::weak_ptr<L3::HistogramPyramid<double> > pyramid;

    };

    struct HistogramPyramidRendererView : glv::Table, HistogramPyramidRenderer, Updateable
    {
        HistogramPyramidRendererView( boost::shared_ptr<L3::HistogramPyramid<double> > histogram_pyramid, int num_pyramids ) 
            : HistogramPyramidRenderer(histogram_pyramid),
            glv::Table( "x x x, "),
            num_pyramids(num_pyramids)
        {
            int width = 180;
            int start = 0;
            for( int i=0; i< num_pyramids; i++ )
            {
                boost::shared_ptr< HistogramDensityRenderer > renderer = boost::make_shared< HistogramDensityRenderer >( glv::Rect( width, width), boost::shared_ptr< Histogram<double > >() );

                std::stringstream ss;

                ss << "Pyramid <" << i << ">";

                boost::shared_ptr< glv::Label > label = boost::make_shared< glv::Label >( ss.str() );
                label->pos( glv::Place::BR, 0, 0 ).anchor( glv::Place::BR ); 
                *renderer << *label;

                renderers.push_back( renderer );
                labels.push_back( label );
                (*this) << renderer.get();
            }

            if( histogram_pyramid )
                loadPyramid( histogram_pyramid );

            this->arrange();
        }

        int num_pyramids;
        std::deque< boost::shared_ptr< HistogramDensityRenderer > > renderers;

        std::deque < boost::shared_ptr< glv::View > > labels;

        void loadPyramid( boost::shared_ptr<L3::HistogramPyramid<double> > histogram_pyramid )
        {
            this->pyramid = histogram_pyramid;

            boost::shared_ptr< L3::HistogramPyramid<double > > pyramid_ptr = this->pyramid.lock();

            if( !pyramid_ptr )
                return;

            if( std::distance( pyramid_ptr->begin(), pyramid_ptr->end() ) > num_pyramids )
            {
                // We have too many pyramids to render, I don't think this will
                // ever be an issue. 
            }

            int counter = 0;
            for( L3::HistogramPyramid<double>::PYRAMID_ITERATOR it = pyramid_ptr->begin();
                    it != pyramid_ptr->end();
                    it++ )
                renderers[counter++]->hist = *it;
        }

        void update();

    };

    template <typename T>
        struct BasicPlottable : glv::Plottable,  Updateable, Lockable
    {

        BasicPlottable() : glv::Plottable( glv::draw::LineStrip, 1 )
        {

        }
        virtual void onMap( glv::GraphicsData& g, const glv::Data& d, const glv::Indexer& i)
        {

        }

        virtual void update()
        {

        }

        std::deque<T> plot_data;

    };

    /*
     *  Statistics 
     */
    template <typename T>
        struct StatisticsPlottable  : BasicPlottable<T>
    {
        StatisticsPlottable() 
        {
            this->color( glv::Color( 1, 0, 0, .5 ) );
            this->stroke(2);
        }

        void onMap( glv::GraphicsData& g, const glv::Data& d, const glv::Indexer& i)
        {
            while(i()){
                double x = i[0];
                double y = d.at<double>(0, i[0]);
                g.addVertex(x, y);
            }
        }

        boost::shared_ptr< variable_lock<T> > lock;

        void setVariable( T& t )
        {
            lock = boost::make_shared< variable_lock<T> >( boost::ref( t ) );
        }

        void update()
        {
            if (!lock)
                return;

            this->plot_data.push_back( lock->t );

            if ( this->plot_data.size() > 100 )
                this->plot_data.pop_front();

            this->mData.resize( glv::Data::DOUBLE, 1, this->plot_data.size() );

            glv::Indexer i( this->mData.size(1));

            int counter = 0;

            while( i() && counter< this->plot_data.size() )
            {
                this->mData.assign( this->plot_data[counter]*10, i[0], i[1] );
                counter++;
            }

        }

    };

    struct Statistics : glv::Table
    {
        Statistics();

        std::vector< boost::shared_ptr< glv::Plot > > plots;
        std::vector< boost::shared_ptr< glv::Label > > labels;
        std::vector< boost::shared_ptr< StatisticsPlottable<double> > > plottables;

        boost::shared_ptr< TextRenderer<double> > observer_update;;
        boost::shared_ptr< TextRenderer<double> > swathe_generation;;
        boost::shared_ptr< TextRenderer<double> > points_per_second;
        boost::shared_ptr< TextRenderer<double> > estimation;
        boost::shared_ptr< TextRenderer<double> > total;

        boost::shared_ptr< glv::TextView > memory_statistics;

        void load( L3::DatasetRunner* runner )
        {
            observer_update->setVariable(   runner->timings[0] );
            swathe_generation->setVariable( runner->timings[1] );
            points_per_second->setVariable( runner->timings[2] );
            estimation->setVariable(        runner->timings[3] );

            plottables[0]->setVariable( runner->timings[0] );
            plottables[1]->setVariable( runner->timings[1] );
            plottables[2]->setVariable( runner->timings[2] );
            plottables[3]->setVariable( runner->timings[3] );
        }

    };

    struct ParticleFilterRendererLeaf : Leaf, Updateable
    {
        ParticleFilterRendererLeaf( boost::shared_ptr< L3::Estimator::ParticleFilter<double> > filter ) : filter(filter)
        {
        }

        boost::weak_ptr< L3::Estimator::ParticleFilter<double> > filter;

        std::vector< L3::SE3 > hypotheses;

        void onDraw3D( glv::GLV& g );
   
        void update();
    };

    /*
     *  Chase
     */
    struct ChaseController : Controller, Leaf
    {
        ChaseController( glv::View3D* view, control_t& position, L3::SE3& chase ) 
            : Controller(position),
            chase(chase)
        {
        }

        L3::SE3& chase;

        void onDraw3D( glv::GLV& g );
    };



}   // Visualisers
}   // L3

#endif

