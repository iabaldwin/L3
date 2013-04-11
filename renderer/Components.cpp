#include "Components.h"
namespace L3
{
namespace Visualisers
{

/*
 *  Components :: Composite
 */
void Composite::onDraw3D( glv::GLV& g )
{
    glv::draw::translate( position.x, position.y, position.z );
    glv::draw::rotate( position.r, position.p, position.q );

    // 1. Compute time since last update
    current = clock();
    double elapsed = double(current - previous)/CLOCKS_PER_SEC;

    // Takes care of initial inf.
    elapsed  = (elapsed > 1.0) ? 1.0 : elapsed;
    
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

/*
 *  Components :: Grid
 */
Grid::Grid()
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

Grid::~Grid()
{
    delete [] vertices;
}

void Grid::onDraw3D(glv::GLV& g)
{ 
    glv::draw::lineWidth( .01 );
    glv::draw::paint( glv::draw::Lines, vertices, colors, counter );
}

/*
 *  Components :: HistogramBoundsRenderer
 */
void HistogramBoundsRenderer::onDraw3D(glv::GLV& g)
{
    glv::Point3 bound_vertices[4];
    glv::Color  bound_colors[4];

    L3::ReadLock( hist->mutex );

    std::pair<float, float> lower_left = hist->coords(0,0);
    std::pair<float, float> upper_right = hist->coords( hist->x_bins, hist->y_bins );

    bound_vertices[0]( lower_left.first, lower_left.second, 0.0 );
    bound_colors[0].set( 1, 1, 0 );
    bound_vertices[1]( lower_left.first, upper_right.second, 0.0 );
    bound_colors[1].set( 1, 1, 0 );
    bound_vertices[2]( upper_right.first, upper_right.second, 0.0 );
    bound_colors[2].set( 1, 1, 0 );
    bound_vertices[3]( upper_right.first, lower_left.second, 0.0 );
    bound_colors[3].set( 1, 1, 0 );

    glv::draw::lineWidth(1.5);

    glv::draw::paint( glv::draw::LineLoop, bound_vertices, bound_colors, 4 );
}


/*
 *  Components :: HistogramVertexRenderer
 */

void HistogramVertexRenderer::onDraw3D( glv::GLV& g)
{
    glv::Point3 quad_vertices[4];
    glv::Color quad_colors[4];

    L3::ReadLock( hist->mutex );
    
    float x_delta = hist->x_delta;
    float y_delta = hist->y_delta;

    for( unsigned int i=0; i < hist->x_bins; i++ )
    {
        for( unsigned int j=0; j < hist->y_bins; j++ )
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

/*
 *  Components :: HistogramPixelRenderer
 */

void HistogramPixelRenderer::run()
{
    while( running )
    {
        if( t.elapsed() > 1.0 )
        {
            t.restart();

            L3::ReadLock( hist->mutex );

            // Aspect ratio?
            //float aspect_ratio = (float)hist->y_bins/(float)hist->x_bins;
            //int w = this->width();
            //this->height( w * aspect_ratio );

            data().resize( glv::Data::FLOAT, 1, hist->x_bins, hist->y_bins );

            for( unsigned int i=0; i< hist->x_bins; i++ )
            {
                for( unsigned int j=0; j< hist->y_bins; j++ )
                {
                    data().assign( hist->bin(i,j)/10.0 , 0, i, j );
                }
            }
        }
    }
}

/*
 *Components :: CoordinateSystem
 */
CoordinateSystem::CoordinateSystem()  : _pose( L3::SE3::ZERO() ), vertices(NULL), colors(NULL)
{
    _init();
}

CoordinateSystem::CoordinateSystem( const L3::SE3& pose ) : _pose( pose )
{
    _init();
}

CoordinateSystem::~CoordinateSystem()
{
    if ( vertices )
        delete [] vertices;
    if ( colors )
        delete [] colors;
}

void CoordinateSystem::_init()
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

void CoordinateSystem::onDraw3D(glv::GLV& g)
{ 
    glv::draw::push();

    glv::draw::translate( _pose.x, _pose.y, _pose.z );
    glv::draw::rotate( L3::Utils::Math::radiansToDegrees(_pose.r), 
            L3::Utils::Math::radiansToDegrees(_pose.p),
            L3::Utils::Math::radiansToDegrees(_pose.q));

    glv::draw::paint( glv::draw::LineStrip, vertices, colors, 6 );

    glv::draw::pop();
}


/*
 *  Point cloud :: vertex renderer
 */
    PointCloudRenderer::PointCloudRenderer( L3::PointCloud<double>* CLOUD ) : cloud(CLOUD)
    {
            
    };
    
    void PointCloudRenderer::onDraw3D( glv::GLV& g )
    {
        L3::ReadLock( cloud->mutex );
        colors   = new glv::Color[cloud->num_points];
        vertices = new glv::Point3[cloud->num_points];

        for( int i=0; i<cloud->num_points; i++) 
        {
            vertices[i]( cloud->points[i].x, cloud->points[i].y, cloud->points[i].z); 
        }

        glv::draw::paint( glv::draw::Points, &*vertices, &*colors, cloud->num_points);

        delete [] colors;
        delete [] vertices;

    }


/*
 *  Point cloud :: bounds renderer
 */

PointCloudBoundsRenderer::PointCloudBoundsRenderer( L3::PointCloud<double>* point_cloud ) :
    cloud(point_cloud)
{
}

void PointCloudBoundsRenderer::onDraw3D(glv::GLV& g)
{ 

    glv::Point3 bound_vertices[4];
    glv::Color  bound_colors[4];

    L3::ReadLock point_cloud_lock( cloud->mutex );
    std::pair<double, double> lower_left = min( cloud );
    std::pair<double, double> upper_right = max(cloud);
    point_cloud_lock.unlock();

    bound_vertices[0]( lower_left.first, lower_left.second, 0.0 );
    bound_colors[0].set( 0, 1, 1 );
    bound_vertices[1]( lower_left.first, upper_right.second, 0.0 );
    bound_colors[1].set( 0, 1, 1 );
    bound_vertices[2]( upper_right.first, upper_right.second, 0.0 );
    bound_colors[2].set( 0, 1, 1 );
    bound_vertices[3]( upper_right.first, lower_left.second, 0.0 );
    bound_colors[3].set( 0, 1, 1 );

    glv::draw::lineWidth(1.5);

    glv::draw::paint( glv::draw::LineLoop, bound_vertices, bound_colors, 4 );

}

/*
 *  Pose prediction
 */

void PoseEstimatesRenderer::onDraw3D( glv::GLV& g )
{
    glv::Point3 points[ pose_estimates->estimates.size() ];
    glv::Color  colors[ pose_estimates->estimates.size() ];

    std::vector< L3::SE3 >::iterator it = pose_estimates->estimates.begin();

    int counter = 0;
    while( it != pose_estimates->estimates.end() )
    {
        points[ counter++ ]( it->x, it->y, 0.0 );
        it++;
    }

    glv::draw::paint( glv::draw::Points, points, colors, counter );


}

}
}
