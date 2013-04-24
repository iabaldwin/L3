#include "Components.h"

#include <boost/scoped_ptr.hpp>

namespace L3
{
namespace Visualisers
{

/*
 *  Components :: Grid
 */
Grid::Grid()
{
    vertices.reset( new glv::Point3[ 100*4 ] );
    colors.reset( new glv::Color[ 100*4 ] );

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

void Grid::onDraw3D(glv::GLV& g)
{ 
    glv::draw::lineWidth( .01 );
    glv::draw::paint( glv::draw::Lines, vertices.get(), colors.get(), counter );
}

/*
 *  Components :: DefaultAxes
 */
DefaultAxes::DefaultAxes()
{
    vertices.reset( new glv::Point3[ 6 ] );
    colors.reset( new glv::Color[ 6 ] );
    
    glv::Color c =  glv::Color( 112.f/255.f, 138.f/255.f, 144.f/255.f ) ; 
    
    counter = 0;

    colors[counter] = c;
    vertices[counter++]( -100, 0, 0 );
    colors[counter] = c;
    vertices[counter++]( 100, 0, 0 );
    colors[counter] = c;
    vertices[counter++]( 0 , -100, 0 );
    colors[counter] = c;
    vertices[counter++]( 0 ,100, 0 );
    colors[counter] = c;
    vertices[counter++]( 0 , 0, -100 );
    colors[counter] = c;
    vertices[counter++]( 0 ,0 , 100 );

}

void DefaultAxes::onDraw3D(glv::GLV& g)
{ 
    glv::draw::lineWidth( .01 );
    glv::draw::paint( glv::draw::Lines, vertices.get(), colors.get(), counter );
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

    L3::Histogram<double> tmp;
    
    L3::ReadLock lock( hist->mutex );
    L3::clone( hist.get(), &tmp );
    lock.unlock();   

    float x_delta = tmp.x_delta;
    float y_delta = tmp.y_delta;

    for( unsigned int i=0; i < tmp.x_bins; i++ )
    {
        for( unsigned int j=0; j < tmp.y_bins; j++ )
        {
            unsigned int val = tmp.bin( i, j );

            glv::Color c = glv::Color( val/10.0 );

            std::pair<float,float> coords = tmp.coords( i, j);

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

void HistogramPixelRenderer::update()
{
    L3::Histogram<double> tmp;
    
    if( hist->empty() )
        return;

    L3::ReadLock lock( hist->mutex );
    L3::clone( hist.get(), &tmp );
    lock.unlock();   

    
    //L3::ReadLock( hist->mutex );
    // Aspect ratio?
    //float aspect_ratio = (float)hist->y_bins/(float)hist->x_bins;
    //int w = this->width();
    //this->height( w * aspect_ratio );

    data().resize( glv::Data::FLOAT, 1, tmp.x_bins, tmp.y_bins );

    for( unsigned int i=0; i< tmp.x_bins; i++ )
    {
        for( unsigned int j=0; j< tmp.y_bins; j++ )
        {
            data().assign( tmp.bin(i,j)/10.0 , 0, i, j );
        }
    }
}

/*
 *Components :: CoordinateSystem
 */
CoordinateSystem::CoordinateSystem( L3::SE3& pose, float scale ) : pose( pose ), scale(scale)
{
    vertices.reset( new glv::Point3[6] );

    vertices[0]( 0, 0, 0 ); 
    vertices[1]( scale, 0, 0 ); 
    vertices[2]( 0, 0, 0 ); 
    vertices[3]( 0, scale, 0 ); 
    vertices[4]( 0, 0, 0 ); 
    vertices[5]( 0, 0, scale ); 

    colors.reset( new glv::Color[6] );
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

    //glMultMatrixf( pose->getHomogeneous().data() );
    glMultMatrixf( pose.getHomogeneous().data() );

    glv::draw::paint( glv::draw::LineStrip, vertices.get(), colors.get(), 6 );

    glv::draw::pop();
}

/*
 *  Point cloud :: vertex renderer
 */
    PointCloudRenderer::PointCloudRenderer( L3::PointCloud<double>* cloud ) : cloud(cloud)
    {
        plot_cloud.reset( new L3::PointCloud<double>() );

        L3::allocate( plot_cloud.get(), 5*1000 );

        colors.reset( new glv::Color[plot_cloud->num_points] );
        vertices.reset( new glv::Point3[plot_cloud->num_points] );
    
    };
    
    void PointCloudRenderer::onDraw3D( glv::GLV& g )
    {
        boost::scoped_ptr< L3::PointCloud<double> > point_cloud( new L3::PointCloud<double>()  );
      
        L3::ReadLock lock( cloud->mutex );
        L3::copy( cloud, point_cloud.get() );
        lock.unlock();

        if ( point_cloud->num_points > 0 )
        {

            L3::sample( point_cloud.get(), plot_cloud.get(), plot_cloud->num_points );

            for( int i=0; i<plot_cloud->num_points; i++) 
                vertices[i]( plot_cloud->points[i].x, plot_cloud->points[i].y, plot_cloud->points[i].z); 
                //colors[i].set(  

            glv::draw::paint( glv::draw::Points, vertices.get(), colors.get(), plot_cloud->num_points);

        }
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
        points[ counter++ ]( it->X(), it->Y(), 0.0 );
        it++;
    }

    glv::draw::paint( glv::draw::Points, points, colors, counter );
}

/*
 *  Single pose orientation renderer
 */

void DedicatedPoseRenderer::update()
{
    *pose = provider->operator()();
}

void DedicatedPoseRenderer::onDraw3D(glv::GLV& g)
{
    L3::SE3 tmp( *pose );

    // Centre
    tmp.X( 0 );
    tmp.Y( 0 );
    tmp.Z( 0 );
   
    //far(1000);

    glv::draw::push3D( -5.0, 5.0, .1f, 150.0, 85 );

    glv::draw::translateZ( -10 );
    glv::draw::translateY( 1 );
    //glv::draw::rotate( 135 , 0 , 45 );
    //glv::draw::rotate( 45 , 0 , 45 );
    //glv::draw::rotate( 0 , 0 , 55 );

    glv::draw::rotateX( 245 );
    glv::draw::rotateZ( 15 );

    
    CoordinateSystem( tmp ).onDraw3D( g );

    axes.onDraw3D( g );

    glv::draw::pop3D();
}


}
}
