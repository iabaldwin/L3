#include "Components.h"

#include <boost/scoped_ptr.hpp>
#include <boost/scoped_array.hpp>

#include "RenderingUtils.h"

#include <mach/vm_statistics.h>
#include <mach/mach_types.h> 
#include <mach/mach_init.h>
#include <mach/mach_host.h>
#include <mach/mach.h> 

namespace L3
{
namespace Visualisers
{
    void transformCameraToPose( L3::SE3& pose )
    {

        // SE3->GL
        L3::SE3 rotation( 0, 0, 0, 0, -1.57, 0 );
        glMultMatrixf( rotation.getHomogeneous().data() );
        
        L3::SE3 rot( 0, 0, 0, -1*pose.R(), -1*pose.P(), -1*pose.Q() );
        glMultMatrixf( rot.getHomogeneous().data() );

        L3::SE3 translate( -1*pose.X(), -1*pose.Y(), -1*pose.Z(), 0, 0, 0 );
        glMultMatrixf( translate.getHomogeneous().data() );
    }

    void renderCube( Cube* cube )
    {
        int num_tris = 6;
        
        glv::Point3 triangle_vertices[num_tris*6];
        glv::Color  triangle_colors[num_tris*6];

        glv::Point3 line_points[12*2];
        glv::Color  line_colors[12*2];

        std::fill( triangle_colors, triangle_colors+(num_tris*6), glv::Color( .5, .5, .5, cube->opacity ) );
        std::fill( line_colors, line_colors+12*2, glv::Color( .5, .5, .5, 1-cube->opacity ) );

        int counter = 0;

        // Bottom
        line_points[counter++]( cube->x_lower, cube->y_lower, cube->z_lower );
        line_points[counter++]( cube->x_lower, cube->y_upper, cube->z_lower );
        line_points[counter++]( cube->x_lower, cube->y_upper, cube->z_lower );
        line_points[counter++]( cube->x_upper, cube->y_upper, cube->z_lower );
        line_points[counter++]( cube->x_upper, cube->y_upper, cube->z_lower );
        line_points[counter++]( cube->x_upper, cube->y_lower, cube->z_lower );
        line_points[counter++]( cube->x_upper, cube->y_lower, cube->z_lower );
        line_points[counter++]( cube->x_lower, cube->y_lower, cube->z_lower );
        
        line_points[counter++]( cube->x_lower, cube->y_lower, cube->z_upper  );
        line_points[counter++]( cube->x_lower, cube->y_upper, cube->z_upper  );
        line_points[counter++]( cube->x_lower, cube->y_upper, cube->z_upper  );
        line_points[counter++]( cube->x_upper, cube->y_upper, cube->z_upper  );
        line_points[counter++]( cube->x_upper, cube->y_upper, cube->z_upper  );
        line_points[counter++]( cube->x_upper, cube->y_lower, cube->z_upper  );
        line_points[counter++]( cube->x_upper, cube->y_lower, cube->z_upper  );
        line_points[counter++]( cube->x_lower, cube->y_lower, cube->z_upper  );


        glv::draw::lineWidth( 2 );
        glv::draw::enable( glv::draw::Blend );
        glv::draw::paint( glv::draw::Lines, line_points, line_colors, counter );
        glv::draw::disable( glv::draw::Blend );
        glv::draw::lineWidth( 1 );

        counter = 0;
        
        // Bottom
        triangle_vertices[counter++]( cube->x_lower, cube->y_lower, cube->z_lower );
        triangle_vertices[counter++]( cube->x_lower, cube->y_upper, cube->z_lower );
        triangle_vertices[counter++]( cube->x_upper, cube->y_upper, cube->z_lower );
        
        triangle_vertices[counter++]( cube->x_lower, cube->y_lower, cube->z_lower );
        triangle_vertices[counter++]( cube->x_upper, cube->y_upper, cube->z_lower );
        triangle_vertices[counter++]( cube->x_upper, cube->y_lower, cube->z_lower );

        // Top
        triangle_vertices[counter++]( cube->x_lower, cube->y_lower, cube->z_upper );
        triangle_vertices[counter++]( cube->x_lower, cube->y_upper, cube->z_upper );
        triangle_vertices[counter++]( cube->x_upper, cube->y_upper, cube->z_upper );
        
        triangle_vertices[counter++]( cube->x_lower, cube->y_lower, cube->z_upper );
        triangle_vertices[counter++]( cube->x_upper, cube->y_upper, cube->z_upper );
        triangle_vertices[counter++]( cube->x_upper, cube->y_lower, cube->z_upper );

        // Left
        triangle_vertices[counter++]( cube->x_lower, cube->y_lower, cube->z_lower );
        triangle_vertices[counter++]( cube->x_upper, cube->y_lower, cube->z_lower );
        triangle_vertices[counter++]( cube->x_upper, cube->y_lower, cube->z_upper );
        
        triangle_vertices[counter++]( cube->x_lower, cube->y_lower, cube->z_lower );
        triangle_vertices[counter++]( cube->x_upper, cube->y_lower, cube->z_upper );
        triangle_vertices[counter++]( cube->x_lower, cube->y_lower, cube->z_upper );


        // Right
        triangle_vertices[counter++]( cube->x_lower, cube->y_upper, cube->z_lower );
        triangle_vertices[counter++]( cube->x_upper, cube->y_upper, cube->z_lower );
        triangle_vertices[counter++]( cube->x_upper, cube->y_upper, cube->z_upper );
        
        triangle_vertices[counter++]( cube->x_lower, cube->y_upper, cube->z_lower );
        triangle_vertices[counter++]( cube->x_upper, cube->y_upper, cube->z_upper );
        triangle_vertices[counter++]( cube->x_lower, cube->y_upper, cube->z_upper );

        // Back
        triangle_vertices[counter++]( cube->x_lower, cube->y_lower, cube->z_lower );
        triangle_vertices[counter++]( cube->x_lower, cube->y_upper, cube->z_lower );
        triangle_vertices[counter++]( cube->x_lower, cube->y_upper, cube->z_upper );
        
        triangle_vertices[counter++]( cube->x_lower, cube->y_lower, cube->z_lower );
        triangle_vertices[counter++]( cube->x_lower, cube->y_upper, cube->z_upper );
        triangle_vertices[counter++]( cube->x_lower, cube->y_lower, cube->z_upper );

        // Front
        triangle_vertices[counter++]( cube->x_upper, cube->y_lower, cube->z_lower );
        triangle_vertices[counter++]( cube->x_upper, cube->y_upper, cube->z_lower );
        triangle_vertices[counter++]( cube->x_upper, cube->y_upper, cube->z_upper );
        
        triangle_vertices[counter++]( cube->x_upper, cube->y_lower, cube->z_lower );
        triangle_vertices[counter++]( cube->x_upper, cube->y_upper, cube->z_upper );
        triangle_vertices[counter++]( cube->x_upper, cube->y_lower, cube->z_upper );


        glv::draw::enable( glv::draw::Blend );
        glv::draw::paint( glv::draw::Triangles, triangle_vertices, triangle_colors, counter );
        glv::draw::disable( glv::draw::Blend );

    }

    /*
     *  Composite
     */
    void Composite::onDraw3D( glv::GLV& g )
    {
        glv::draw::rotate( position.r, position.p, position.q ); 
        glv::draw::translate( position.x, position.y, position.z );

        glGetDoublev(GL_PROJECTION_MATRIX, projection );
        glGetDoublev(GL_MODELVIEW_MATRIX, model );
        glGetIntegerv(GL_VIEWPORT,viewport);

        std::list<Leaf*>::iterator leaf_iterator = components.begin();

        L3::SE3 origin = L3::SE3::ZERO();
        CoordinateSystem( origin, 20.0 ).onDraw3D(g);

        while( leaf_iterator != components.end() )
        {
            if( (*leaf_iterator)->visible )
            {
                (*leaf_iterator)->onDraw3D( g );

                if( ( *leaf_iterator )->draw_bounds )
                    ( *leaf_iterator )->drawBounds();

            }
                
            leaf_iterator++;
        }
    }

    /*
     *  Leaf
     */
    void Leaf::drawBounds()
    {
        glv::Color fill_color = glv::Color( .75, .75, .75, .85  );
        std::fill( bound_colors.get(), bound_colors.get()+4, fill_color );

        int counter = 0;

        // Bottom
        bound_vertices[counter++]( lower.x, lower.y, lower.z );
        bound_vertices[counter++]( lower.x, upper.y, lower.z );
        bound_vertices[counter++]( upper.x, upper.y, lower.z );
        bound_vertices[counter++]( upper.x, lower.y, lower.z );


        glv::draw::enable( glv::draw::Blend );
        glv::draw::paint( glv::draw::LineLoop, bound_vertices.get(), bound_colors.get(), 4 );

        // Left Lower
        counter = 0;
        bound_vertices[counter++]( lower.x, lower.y, lower.z );
        bound_vertices[counter++]( lower.x, lower.y, upper.z );

        glv::draw::paint( glv::draw::Lines, bound_vertices.get(), bound_colors.get(), 2 );

        // Left upper
        counter = 0;
        bound_vertices[counter++]( lower.x, upper.y, lower.z );
        bound_vertices[counter++]( lower.x, upper.y, upper.z );

        glv::draw::paint( glv::draw::Lines, bound_vertices.get(), bound_colors.get(), 2 );


        // Right upper
        counter = 0;
        bound_vertices[counter++]( upper.x, upper.y, lower.z );
        bound_vertices[counter++]( upper.x, upper.y, upper.z );

        glv::draw::paint( glv::draw::Lines, bound_vertices.get(), bound_colors.get(), 2 );

        // Right Lower
        counter = 0;
        bound_vertices[counter++]( upper.x, lower.y, lower.z );
        bound_vertices[counter++]( upper.x, lower.y, upper.z );

        glv::draw::paint( glv::draw::Lines, bound_vertices.get(), bound_colors.get(), 2 );

        // Top
        counter = 0;
        bound_vertices[counter++]( lower.x, lower.y, upper.z );
        bound_vertices[counter++]( lower.x, upper.y, upper.z );
        bound_vertices[counter++]( upper.x, upper.y, upper.z );
        bound_vertices[counter++]( upper.x, lower.y, upper.z );

        glv::draw::paint( glv::draw::LineLoop, bound_vertices.get(), bound_colors.get(), 4 );
        glv::draw::disable( glv::draw::Blend );

    }

    /*
     *  Components :: Grid
     */
    Grid::Grid( float lower, float upper, float spacing) 
        : lower(lower), upper(upper), spacing(spacing)
    {
        vertices = boost::make_shared< glv::Point3[] >( 101*4 );
        colors = boost::make_shared< glv::Color[] >( 101*4 );

        counter = 0;

        // Add horizontal
        for ( int i=lower; i <= upper; i+=spacing )
        {
            glv::Color c = (counter % 4 == 0 ) ? glv::Color( .82 ) : glv::Color( 112.f/255.f, 138.f/255.f, 144.f/255.f ) ; 

            colors[counter] = c; 
            vertices[counter++]( i, lower, 0);
            colors[counter] = c; 
            vertices[counter++]( i, upper, 0);
        }


        for ( int i=lower; i<=upper; i+=spacing )
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
     *  Grid [spatial]
     */
    void DynamicGrid::onDraw3D(glv::GLV& g)
    { 

        int rounded_x = (int)(current_x)-(int(current_x) % 100) ;
        int rounded_y = (int)(current_y)-(int(current_y) % 100);

        glv::draw::push();
        glv::draw::translate( rounded_x, rounded_y, 0.0);
        Grid::onDraw3D( g );
        glv::draw::pop();
    }


    /*
     *  Components :: DefaultAxes
     */
    DefaultAxes::DefaultAxes()
    {
        vertices = boost::make_shared< glv::Point3[ ] >( 6 );
        colors = boost::make_shared< glv::Color[] >(  6 );

        glv::Color c =  glv::Color( 112.f/255.f, 138.f/255.f, 144.f/255.f ) ; 

        counter = 0;

        colors[counter] = c;
        vertices[counter++]( -10, 0, 0 );
        colors[counter] = c;
        vertices[counter++]( 10, 0, 0 );
        colors[counter] = c;
        vertices[counter++]( 0 , -10, 0 );
        colors[counter] = c;
        vertices[counter++]( 0 ,10, 0 );
        colors[counter] = c;
        vertices[counter++]( 0 , 0, -10 );
        colors[counter] = c;
        vertices[counter++]( 0 ,0 , 10 );

    }

    void DefaultAxes::onDraw3D(glv::GLV& g)
    {
        glv::draw::lineStippling(true);
        glv::draw::lineStipple(4, 0xAAAA );
        glv::draw::lineWidth( .01 );
        glv::draw::paint( glv::draw::Lines, vertices.get(), colors.get(), counter );
        glv::draw::lineStippling(false);
    }

    /*
     *  Pose Renderer types
     */
    void AnimatedPoseRenderer::onDraw3D(glv::GLV& g )
    {
        int num_points = 100;

        static int counter = 0;

        glv::Point3 vertices[num_points];
        glv::Color colors[num_points];

        float angle_spacing = 2*M_PI/num_points;
        float angle = 0;

        float x = pose.X();
        float y = pose.Y();

        for( int i=0; i<num_points; i++ )
        {
            vertices[i]( (range*cos(angle))+x, (range*sin(angle))+y, 0 );
            colors[i].set( 1, 1, 1, 1.0-double(counter)/20.0 ); 
            angle += angle_spacing;
        }

        glv::draw::enable( glv::draw::Blend );
        glv::draw::lineStippling(true);
        glv::draw::lineStipple(8, 0xAAAA );
        glv::draw::lineWidth( ( 20 -counter )/4.0 );
        glv::draw::paint( glv::draw::LineLoop, vertices, colors, num_points );
        glv::draw::disable( glv::draw::Blend );
        glv::draw::lineStippling(false);

        if ( counter++ == 20 )
        {
            range = 1.0f;
            counter = 0; 
        }
        else
            range += 1.5;
    }

    /*
     *  Components :: HistogramBoundsRenderer
     */
    void HistogramBoundsRenderer::onDraw3D(glv::GLV& g)
    {
        glv::Point3 bound_vertices[4];
        glv::Color  bound_colors[4];

        // Obtain the pointer
        boost::shared_ptr<L3::Histogram<double> > hist_ptr = hist.lock();

        if ( !hist_ptr)
            return;

        // Obtain the mutex
        L3::ReadLock( hist_ptr->mutex );

        std::pair<float, float> lower_left = hist_ptr->coords(0,0);
        std::pair<float, float> upper_right = hist_ptr->coords( hist_ptr->x_bins, hist_ptr->y_bins );

        this->lower.x = lower_left.first;
        this->lower.y = lower_left.second;
        this->lower.z = depth;
        this->upper.x = upper_right.first;
        this->upper.y = upper_right.second;
        this->upper.z = 50.0;


        bound_vertices[0]( lower_left.first, lower_left.second, depth );
        bound_colors[0].set( 1, 1, 0, .25 );
        bound_vertices[1]( lower_left.first, upper_right.second, depth );
        bound_colors[1].set( 1, 1, 0, .25);
        bound_vertices[2]( upper_right.first, upper_right.second, depth );
        bound_colors[2].set( 1, 1, 0, .25 );
        bound_vertices[3]( upper_right.first, lower_left.second, depth );
        bound_colors[3].set( 1, 1, 0, .25 );

        glv::draw::lineStippling(true);
        glv::draw::lineStipple(4, 0xAAAA );
        glv::draw::lineWidth(.1);
        glv::draw::paint( glv::draw::LineLoop, bound_vertices, bound_colors, 4 );
        glv::draw::lineStippling(false);

        glv::draw::enable( glv::draw::Blend );
        glv::draw::paint( glv::draw::TriangleFan, bound_vertices, bound_colors, 4 );
        glv::draw::disable( glv::draw::Blend );
    }


    /*
     *  Components :: HistogramVertexRenderer
     */

    void HistogramVertexRenderer::onDraw( glv::GLV& g)
    {
        // Obtain the pointer
        boost::shared_ptr<L3::Histogram<double> > hist_ptr = hist.lock();

        if ( !hist_ptr)
            return;

        if (hist_ptr->empty())
            return;

        L3::Histogram<double> tmp;

        L3::ReadLock lock( hist_ptr->mutex );
        L3::clone( hist_ptr.get(), &tmp );
        lock.unlock();   

        float x_delta = tmp.x_delta;
        float y_delta = tmp.y_delta;

        glv::Point3     vertices[ tmp.x_bins*tmp.y_bins];
        glv::Color      colors[ tmp.x_bins*tmp.y_bins];

        int counter = 0;;

        for( unsigned int i=0; i < tmp.x_bins; i++ )
        {
            for( unsigned int j=0; j < tmp.y_bins; j++ )
            {
                unsigned int val = tmp.bin( i, j );

                vertices[counter]( counter, val, 0 );

            }
        }

        glv::draw::paint( glv::draw::Points, vertices, colors, counter );
    }

    /*
     *  Components :: HistogramDensityRenderer
     */
    void HistogramDensityRenderer::update()
    {
        // Obtain the pointer
        boost::shared_ptr<L3::Histogram<double> > hist_ptr = hist.lock();

        if ( !hist_ptr)
            return;

        if( hist_ptr->empty() )
            return;

        L3::ReadLock rlock( hist_ptr->mutex );
        L3::WriteLock wlock( render_histogram->mutex );
        if( !hist_ptr->empty() ) 
            L3::clone( hist_ptr.get(), render_histogram.get() );
        rlock.unlock();   
        wlock.unlock();   
    }

    void HistogramDensityRenderer::onDraw( glv::GLV& g)
    {
        L3::Histogram<double> tmp;

        if( render_histogram->empty() ) 
            return;
        
        L3::ReadLock lock( render_histogram->mutex );
        L3::clone( render_histogram.get(), &tmp ); 
        lock.unlock();

        mTex.magFilter(GL_NEAREST);
        mTex.dealloc();
        mTex.alloc( render_histogram->x_bins, render_histogram->y_bins );

        double normalizer = tmp.max();

        unsigned char * pixs = mTex.buffer<unsigned char>();

        for(int j=0; j< tmp.y_bins; j++ )
        {
            for(int i=0; i<tmp.x_bins; i++ )
            {
                int data = (int)(((tmp.bin(i,j))));
                data  = data*20;

                *pixs++ = (unsigned char)(data);
                *pixs++ = (unsigned char)(data);
                *pixs++ = (unsigned char)(data);
                *pixs++ = (unsigned char)255;

            }

        }

        mTex.recreate();				            // Recreate texture on GPU
        mTex.send();					            // Send over texel data

        glv::draw::enable( glv::draw::Texture2D );	// Enable texture mapping

        mTex.begin();						        // Bind texture
        glv::draw::color(1,1,1,1);				    // Set current color
        mTex.draw(0,0, width(),height() );	        // Draw a textured quad filling the view's area
        mTex.end();							        // Unbind texture

        glv::draw::disable( glv::draw::Texture2D);	// Disable texture mapping
    }


    /*
     *  Components :: HistogramVoxelRenderer
     */
    void HistogramVoxelRenderer::onDraw3D( glv::GLV& g )
    {
        int counter = 0;

        double max = gsl_histogram2d_max_val( plot_histogram->hist );

        float x_delta = plot_histogram->x_delta;
        float y_delta = plot_histogram->y_delta;

        for( unsigned int i=0; i< plot_histogram->x_bins; i++ )
        {
            for( unsigned int j=0; j< plot_histogram->y_bins; j++ )
            {
                unsigned int val = plot_histogram->bin( i, j );

                if (val > 0)
                {
                    float x = plot_histogram->hist->xrange[i];
                    float y = plot_histogram->hist->yrange[j];

                    // Bottom
                    float scale = float(val)/float(max);

                    float z_val = scale*50.0;

                    glv::Point3 points[4];
                    glv::Color  colors[4];

                    colors[0].set( 0, 0, 1, 1 );
                    points[0](x+x_delta, y+y_delta, z_val );
                    colors[1].set( 0, 0, 1, 1 );
                    points[1](x+x_delta, y-y_delta, z_val );
                    colors[2].set( 0, 0, 1, 1 );
                    points[2](x-x_delta, y-y_delta, z_val );
                    colors[3].set( 0, 0, 1, 1 );
                    points[3]( x-x_delta, y+y_delta, z_val );

                    glv::draw::enable( glv::draw::Blend );
                    glv::draw::paint( glv::draw::TriangleFan, points, colors, 4 );
                    glv::draw::disable( glv::draw::Blend );

                    // Top
                    colors[0].set( 0, 0, 1, 1 );
                    points[0]( x+x_delta, y+y_delta, z_val );
                    colors[1].set( 0, 0, 1, 1 );
                    points[1]( x+x_delta, y-y_delta, z_val );
                    colors[2].set( 0, 0, 1, 1 );
                    points[2]( x-x_delta, y-y_delta, z_val );
                    colors[3].set( 0, 0, 1, 1 );
                    points[3]( x-x_delta, y+y_delta, z_val );

                    glv::draw::enable( glv::draw::Blend );
                    glv::draw::paint( glv::draw::TriangleFan, points, colors, 4 );
                    glv::draw::disable( glv::draw::Blend );

                    // Side 1
                    float x_val = x+x_delta;

                    colors[0].set( 0, 0, 1, scale );
                    points[0]( x_val, y-y_delta, 0 );
                    colors[1].set( 0, 0, 1, scale );
                    points[1]( x_val, y+y_delta, 0 );
                    colors[2].set( 0, 0, 1, scale );
                    points[2]( x_val, y+y_delta, z_val );
                    colors[3].set( 0, 0, 1, scale );
                    points[3]( x_val, y-y_delta, z_val );

                    glv::draw::enable( glv::draw::Blend );
                    glv::draw::paint( glv::draw::TriangleFan, points, colors, 4 );
                    glv::draw::disable( glv::draw::Blend );

                    // Side 2
                    x_val = x-x_delta;
                    colors[0].set( 0, 0, 1, scale );
                    points[0]( x_val, y-y_delta, 0 );
                    colors[1].set( 0, 0, 1, scale );
                    points[1]( x_val, y+y_delta, 0 );
                    colors[2].set( 0, 0, 1, scale );
                    points[2]( x_val, y+y_delta, z_val );
                    colors[3].set( 0, 0, 1, scale );
                    points[3]( x_val, y-y_delta, z_val );

                    glv::draw::enable( glv::draw::Blend );
                    glv::draw::paint( glv::draw::TriangleFan, points, colors, 4 );
                    glv::draw::disable( glv::draw::Blend );

                    // Front 
                    float y_val = y-x_delta;

                    colors[0].set( 0, 0, 1, scale );
                    points[0]( x-x_delta, y_val, 0 );
                    colors[1].set( 0, 0, 1, scale );
                    points[1]( x+x_delta, y_val, 0 );
                    colors[2].set( 0, 0, 1, scale );
                    points[2]( x+x_delta, y_val, z_val );
                    colors[3].set( 0, 0, 1, scale );
                    points[3]( x-x_delta, y_val, z_val );

                    glv::draw::enable( glv::draw::Blend );
                    glv::draw::paint( glv::draw::TriangleFan, points, colors, 4 );
                    glv::draw::disable( glv::draw::Blend );

                    // Back
                    y_val = y+x_delta;

                    colors[0].set( 0, 0, 1, scale );
                    points[0]( x-x_delta, y_val, 1 );
                    colors[1].set( 0, 0, 1, scale );
                    points[1]( x+x_delta, y_val, 1 );
                    colors[2].set( 0, 0, 1, scale );
                    points[2]( x+x_delta, y_val, z_val );
                    colors[3].set( 0, 0, 1, scale );
                    points[3]( x-x_delta, y_val, z_val );

                    glv::draw::enable( glv::draw::Blend );
                    glv::draw::paint( glv::draw::TriangleFan, points, colors, 4 );
                    glv::draw::disable( glv::draw::Blend );

                }

            }
        }
    }

    /*
     *  Components :: CoordinateSystem
     */
    CoordinateSystem::CoordinateSystem( L3::SE3& pose, float scale, float alpha ) 
        : pose( pose ), scale(scale), alpha(alpha)
    {
        vertices = boost::make_shared< glv::Point3[]> (6);

        vertices[0]( 0, 0, 0 ); 
        vertices[1]( scale, 0, 0 ); 
        vertices[2]( 0, 0, 0 ); 
        vertices[3]( 0, scale, 0 ); 
        vertices[4]( 0, 0, 0 ); 
        vertices[5]( 0, 0, scale ); 

        colors.reset( new glv::Color[6] );
        colors[0] = glv::Color( 1, 0, 0, alpha ); 
        colors[1] = glv::Color( 1, 0, 0, alpha ); 
        colors[2] = glv::Color( 0, 1, 0, alpha ); 
        colors[3] = glv::Color( 0, 1, 0, alpha ); 
        colors[4] = glv::Color( 0, 0, 1, alpha ); 
        colors[5] = glv::Color( 0, 0, 1, alpha ); 

    }

    void CoordinateSystem::onDraw3D(glv::GLV& g)
    { 
        glv::draw::push();
        glMultMatrixf( pose.getHomogeneous().data() );
        glv::draw::paint( glv::draw::LineStrip, vertices.get(), colors.get(), 6 );
        glv::draw::pop();
    }

    /*
     *  Component :: Point cloud renderer
     */
    PointCloudRenderer::PointCloudRenderer( boost::shared_ptr< L3::PointCloud<double> > cloud , glv::Color color ) 
        : cloud(cloud),
        color(color)
    {
        // Construct the plot cloud
        plot_cloud.reset( new L3::PointCloud<double>() );

        // Allocate
        L3::allocate( plot_cloud.get(), 5*1000 );

        vertices.reset( new glv::Point3[plot_cloud->num_points] );
        colors.reset( new glv::Color[plot_cloud->num_points] );
    };

    void PointCloudRenderer::onDraw3D( glv::GLV& g )
    {
        boost::tuple<double,double,double> bounds = L3::min( plot_cloud.get() );
        for( int i=0; i<plot_cloud->num_points; i++) 
        {
            vertices[i]( plot_cloud->points[i].x, plot_cloud->points[i].y, plot_cloud->points[i].z); 
            colors[i] = glv::Color( (plot_cloud->points[i].z - bounds.get<2>())/10.0 );
        }

        glv::draw::paint( glv::draw::Points, vertices.get(), colors.get(), plot_cloud->num_points);
    }

    /*
     *  Components :: Point cloud renderer (leaf)
     */
    void PointCloudRendererLeaf::onDraw3D( glv::GLV& g )
    {
        boost::shared_ptr< L3::PointCloud<double> > cloud_ptr = cloud.lock();

        if( !cloud_ptr )
            return;

        if( cloud_ptr->num_points == 0 ) 
            return;

        L3::sample( cloud_ptr.get(), plot_cloud.get(), plot_cloud->num_points, false );

        boost::shared_ptr< L3::PoseProvider > provider_ptr = provider.lock();

        // Transform?
        if( provider_ptr )
        {
            L3::SE3 pose = provider_ptr->operator()();
            L3::transform( plot_cloud.get(), &pose );
        }

        PointCloudRenderer::onDraw3D(g);    
    }

    /*
     *  Components :: Point cloud renderer (view)
     */

    void PointCloudRendererView::onDraw3D( glv::GLV& g )
    {
        far(500);

        // Centering heuristic
        glv::draw::translate( 0, 60, -200 );

        PointCloudRenderer::onDraw3D(g);    

    }

    void PointCloudRendererView::update()
    {
        boost::shared_ptr< L3::PointCloud<double> > cloud_ptr = cloud.lock();

        if( !cloud_ptr )
            return;

        if( cloud_ptr->num_points > 0 ) 
            L3::sample( cloud_ptr.get(), plot_cloud.get(), plot_cloud->num_points, false );
    }

    /*
     *  Point cloud :: bounds renderer
     */
    void PointCloudBoundsRenderer::onDraw3D(glv::GLV& g)
    { 
        glv::Point3 bound_vertices[4];
        glv::Color  bound_colors[4];

        boost::shared_ptr< L3::PointCloud<double> > plot_cloud = cloud.lock();

        if( !plot_cloud )
            return;

        boost::scoped_ptr< L3::PointCloud<double> > tmp( new L3::PointCloud<double>() );

        L3::ReadLock point_cloud_lock( plot_cloud->mutex );
        L3::copy( plot_cloud.get(), tmp.get() ); 
        point_cloud_lock.unlock();

        boost::tuple<double, double, double> lower_left = min( tmp.get() );
        boost::tuple<double, double, double> upper_right = max( tmp.get() );

        glv::draw::blendTrans();

        bound_vertices[0]( lower_left.get<0>(), lower_left.get<1>(), -3.0 );
        bound_colors[0].set( 0, 1, 1, .25 );
        bound_vertices[1]( lower_left.get<0>(), upper_right.get<1>(), -3.0 );
        bound_colors[1].set( 0, 1, 1, .25 );
        bound_vertices[2]( upper_right.get<0>(), upper_right.get<1>(), -3.0 );
        bound_colors[2].set( 0, 1, 1, .25 );
        bound_vertices[3]( upper_right.get<0>(), lower_left.get<1>(), -3.0 );
        bound_colors[3].set( 0, 1, 1, .25 );

        glv::draw::lineStippling(true);
        glv::draw::lineStipple(4, 0xAAAA );
        glv::draw::lineWidth(.1);
        glv::draw::paint( glv::draw::LineLoop, bound_vertices, bound_colors, 4 );
        glv::draw::lineStippling(false);

        glv::draw::enable( glv::draw::Blend );
        glv::draw::paint( glv::draw::TriangleFan, bound_vertices, bound_colors, 4 );
        glv::draw::disable( glv::draw::Blend );

    }

    /*
     *  Pose prediction
     */

    void PoseEstimatesRenderer::onDraw3D( glv::GLV& g )
    {

        if( boost::shared_ptr< L3::Estimator::PoseEstimates > pose_estimates = estimates.lock() )
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
    }

    /*
     *  Single pose orientation renderer
     */

    void DedicatedPoseRenderer::update()
    {
        boost::shared_ptr< L3::PoseProvider > ptr = provider.lock();
        if ( !ptr )
            return;

        *pose = ptr->operator()();
    }

    void DedicatedPoseRenderer::onDraw3D(glv::GLV& g)
    {
        L3::SE3 tmp( *pose );

        // Centre
        tmp.X( 0 );
        tmp.Y( 0 );
        tmp.Z( 0 );

        glv::draw::translateZ( -25 );
        glv::draw::translateY( 1 );
        glv::draw::rotateX( 245 );
        glv::draw::rotateZ( 15 );

        CoordinateSystem( tmp ).onDraw3D( g );

        axes.onDraw3D( g );
    }

    /*
     *  2D Scan renderer
     */

    void ScanRenderer2D::onDraw3D( glv::GLV& g )
    {
        int draw_counter = 0;

        glv::Point3 points[541+2];
        glv::Color  fan[541+2];
        glv::Color  perimeter[541+2];

        points[draw_counter]( 0, 0, 0);
        perimeter[draw_counter].set( color, .75 );
        fan[draw_counter].set( color, .5 );

        draw_counter++;

        // Draw the front
        for (int scan_counter=0; scan_counter<541; scan_counter++) 
        {
            double range = scan->ranges[scan_counter];  

            if ( range < range_threshold)
                continue;

            // Compute angle 
            double angle = scan_counter*scan->angle_spacing +  scan->angle_start; 

            double x = range*cos( angle );
            double y = range*sin( angle );

            points[draw_counter]( x, y, 0 );
            perimeter[draw_counter].set( color, .75 );
            fan[draw_counter].set( color, 1 );

            draw_counter++;

        }

        glv::draw::translateZ( -55 );
        glv::draw::rotateZ( rotate_z );

        glv::draw::blendTrans();
        glv::draw::lineWidth(1);
        glv::draw::paint( glv::draw::LineLoop, points, perimeter, draw_counter );

        glEnable(GL_POLYGON_STIPPLE);
        glPolygonStipple(L3::Visualisers::mask);
        glv::draw::enable( glv::draw::Blend );
        glv::draw::paint( glv::draw::TriangleFan, points, fan, draw_counter );
        glv::draw::disable( glv::draw::Blend );
        glDisable(GL_POLYGON_STIPPLE);

    }

    void ScanRenderer2D::update()
    {
        boost::shared_ptr< L3::ConstantTimeIterator< L3::LMS151 > >  scan_ptr = windower.lock();

        if ( !scan_ptr || scan_ptr->window.empty() )
            return;

        scan = scan_ptr->window.back().second;

    }

    /*
     *  Cost Renderer
     */

    void CostRenderer::onDraw3D( glv::GLV& g )
    {
        glv::Point3 vertices[ estimates.costs.size() ];
        glv::Color colors[ estimates.costs.size() ];

        glv::Color current_color;

        double cost;
        int counter = 0;

        for( L3::Estimator::PoseEstimates::ESTIMATES_ITERATOR it = estimates.estimates.begin();
                it != estimates.estimates.end();
                it++ )
        {

            cost = std::isinf( estimates.costs[counter] ) ? 0.0 : estimates.costs[counter] ;
            vertices[counter]( it->X(), it->Y(), cost );
            current_color = glv::Color( cost/40.0, cost/40.0, cost/40.0);
            colors[counter] = current_color; 

            counter++;
        }


        glv::draw::pointSize( 2 );
        glv::draw::paint( glv::draw::Points, vertices, colors, counter );
    }

    /*
     *  Cost renderer (View)
     */

    void CostRendererView::onDraw3D( glv::GLV& g )
    {

        //float x_offset = (float)( rand()%100 );
        //float y_offset = (float)( rand()%100 );

        //glv::draw::translate( -1*x_offset, -1*y_offset, -75 );

        //glv::Point3 vertices[100];
        //glv::Color  colors[100];

        //for( int i=0; i<100; i++ )
        //vertices[i]( ((rand()%50)-25)+x_offset, ((rand()%50)-25)+y_offset, 0 );

        //glv::draw::paint( glv::draw::Points, vertices, colors, 100 );

        glv::draw::translate( -1*estimates.position->X(), -1*estimates.position->Y(), -30);

        //L3::ReadLock lock( estimates.mutex );
        CostRenderer::onDraw3D( g );     

    }

    /*
     *  Locale specific
     */
    void LocaleBoundsRenderer::onDraw3D( glv::GLV& g )
    {
        glv::Point3 outer_vertices[4];
        glv::Color  outer_colors[4];

        outer_vertices[0]( -500.0, -500.0, 10 );
        outer_colors[0].set( 0, 1, 0, .4); 

        outer_vertices[1]( -500.0, 500.0, 10 );
        outer_colors[1].set( 0, 1, 0, .4); 

        outer_vertices[2]( 500.0, 500.0, 10 );
        outer_colors[2].set( 0, 1, 0, .4); 

        outer_vertices[3]( 500.0, -500.0, 10 );
        outer_colors[3].set( 0, 1, 0, .4); 

        glv::draw::enable( glv::draw::Blend );

        glv::draw::lineStippling(true);
        glv::draw::lineStipple(4, 0xAAAA );
        glv::draw::lineWidth( 1 );
        glv::draw::lineStippling(false);

        glv::draw::paint( glv::draw::LineLoop, outer_vertices, outer_colors, 4 );

        glv::draw::disable( glv::draw::Blend );

    }

    void HistogramPyramidRendererView::update()
    {
        for( std::deque< boost::shared_ptr< HistogramDensityRenderer > >::iterator it=renderers.begin();
                it != renderers.end();
                it++ )
            (*it)->update();
    }


    /*
     *  Algorithm rendering
     */
    void AlgorithmCostRendererLeaf::onDraw3D( glv::GLV& g )
    {
        // Does it exist?
        boost::shared_ptr< L3::Estimator::Algorithm<double> > algorithm_ptr = algorithm.lock();

        if( !algorithm_ptr)
        {
            std::cerr << "No associated algorithm" << std::endl;
            return;
        }

        // Can we handle it?
        boost::shared_ptr< L3::Estimator::IterativeDescent<double> > discrete_algorithm_ptr = boost::dynamic_pointer_cast< L3::Estimator::IterativeDescent<double> >( algorithm_ptr );

        if( !discrete_algorithm_ptr )
            return;

        double layer_height = 20;

        double LOWER = std::numeric_limits<double>::infinity();
        double UPPER = -1*LOWER;

        double min_x=LOWER, min_y=LOWER, min_z=LOWER;
        double max_x=UPPER, max_y=UPPER, max_z=UPPER;

        for( std::deque< boost::shared_ptr< L3::Estimator::DiscreteEstimator<double> > >::iterator it = discrete_algorithm_ptr->discrete_estimators.begin();
                it != discrete_algorithm_ptr->discrete_estimators.end(); 
                it++ )
        {
            std::vector< L3::SE3 > current_estimates;
            std::vector< double >  current_costs;

            current_estimates.assign( (*it)->pose_estimates->estimates.begin(), (*it)->pose_estimates->estimates.end() );
            current_costs.assign(  (*it)->pose_estimates->costs.begin(), (*it)->pose_estimates->costs.end() );

            glv::Point3 vertices[ current_estimates.size() ];
            glv::Color  colors[ current_estimates.size() ];

            int counter= 0;
            for( std::vector< L3::SE3 >::iterator it=current_estimates.begin(); 
                    it != current_estimates.end();
                    it++ ) 
            {
                float plot_height = 0;

                glv::Color plot_color( 1, 0, 0 ); 
                if( !std::isinf( current_costs[counter] ) )
                {
                    plot_height = current_costs[counter];
                    plot_color = glv::Color( 0, 1, 0 ); 
                }

                vertices[counter]( it->X(), it->Y(), layer_height+(plot_height) );
                colors[counter] =  plot_color;           
                counter++;

                min_x = std::min( min_x, it->X() );
                min_y = std::min( min_y, it->Y() );
                min_z = std::min( min_z, layer_height+plot_height );

                max_x = std::max( max_x, it->X() );
                max_y = std::max( max_y, it->Y() );
                max_z = std::max( max_z, layer_height+plot_height );

            }

            layer_height += 20;

            glv::draw::paint( glv::draw::Points, vertices, colors, counter );
        }

        this->lower.x = min_x;
        this->lower.y = min_y;
        this->lower.z = min_z;
        this->upper.x = max_x;
        this->upper.y = max_y;
        this->upper.z = max_z;
    }

    /*
     *  Scan-matching trajectory renderer
     */

    void ScanMatchingTrajectoryRenderer::onDraw3D( glv::GLV& g )
    {
        boost::shared_ptr< L3::ScanMatching::Engine > engine_ptr = engine.lock();
            
        std::deque< std::pair< double, Eigen::Matrix4f > > trajectory;
        
        if( engine_ptr )
        {
            L3::ReadLock lock( engine_ptr->mutex );
            trajectory.assign( engine_ptr->trajectory.begin(), engine_ptr->trajectory.end() );
            lock.unlock();
        }
        
        L3::SE3 zero;
        CoordinateSystem( zero ).onDraw3D(g);
        
        if( enabled( glv::Maximized ) )
            Composite::onDraw3D(g);
        else
        {
            glv::draw::translateZ( -45 );

            if( !trajectory.empty() )
            {
                std::pair< double, Eigen::Matrix4f > start_pose = trajectory.back();

                glv::draw::translate( -1*start_pose.second(0,3), -1*start_pose.second(1,3), 0.0 );
            }
        }

        for( std::deque< std::pair< double, Eigen::Matrix4f > >::iterator it = trajectory.begin();
                it != trajectory.end();
                it++ )
        {
            L3::SE3 pose;
            pose.setHomogeneous( it->second );
            CoordinateSystem( pose, 10 ).onDraw3D(g);
        }

    }

    void ScanMatchingScanRenderer::onDraw3D( glv::GLV& g )
    {
        // Are we still valid
        boost::shared_ptr< L3::ScanMatching::Engine > ptr = engine.lock();

        if( !ptr )
            return;

        boost::scoped_array<double> scan;
        boost::scoped_array<double> putative;

        // Lock
        L3::ReadLock lock( ptr->mutex );

        int scan_points = ptr->matcher->scan_points;
        int putative_points = ptr->matcher->putative_points;

        if( (scan_points <= 0 || scan_points > 541 ) || (putative_points <= 0  || putative_points > 541 ) )
            return;

        scan.reset( new double[scan_points*3] );
        std::copy( ptr->matcher->scan.get(), 
                ptr->matcher->scan.get()+ scan_points*3, 
                scan.get() );

        putative.reset( new double[putative_points*3] );
        std::copy( ptr->matcher->putative.get(), 
                ptr->matcher->putative.get()+putative_points*3,
                putative.get() );

        lock.unlock();

        glv::Point3 scan_vertices[scan_points];
        glv::Color  scan_colors[scan_points];

        double* iterator = &scan[0];

        float x,y,z;
        for( int i=0; i<scan_points; i++ )
        {
            x = *iterator++;
            y = *iterator++;
            z = *iterator++;

            scan_vertices[i]( x, y, 0 );
            scan_colors[i].set( .5, .5, .5, .5 ); 
        }

        glv::draw::translateZ( -80 );

        glv::Point3 putative_vertices[scan_points];
        glv::Color  putative_colors[scan_points];

        iterator = &putative[0];

        for( int i=0; i<putative_points; i++ )
        {
            x = *iterator++;
            y = *iterator++;
            z = *iterator++;

            putative_vertices[i]( x, y, 0 );
            putative_colors[i].set( 1, 0, 0, .5 ); 
        }

        if( enabled( glv::Maximized ) )
            glv::draw::pointSize(4);
        else
            glv::draw::pointSize(1);

        glv::draw::enable( glv::draw::Blend );
        glv::draw::paint( glv::draw::Points, scan_vertices, scan_colors, scan_points );
        glv::draw::paint( glv::draw::Points, putative_vertices, putative_colors, putative_points );
        glv::draw::disable( glv::draw::Blend );

        // Are we maximised?
        if (enabled( glv::Maximized ))
        {
            trajectory->enable( glv::Visible );
            boost::dynamic_pointer_cast< ScanMatchingTrajectoryRenderer >(trajectory)->engine = engine;
        }
        else
            trajectory->disable( glv::Visible );

    }

    /*
     *  Pose sequences
     */
    void PoseSequenceRenderer::onDraw3D( glv::GLV& g )
    {
        std::stringstream ss;
        if ( pose_sequence)
        {
            std::vector< std::pair< double, boost::shared_ptr< L3::SE3 > > >::iterator it = pose_sequence->begin();
            ss << it->first;

            glv::draw::lineWidth( 1 );

            std::list< int > window;

            int counter = 0;

            // Draw transparent
            glv::draw::enable( glv::draw::Blend );
            while( it < pose_sequence->end() )
            {
                float alpha = 0.3;

                int diff = (counter - highlighted_position );
                if ( diff > -10  && diff  < 10 ) 
                {
                    alpha = 1.0;
                    CoordinateSystem( *(it->second), 10-abs(diff), 1 ).onDraw3D(g);
                }
                else
                    CoordinateSystem( *(it->second), 5, .3 ).onDraw3D(g);


                it+=skip;
                counter++;
            }
            glv::draw::disable( glv::draw::Blend );

            highlighted_position++;

            if( highlighted_position > pose_sequence->size()/skip )
                highlighted_position=0;
        }

    }

    Text3D::Text3D() : scale(4)
    {
        //glfInit();

        //font_descriptor = glfLoadFont( (char*)"/Users/ian/code/thirdparty/glf_distr/fonts/courier1.glf" );

        //glfSetCurrentFont(font_descriptor);
    }

    void Text3D::setText( std::string text )
    {
        this->text = text;
    }

    void Text3D::onDraw3D( glv::GLV& g )
    {
        //float currentColor[4];
        //glGetFloatv(GL_CURRENT_COLOR,currentColor);

        //glColor3f(1.0,0.2,0.3);
        //glEnable(GL_BLEND);
        //glColor4f( .5, .5, .5, .85 );
        //glPushMatrix();
        //glTranslatef( scale/2, scale/2, 0 );
        //glScalef(scale,scale,1.0);
        //glfGetStringBounds( const_cast<char*>(text.c_str()), &xmin, &ymin, &xmax, &ymax);
        //glfDrawSolidString( const_cast<char*>(text.c_str() ) );
        //glDisable(GL_BLEND);
        //glPopMatrix();

        //glColor4f( currentColor[0], currentColor[1], currentColor[2], currentColor[3] );

    }

    /*
     *Label renderer
     */

    void LeafLabel::onDraw3D( glv::GLV& g )
    {
        if ( std::isnan( tag->x ) || std::isnan( tag->y ))
            return;

        //glPushMatrix();
        glv::draw::push();
        glRotatef( 90, 1, 0, 0 );
        glTranslatef( tag->x, 20, -1*tag->y);
        Text3D::setText( tag->text );
        Text3D::onDraw3D( g );
        //glPopMatrix();
        glv::draw::pop();

        glv::Point3 vertices[4];
        glv::Color  colors[4];
        glv::Color  sheen[4];

        std::fill( colors, colors+4, glv::Color( .3, .3, .3, .8 ) );
        std::fill( sheen, sheen+4, glv::Color( 1, 0, 0, .1 ) );

        // Scale factor
        float width = Text3D::scale*( Text3D::xmax - Text3D::xmin );
        float height = Text3D::scale*( Text3D::ymax - Text3D::ymin );

        float margin = 2.0;

        vertices[0]( tag->x-margin, tag->y, 20-margin );
        vertices[1]( tag->x-margin, tag->y, 20+height+margin );
        vertices[2]( tag->x +width +margin, tag->y, 20+height+margin  );
        vertices[3]( tag->x +width +margin, tag->y, 20-margin );


        glv::draw::blendTrans();
        glv::draw::enable( glv::draw::Blend );
        glv::draw::paint( glv::draw::LineLoop, vertices, colors, 4 );
        glv::draw::paint( glv::draw::TriangleFan, vertices, sheen, 4 );
        glv::draw::disable( glv::draw::Blend );

        // Draw a bounding box, transparency, etc.
    }

    /*
     *  Experience location overview
     */

    void ExperienceOverviewView::onDraw3D(glv::GLV& g)
    {
        far(1000); 

        glv::draw::translateZ( -850 );

        boost::shared_ptr< L3::PoseProvider > provider_ptr = provider.lock();
        if( provider_ptr )
        {
            *current = provider_ptr->operator()();
            glv::draw::translate( -1*current->X(), -1*current->Y() );
            animation->onDraw3D(g);
        }

        boost::shared_ptr< L3::Experience > ptr = experience.lock();

        if( !ptr )
            return;

        experience_nodes_vertices.reset( new glv::Point3[ptr->sections.size()] );
        experience_nodes_colors.reset( new glv::Color[ptr->sections.size()] );

        std::deque<L3::experience_section>::iterator it = ptr->sections.begin();

        GLfloat ctrlpoints[ptr->sections.size()][3];
        
        int counter =0;

        ColorInterpolator interpolator;

        glv::draw::pointSize( 3 );
        while( it != ptr->sections.end() )
        {
            experience_nodes_vertices[ counter ]( it->x, it->y, 0 );
            experience_nodes_colors[ counter++ ].set( interpolator( double(counter)/ptr->sections.size() ) ); 

            ctrlpoints[counter][0]= it->x;
            ctrlpoints[counter][1]= it->y;
            ctrlpoints[counter][2]= 0.0;

            it++;
        }

        float point_size = enabled( glv::Maximized ) ? 5 : 1;

        glv::draw::pointSize( point_size );
        glv::draw::paint( glv::draw::Points, experience_nodes_vertices.get(), experience_nodes_colors.get(), ptr->sections.size());
        glv::draw::lineWidth( .05 );
        std::fill( experience_nodes_colors.get(), experience_nodes_colors.get()+ptr->sections.size(), glv::Color( .7, .7, .7 ) );
        glv::draw::paint( glv::draw::LineLoop, experience_nodes_vertices.get(), experience_nodes_colors.get(), ptr->sections.size());

        static int draw_counter = 0;
        glv::draw::text( "Is this the real world", draw_counter++, 40, 20 );

        if( enabled( glv::Maximized ) )
        {
            boost::dynamic_pointer_cast < ExperienceCloudView >(experience_point_cloud)->experience = experience;
            experience_point_cloud->enable( glv::Visible );
        }
        else
            experience_point_cloud->disable( glv::Visible );
    }

    /*
     *  Experience point viewer
     */
    void ExperienceCloudView::onDraw3D( glv::GLV& g)
    {
        far( 1000 );
        
        L3::SE3 current;
        if( boost::shared_ptr< L3::PoseProvider > provider_ptr = provider.lock() )
            current = provider_ptr->operator()();


        current.Z( current.Z() + 3.0);
        transformCameraToPose( current );
               
        glv::draw::enable( glv::draw::Blend );
        glv::draw::paint( glv::draw::Points, &vertices[0], &colors[0], vertices.size() );
        glv::draw::disable( glv::draw::Blend );

        //point_cloud_renderer_leaf->onDraw3D( g );
        //for( std::deque< Cube >::iterator it = voxels.begin();
                //it != voxels.end();
                //it++ )
            //renderCube( &*it ); 

        CoordinateSystem( current ).onDraw3D(g);
    
    }

    void ExperienceCloudView::load(  boost::shared_ptr< L3::Experience > experience )
    {
        int section_counter = 0;

        while( true )
        {
            try
            {
                // Load the experience
                std::pair< long unsigned int, L3::Point<double>* > load_result = experience->load( section_counter );
           
                boost::shared_ptr< L3::PointCloud<double> > cloud = boost::make_shared<L3::PointCloud<double> >();

                cloud->num_points = load_result.first;
                cloud->points = load_result.second;

                clouds.push_back( cloud );
            }
            catch( ... )
            {
                break;
            }

            section_counter++;
        }

        join( clouds, master );

        point_cloud_renderer_leaf = boost::make_shared< PointCloudRendererLeaf >( master );

        cloud.reset (new pcl::PointCloud<pcl::PointXYZ>);
  
        cloud->height = 1;
        cloud->width = master->num_points;
        cloud->points.resize (cloud->width * cloud->height);

        for (size_t i = 0; i < cloud->points.size (); ++i)
        {
            cloud->points[i].x = master->points[i].x; 
            cloud->points[i].y = master->points[i].y; 
            cloud->points[i].z = master->points[i].z; 
        }

        octree->setInputCloud (cloud);
        octree->addPointsFromInputCloud ();
    
        pcl::octree::OctreePointCloudDensity<pcl::PointXYZ>::Iterator tree_iterator;
        pcl::octree::OctreePointCloudDensity<pcl::PointXYZ>::Iterator tree_iterator_end = octree->end();

        // Traverse the whole depth
        int depth = octree->getTreeDepth();

        pcl::PointXYZ pt;

        for (tree_iterator = octree->begin(depth); tree_iterator!=tree_iterator_end; ++tree_iterator)
        {
            Eigen::Vector3f voxel_min, voxel_max;
            octree->getVoxelBounds(tree_iterator, voxel_min, voxel_max);

            pt.x = (voxel_min.x() + voxel_max.x()) / 2.0f;
            pt.y = (voxel_min.y() + voxel_max.y()) / 2.0f;
            pt.z = (voxel_min.z() + voxel_max.z()) / 2.0f;
       
            int density = octree->getVoxelDensityAtPoint( pt ); 
          
            //voxels.push_back( Cube( voxel_min.x(), voxel_min.y(), voxel_min.z(),
                                //voxel_max.x(), voxel_max.y(), voxel_max.z(), density ) );

            vertices.push_back( glv::Point3( pt.x, pt.y, pt.z ) );
            colors.push_back( glv::Color( .5, .5, .5, float(density)/10.0 ) );
        }
    }


    /*
     *  Dataset viewer
     */

    void DatasetOverviewView::onDraw3D( glv::GLV& g )
    {
        glv::Point3    vertices[ poses->size() ];
        glv::Color     colors[ poses->size() ];

        boost::shared_ptr< L3::PoseProvider > provider_ptr = provider.lock();

        if( provider_ptr )
        {
            L3::SE3 pose = provider_ptr->operator()();
            //*current_pose  =  provider_ptr->operator()();
            //glv::draw::translate( -1*current_pose->X(), -1*current_pose->Y(), 0.0 );
            //*current_pose  =  pose;
            glv::draw::translate( -1*pose.X(), -1*pose.Y(), 0.0 );

            pose.X( -1*pose.X() );
            pose.Y( -1*pose.Y() );

            L3::Visualisers::AnimatedPoseRenderer( pose ).onDraw3D(g);
        }

        int counter= 0;
        for( std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > >::iterator it = poses->begin();
                it != poses->end();
                it++ )
        {
            vertices[counter++]( it->second->X(), it->second->Y(), 0.0 );
        }

        far( 1500 );

        glv::draw::translate(0,0,-550);
        glv::draw::lineWidth(10.0);
        glv::draw::paint( glv::draw::Points, vertices, colors, counter );

        renderer->onDraw3D(g);
    }


    /*
     * Statistics
     */

    struct MemoryStatistics : glv::TextView
    {
        
        explicit MemoryStatistics() : glv::TextView( glv::Rect(150,25 ) )
        {
            this->disable( glv::DrawBorder );
        }
        
        vm_size_t page_size;
        mach_port_t mach_port;
        mach_msg_type_number_t count;
        vm_statistics_data_t vm_stats;

        std::pair< int64_t, int64_t > getSystemMemoryUsage()
        {
            mach_port = mach_host_self();
            count = sizeof(vm_stats) / sizeof(natural_t);
            if (KERN_SUCCESS == host_page_size(mach_port, &page_size) &&
                    KERN_SUCCESS == host_statistics(mach_port, HOST_VM_INFO, 
                        (host_info_t)&vm_stats, &count))
            {
                int64_t myFreeMemory = (int64_t)vm_stats.free_count * (int64_t)page_size;

                int64_t used_memory = ((int64_t)vm_stats.active_count + 
                        (int64_t)vm_stats.inactive_count + 
                        (int64_t)vm_stats.wire_count) *  (int64_t)page_size;


                return std::make_pair( used_memory, myFreeMemory );
            }
                
            return std::make_pair( 0, 0);
        }


        std::pair< int64_t, int64_t > getApplicationMemoryUsage()
        {
            struct task_basic_info t_info;
            mach_msg_type_number_t t_info_count = TASK_BASIC_INFO_COUNT;
            
            if (KERN_SUCCESS != task_info(mach_task_self(),
                        TASK_BASIC_INFO, (task_info_t)&t_info, 
                        &t_info_count))
            {
                return std::make_pair( 0, 0 ) ;
            }
            else
                 return std::make_pair( t_info.resident_size, t_info.virtual_size ) ;

        }


        void onDraw(glv::GLV& g)
        {
            std::stringstream ss;
            ss.precision( 8 );
     
            std::pair< int64_t, int64_t > memory_stats = getApplicationMemoryUsage();

            ss << memory_stats.first/(1024) << " kB (" << memory_stats.first/(1024*1024) << " MB)" << std::endl;
            //ss << memory_stats.second/(1024) << " kB (" << memory_stats.second/(1024*1024) << " MB)" << std::endl;

            mText = ss.str();

            glv::TextView::onDraw(g);
           
        }

    };


    Statistics::Statistics() : glv::Table( "<,", 0, 0 )
    {
        observer_update.reset(      new TextRenderer<double>() );
        swathe_generation.reset(    new TextRenderer<double>() );
        points_per_second.reset(    new TextRenderer<double>() );
        estimation.reset(           new TextRenderer<double>() );
        memory_statistics.reset(    new MemoryStatistics() );

        /*
         *DBG
         */
        //observer_update->enable( glv::DrawBorder );
        //swathe_generation->enable( glv::DrawBorder );

        observer_update->paddingY( 7 );
        swathe_generation->paddingY( 7 );
        points_per_second->paddingY( 7 );
        estimation->paddingY( 7 );
        memory_statistics->paddingY( 10 );

        (*this) << dynamic_cast< glv::View* >(observer_update.get()) <<
                    dynamic_cast< glv::View* >(swathe_generation.get()) <<
                    dynamic_cast< glv::View* >(points_per_second.get()) <<
                    dynamic_cast< glv::View* >(estimation.get()) <<
                    dynamic_cast< glv::View* >(memory_statistics.get());

        boost::shared_ptr< glv::Label > observer_label = boost::make_shared< glv::Label >( "Observer update" );
        labels.push_back( observer_label );
        observer_label->pos( glv::Place::CL, 0, 0 ).anchor( glv::Place::CR ); 
        (*observer_update) << (*observer_label );

        boost::shared_ptr< glv::Label > swathe_label = boost::make_shared< glv::Label >( "Swathe generation" );
        labels.push_back( swathe_label );
        swathe_label->pos( glv::Place::CL, 0, 0 ).anchor( glv::Place::CR ); 
        (*swathe_generation) << (*swathe_label );

        boost::shared_ptr< glv::Label > points_per_second_label = boost::make_shared< glv::Label >( "Point generation" );
        labels.push_back( points_per_second_label );
        points_per_second_label->pos( glv::Place::CL, 0, 0 ).anchor( glv::Place::CR ); 
        (*points_per_second) << (*points_per_second_label );

        boost::shared_ptr< glv::Label > estimation_label = boost::make_shared< glv::Label >( "Estimation" );
        labels.push_back( estimation_label );
        estimation_label->pos( glv::Place::CL, 0, 0 ).anchor( glv::Place::CR ); 
        (*estimation) << (*estimation_label );

        boost::shared_ptr< glv::Label > memory_label = boost::make_shared< glv::Label >( "Resource Usage" );
        labels.push_back( memory_label );
        memory_label->pos( glv::Place::CL, 0, 0 ).anchor( glv::Place::CR ); 
        (*memory_statistics) << (*memory_label );

        std::vector< std::string > history_labels;
        history_labels.push_back( "Observer (ms)" );
        history_labels.push_back( "Swathe (ms)" );
        history_labels.push_back( "Points (ms)");
        history_labels.push_back( "Estimation (ms)" );

        for( int i=0; i<4; i++ )
        {
            // Create the plottable
            boost::shared_ptr< StatisticsPlottable<double> > plottable( new StatisticsPlottable<double>() ); 
            // Create the plot
            boost::shared_ptr< glv::Plot > plot( new glv::Plot( glv::Rect( 540, 80), *plottable ) );
           
            // Label
            boost::shared_ptr< glv::Label > graph_label = boost::make_shared< glv::Label >( history_labels[i] );
            graph_label->pos( glv::Place::TR, 0, 0 ).anchor( glv::Place::TR ); 
            *plot  << *graph_label;
            labels.push_back( graph_label );

            // Disable control (dragging)
            plot->disable( glv::Controllable );
            plot->showNumbering(true);
            plot->numbering(false,0);
            plot->range( 0, 100, 0 );
            plot->range( 0, 1.1, 1 );
            
            (*this) << *plot;
           
            plottables.push_back( plottable );
            plots.push_back( plot );
                        
        }

        this->arrange();
    }

    void ParticleFilterRendererLeaf::onDraw3D( glv::GLV& g )
    {
        boost::shared_ptr< L3::Estimator::ParticleFilter<double> > filter_ptr = filter.lock();

        if( !filter_ptr )
            return;

        L3::ReadLock lock( filter_ptr->mutex );
        std::vector< L3::SE3 > hypotheses( filter_ptr->hypotheses.begin(), filter_ptr->hypotheses.end() );
        lock.unlock();
       
        for ( L3::Estimator::ParticleFilter<double>::PARTICLE_ITERATOR it = hypotheses.begin();  
                it != hypotheses.end();
                it++ )
            CoordinateSystem( *it ).onDraw3D(g);
    }


    /*
     *  Specific controllers
     */

    void ChaseController::onDraw3D( glv::GLV& g )
    {
        current.x = -1*chase.X();
        current.y = -1*chase.Y();
    }

}
}
