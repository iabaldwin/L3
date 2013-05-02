#include "Components.h"

#include <boost/scoped_ptr.hpp>
#include <GL/freeglut.h>
#include <GL/freeglut_ext.h>

//const GLubyte mask[] =   {0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
          //0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
              //0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
                  //0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
                      //0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
                          //0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
                              //0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
                                  //0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01};


const GLubyte mask[] =   {0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55,
          0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55,
              0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55,
                  0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55,
                      0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55,
                          0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55,
                              0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55,
                                  0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55, 0xff, 0xff, 0xff, 0xff, 0x55, 0x55, 0x55, 0x55};

namespace L3
{
namespace Visualisers
{

    void Composite::onDraw3D( glv::GLV& g )
    {
        glv::draw::translate( position.x, position.y, position.z );
        glv::draw::rotate( position.r, position.p, position.q );
        
        std::list<Leaf*>::iterator leaf_iterator = components.begin();
        while( leaf_iterator != components.end() )
        {
            (*leaf_iterator)->onDraw3D( g );
            leaf_iterator++;
        }

    }

    /*
     *  Components :: Grid
     */
    Grid::Grid( float lower, float upper, float spacing) 
        : lower(lower), upper(upper), spacing(spacing)
    {
        vertices.reset( new glv::Point3[ 101*4 ] );
        colors.reset( new glv::Color[ 101*4 ] );

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
     *  Components :: DefaultAxes
     */
    DefaultAxes::DefaultAxes()
    {
        vertices.reset( new glv::Point3[ 6 ] );
        colors.reset( new glv::Color[ 6 ] );

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
     *  Components :: HistogramBoundsRenderer
     */
    void HistogramBoundsRenderer::onDraw3D(glv::GLV& g)
    {
        glv::Point3 bound_vertices[4];
        glv::Color  bound_colors[4];

        L3::ReadLock( hist->mutex );

        std::pair<float, float> lower_left = hist->coords(0,0);
        std::pair<float, float> upper_right = hist->coords( hist->x_bins, hist->y_bins );

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
     *  Components :: HistogramDensityRenderer
     */
    void HistogramDensityRenderer::update()
    {
        if( hist->empty() )
            return;

        L3::Histogram<double> tmp;
        L3::ReadLock lock( hist->mutex );
        L3::clone( hist.get(), &tmp );
        lock.unlock();   
        
        //data().resize( glv::Data::FLOAT, 1, tmp.x_bins, tmp.y_bins );
        //for( unsigned int i=0; i< tmp.x_bins; i++ )
        //{
            //for( unsigned int j=0; j< tmp.y_bins; j++ )
            //{
                //data().assign( tmp.bin(i,j)/10.0 , 0, i, j );
            //}
        //}
    
        mTex.magFilter(GL_NEAREST);
        mTex.dealloc();
        //mTex.alloc( tmp.x_bins, tmp.y_bins );
        mTex.alloc( tmp.y_bins, tmp.x_bins );

        double normalizer = tmp.max();

        unsigned char * pixs = mTex.buffer<unsigned char>();
            
        for(int i=0; i< tmp.x_bins; i++)
        {
        //for(int j=0; j<tmp.y_bins; j++ )
        for(int j=tmp.y_bins-1; j>=0; j-- )
            {
                //double data = (tmp.bin(i,j)/normalizer)*255;
                //double dta = tmp.bin(i,j)/normalizer;
                //double data = ((tmp.bin(j,i)/normalizer)*255);
                //double data = (tmp.bin(j,i)/normalizer);
                //double data = (tmp.bin(j,i)/normalizer)*255;
                //double data = (tmp.bin(i,j)/normalizer)*255;
             
                //double data = 0.0;
                double data = rand()%255;

                //std::cout << (int)(unsigned char)data << " ";
                
                //unsigned char* c = ((unsigned char*)&data );

                //std::cout << (int)(*c) << " ";

                *pixs++ = (unsigned char)(data);
                *pixs++ = (unsigned char)(data);
                *pixs++ = (unsigned char)(data);
                //*pixs++ = (unsigned char)255;
                *pixs++ = (unsigned char)data;

                //*pixs++ = *c;
                //*pixs++ = *c;
                //*pixs++ = *c;
                //*pixs++ = (unsigned char)255;

                //unsigned char* b = &((unsigned char*)(img->imageData + img->widthStep*j))[i*3];
                //*pixs++ = *b;
                //unsigned char* g = &((unsigned char*)(img->imageData + img->widthStep*j))[i*3+1];
                //*pixs++ = *g;
                //unsigned char* r = &((unsigned char*)(img->imageData + img->widthStep*j))[i*3+2];
                //*pixs++ = *r;

                //*pixs++ = (unsigned char)255;
            }
        
            //std::cout << std::endl;
        }

        //std::cout << "------------" << std::endl;

    }

    void HistogramDensityRenderer::onDraw( glv::GLV& g)
    {
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
        if ( plot_cloud->num_points > 0 )
        {
            for( int i=0; i<plot_cloud->num_points; i++) 
            {
                vertices[i]( plot_cloud->points[i].x, plot_cloud->points[i].y, plot_cloud->points[i].z); 
                //colors[i] = color; 
                // TODO: Have a color policy
                colors[i] = glv::Color( plot_cloud->points[i].z/10.0 );
            }

            glv::draw::paint( glv::draw::Points, vertices.get(), colors.get(), plot_cloud->num_points);

        }
    }
    
    /*
     *  Components :: Point cloud renderer (leaf)
     */
    void PointCloudRendererLeaf::onDraw3D( glv::GLV& g )
    {
        L3::ReadLock lock( cloud->mutex );
        L3::sample( cloud.get(), plot_cloud.get(), plot_cloud->num_points, false );
        lock.unlock();
       
        PointCloudRenderer::onDraw3D(g);    
    }
    
    /*
     *  Components :: Point cloud renderer (view)
     */

    void PointCloudRendererView::onDraw3D( glv::GLV& g )
    {
        far(500);
        glv::draw::translateZ(-250 );

        PointCloudRenderer::onDraw3D(g);    
    }

    void PointCloudRendererView::update()
    {
        L3::ReadLock lock( cloud->mutex );
        L3::sample( cloud.get(), plot_cloud.get(), plot_cloud->num_points, false );
        lock.unlock();

        //This used to exist, because we had already projected the point cloud
        //L3::SE3 tmp( *current_estimate );
        //Eigen::Matrix4f h = tmp.getHomogeneous();
        //tmp.setHomogeneous( h.inverse() );
        //L3::transform( point_cloud.get(), &tmp );

    }

    /*
     *  Point cloud :: bounds renderer
     */
    void PointCloudBoundsRenderer::onDraw3D(glv::GLV& g)
    { 
        glv::Point3 bound_vertices[4];
        glv::Color  bound_colors[4];

        boost::scoped_ptr< L3::PointCloud<double> > tmp( new L3::PointCloud<double>() );

        L3::ReadLock point_cloud_lock( cloud->mutex );
        L3::copy( cloud.get(), tmp.get() ); 
        point_cloud_lock.unlock();

        std::pair<double, double> lower_left = min( tmp.get() );
        std::pair<double, double> upper_right = max( tmp.get() );

        glv::draw::blendTrans();

        bound_vertices[0]( lower_left.first, lower_left.second, -3.0 );
        bound_colors[0].set( 0, 1, 1, .25 );
        bound_vertices[1]( lower_left.first, upper_right.second, -3.0 );
        bound_colors[1].set( 0, 1, 1, .25 );
        bound_vertices[2]( upper_right.first, upper_right.second, -3.0 );
        bound_colors[2].set( 0, 1, 1, .25 );
        bound_vertices[3]( upper_right.first, lower_left.second, -3.0 );
        bound_colors[3].set( 0, 1, 1, .25 );

        glv::draw::lineStippling(true);
        glv::draw::lineStipple(4, 0xAAAA );
        glv::draw::lineWidth(.1);
        glv::draw::paint( glv::draw::LineLoop, bound_vertices, bound_colors, 4 );
        glv::draw::lineStippling(false);

        glv::draw::enable( glv::draw::Blend );
        //glv::draw::paint( glv::draw::Quads, bound_vertices, bound_colors, 4 );
        glv::draw::paint( glv::draw::TriangleFan, bound_vertices, bound_colors, 4 );
        glv::draw::disable( glv::draw::Blend );

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

        //glv::draw::push3D( -1.0, 1.0, 20.0,  150.0, 35 );
        //glv::draw::push( -1.0, 1.0, 20.0,  150.0, 35 );

        glv::draw::translateZ( -25 );
        glv::draw::translateY( 1 );
        //glv::draw::rotate( 135 , 0 , 45 );
        //glv::draw::rotate( 45 , 0 , 45 );
        //glv::draw::rotate( 0 , 0 , 55 );

        glv::draw::rotateX( 245 );
        glv::draw::rotateZ( 15 );


        CoordinateSystem( tmp ).onDraw3D( g );

        axes.onDraw3D( g );
        const char* test = "TEST";
        //glutStrokeString( GLUT_BITMAP_HELVETICA_18, reinterpret_cast< const unsigned char*>( test ) );
        glutBitmapString( GLUT_BITMAP_HELVETICA_18, reinterpret_cast< const unsigned char*>( test ) );

        //glv::draw::pop3D();
        glv::draw::pop();
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

        //points[draw_counter]( 0, 0, 0);
        //perimeter[draw_counter].set( color, .75 );
        //fan[draw_counter].set( color, .5 );

        //draw_counter++;

        // Draw the front
        for (int scan_counter=0; scan_counter<541; scan_counter++) 
        {
            double range = scan->ranges[scan_counter];  

            if ( range < 5 )
                continue;

            draw_counter++;

            // Compute angle 
            double angle = scan_counter*scan->angle_spacing +  scan->angle_start; 

            double x = range*cos( angle );
            double y = range*sin( angle );

            points[draw_counter]( x, y, 0 );
            perimeter[draw_counter].set( color, .75 );
            fan[draw_counter].set( color, 1 );

        }

        //points[draw_counter]( 0, 0, 0);
        //perimeter[draw_counter].set( color, .75 );
        //fan[draw_counter].set( color, .5 );

        glv::draw::translateZ( -55 );
        glv::draw::rotateZ( rotate_z );

        glv::draw::blendTrans();
        //glv::draw::lineWidth(1);
        //glv::draw::paint( glv::draw::LineLoop, points, perimeter, draw_counter );

           
        glEnable(GL_POLYGON_STIPPLE);
        glPolygonStipple(mask);
        glv::draw::enable( glv::draw::Blend );
        glv::draw::paint( glv::draw::TriangleStrip, points, fan, draw_counter );
        glv::draw::disable( glv::draw::Blend );
        glDisable(GL_POLYGON_STIPPLE);

    }

    void ScanRenderer2D::update()
    {
        windower->getWindow( window );

        if (window.size() > 0)
            scan = window.back().second;
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
        //Grid( -10, 10, 1 ).onDraw3D( g );
        Grid().onDraw3D( g );
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
        
        L3::ReadLock lock( estimates.mutex );
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
        for( std::list< boost::shared_ptr< HistogramDensityRenderer > >::iterator it=renderers.begin();
                it != renderers.end();
                it++ )
        {
            (*it)->update();
        }
    }


}
}
