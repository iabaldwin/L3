#ifndef L3_VISUAL_COMPONENTS_H
#define L3_VISUAL_COMPONENTS_H

#include <iostream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"
#include "Components.h"

namespace L3
{
namespace Visualisers
{

/*
 *  Pose chain renderer
 *
 *      Render a static chain of poses
 *
 */
struct PoseChainRenderer : Leaf
{
    std::deque< boost::shared_ptr<L3::Visualisers::CoordinateSystem> > coords;

    PoseChainRenderer( std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > >* poses ) 
    {
        std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > >::iterator it;
        
        for( it=poses->begin(); it < poses->end(); it+= 100 )
        {
            coords.push_back( boost::shared_ptr<L3::Visualisers::CoordinateSystem>( new L3::Visualisers::CoordinateSystem( (*it->second) ) ) );
        }
    }

    void onDraw3D(glv::GLV& g)
    { 
        for ( std::deque< boost::shared_ptr<L3::Visualisers::CoordinateSystem> >::iterator it = coords.begin(); it!= coords.end(); it++ )
            (*it)->onDraw3D( g );
    }
};

/*
 *Cloud Renderer
 */
template <typename T>
struct CloudRenderer : Leaf
{
    CloudRenderer( L3::PointCloud<T>* CLOUD ) : cloud(CLOUD)
    {
        colors   = new glv::Color[cloud->num_points];
        vertices = new glv::Point3[cloud->num_points];
   
        for( int i=0; i<cloud->num_points; i++) 
        {
            vertices[i]( cloud->points[i].x, cloud->points[i].y, cloud->points[i].z); 
            colors[i] = glv::HSV(0.6, .1, cloud->points[i].z*.5);
        }
    
    };

    ~CloudRenderer()
    {
        delete [] colors;
        delete [] vertices;
    }

    void onDraw3D( glv::GLV& g )
    {
        glv::draw::paint( glv::draw::Points, &*vertices, &*colors, cloud->num_points);
    }
    
    glv::Color*         colors;
    glv::Point3*        vertices;
    L3::PointCloud<T>*  cloud;

};

/*
 * Iterator renderer
 *
 *  Render the poses from a dataset iterator
 *
 */
template <typename T>
struct IteratorRenderer : Leaf
{

    IteratorRenderer( L3::Iterator<T>* ITERATOR )  : iterator(ITERATOR )
    {
    }
    
    void onDraw3D( glv::GLV& g )
    {
        // Update the iterator
        iterator->update( time );

        // Reserve
        glv::Color* colors    = new glv::Color[iterator->window.size()];
        glv::Point3* vertices = new glv::Point3[iterator->window.size()];;

        typename L3::Iterator<T>::WINDOW_ITERATOR it = iterator->window.begin();

        int counter = 0;

        while( it != iterator->window.end() )
        {
            vertices[counter]( it->second->x, it->second->y, 0 );
            counter++;
            it++; 
        }
    
        glv::draw::paint( glv::draw::Points, vertices, colors, counter );

        delete [] colors;
        delete [] vertices;
    }

    L3::Iterator<T>* iterator;

};


/*
 * Swathe renderer
 *
 *  Render the poses from a swathe generator
 *
 */
struct SwatheRenderer : Leaf
{
    SwatheRenderer( L3::SwatheBuilder* SWATHE_BUILDER )  : swathe_builder(SWATHE_BUILDER), current_alloc(500)
    {
        // Projector  
        L3::SE3 calibration( 0, 0, 0, -1.57, 0, 0 ); 
        point_cloud = new L3::PointCloud<double>();
        projector.reset( new L3::Projector<double>( &calibration, point_cloud ) );
    
        pose_colors    = new glv::Color[current_alloc];
        pose_vertices  = new glv::Point3[current_alloc];
        point_colors   = new glv::Color[10*100000];
        point_vertices = new glv::Point3[10*100000];
  
        histogram_renderer.reset( new L3::Visualisers::HistogramRenderer() ); 
    }

    std::auto_ptr< L3::Visualisers::HistogramRenderer > histogram_renderer;

    L3::SwatheBuilder* swathe_builder;
    unsigned int current_alloc;

    void realloc( int size )
    {
        delete [] pose_colors;
        delete [] pose_vertices;

        pose_colors   = new glv::Color[size];
        pose_vertices = new glv::Point3[size];

        current_alloc = size;
    }

    double x,y;
    L3::PointCloud<double>* point_cloud;
    std::auto_ptr<L3::Projector<double> > projector;
    
    glv::Color* pose_colors;
    glv::Point3* pose_vertices;
    glv::Color*  point_colors  ;
    glv::Point3* point_vertices;

    L3::Tools::Timer t;
    
    void onDraw3D( glv::GLV& g )
    {
        // Update the swathe_builder
        if ( !swathe_builder->update( time ))
            throw std::exception();

        // Do projection
        projector->project( swathe_builder->swathe );
    
        // Do histogram
        SWATHE_ITERATOR pose_iterator = swathe_builder->swathe.begin();

        // Get bounds
        std::pair<double,double> min_bound = L3::min<double>( point_cloud );
        std::pair<double,double> max_bound = L3::max<double>( point_cloud );
        std::pair<double,double> means     = L3::mean( point_cloud );

        // Build histogram 
        L3::histogram<double> hist( means.first, 
                                    means.first - min_bound.first, 
                                    max_bound.first - means.first, 
                                    means.second, 
                                    means.second - min_bound.second, 
                                    max_bound.second - means.second, 
                                    40 );
        hist( point_cloud );
        (*histogram_renderer)( &hist ) ;
        histogram_renderer->onDraw3D( g );

        if (swathe_builder->swathe.size() > current_alloc )
            realloc( swathe_builder->swathe.size() );

        int counter = 0;
        while( pose_iterator != swathe_builder->swathe.end() )
        {
            pose_vertices[counter]( pose_iterator->first->x, pose_iterator->first->y, 0 );
            pose_iterator++; 
            counter++;
        }
        glv::draw::paint( glv::draw::Points, pose_vertices, pose_colors, counter );
        
        PointCloud<double>::ITERATOR point_iterator = point_cloud->begin();

        counter = 0;

        //while( point_iterator != point_cloud->end() )
        while( point_iterator < point_cloud->end() )
        {
            point_vertices[counter++]( point_iterator->x , point_iterator->y , point_iterator->z);
            //point_iterator++; 
            point_iterator+=10; 
        }
        
        glv::draw::paint( glv::draw::Points, point_vertices, point_colors, counter );
    
    }

};

/*
 *Render experience point clouds
 */
struct ExperienceRenderer : Leaf
{

    ExperienceRenderer( boost::shared_ptr<L3::Experience> EXPERIENCE ) : experience(EXPERIENCE), pose_provider(NULL)
    {
        pt_limit = 1*10000;

        point_vertices = new glv::Point3[pt_limit];
        point_colors = new glv::Color[pt_limit];
   
    }

    boost::shared_ptr<L3::Experience> experience;
    int pt_limit, pt_counter, sample_counter; 
    glv::Point3* point_vertices;
    glv::Color*  point_colors;
    L3::PoseProvider* pose_provider;

    ~ExperienceRenderer()
    {
        delete [] point_vertices;
        delete [] point_colors;
    }

    
    void addPoseProvider( L3::PoseProvider* provider )
    {
        pose_provider = provider;
    }

    void onDraw3D( glv::GLV& g )
    {
        // Update experience
        if( pose_provider )
        {
            experience->update( random()%100, random()%100 );
        }

        boost::shared_ptr< L3::PointCloud<double> > cloud;
        experience->getExperienceCloud( cloud );

        for( pt_counter = 0, sample_counter = 0; (pt_counter < cloud->num_points) && ( sample_counter++< pt_limit ); pt_counter+=50 )
        {
            point_vertices[sample_counter]( cloud->points[pt_counter].x, cloud->points[pt_counter].y, cloud->points[pt_counter].z );
        }

        glv::draw::paint( glv::draw::Points, point_vertices, point_colors, sample_counter );

    }

};

struct PoseProviderRenderer : Leaf
{
    PoseProviderRenderer( L3::PoseProvider* provider ) : pose_provider(provider),
                                                        counter(0), 
                                                        history(20)
    {
        positions.resize( history );
   
        vertices = new glv::Point3[history];
        colors   = new glv::Color[history];
    }

        
    glv::Point3*    vertices;
    glv::Color*     colors;

    L3::PoseProvider* pose_provider;

    std::vector< std::pair<double,double> > positions;

    int counter, history;

    void onDraw3D( glv::GLV& g )
    {
        L3::SE3 pose = (*pose_provider)();

        positions[ counter++%(positions.size()) ] = std::make_pair( pose.x, pose.y );

        for ( int it = 0; it <history; it++ )
        {
            vertices[it]( positions[it].first, positions[it].second, 0.0 );
        }

        glv::draw::paint( glv::draw::Points, vertices, colors, positions.size() );

    }

};


}   // ::Visualisers
}   // ::L3

#endif
