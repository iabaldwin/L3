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
    PoseChainRenderer( std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > >* poses ) ;

    std::deque< boost::shared_ptr<L3::Visualisers::CoordinateSystem> > coords;

    void onDraw3D(glv::GLV& g);
    
};

/*
 *Cloud Renderer
 */
template <typename T>
struct CloudRenderer : Leaf
{
    CloudRenderer( L3::PointCloud<T>* CLOUD );
    glv::Color*         colors;
    glv::Point3*        vertices;
    L3::PointCloud<T>*  cloud;
    
    ~CloudRenderer();

    void onDraw3D( glv::GLV& g );

    
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
    IteratorRenderer( L3::Iterator<T>* ITERATOR ) ;
    
    L3::Iterator<T>* iterator;
   
    void onDraw3D( glv::GLV& g );
};


/*
 * Swathe renderer
 *
 *  Render the poses from a swathe generator
 *
 */
struct SwatheRenderer : Leaf
{
    SwatheRenderer( L3::SwatheBuilder* SWATHE_BUILDER ); 
    
    unsigned int                                        current_alloc;
    L3::SwatheBuilder*                                  swathe_builder;
    std::auto_ptr< L3::Visualisers::HistogramRenderer > histogram_renderer;

    double                                              x,y;
    L3::PointCloud<double>*                             point_cloud;
    std::auto_ptr<L3::Projector<double> >               projector;
    
    glv::Color*         pose_colors;
    glv::Point3*        pose_vertices;
    glv::Color*         point_colors  ;
    glv::Point3*        point_vertices;

    L3::Tools::Timer    t;
    
    void realloc( int size );

    void onDraw3D( glv::GLV& g );

};

/*
 * Experience renderer 
 */
struct ExperienceRenderer : Leaf
{

    ExperienceRenderer( boost::shared_ptr<L3::Experience> EXPERIENCE );

    int                                 pt_limit, pt_counter, sample_counter; 
    boost::shared_ptr<L3::Experience>   experience;
    glv::Point3*                        point_vertices;
    glv::Color*                         point_colors;
    L3::PoseProvider*                   pose_provider;

    ~ExperienceRenderer();
    
    void addPoseProvider( L3::PoseProvider* provider );

    void onDraw3D( glv::GLV& g );

};

/*
 *Real or synthesized pose provider
 */
struct PoseProviderRenderer : Leaf
{
    PoseProviderRenderer( L3::PoseProvider* provider ) ;

    glv::Color*                             colors;
    glv::Point3*                            vertices;

    int                                     counter, history;
    L3::PoseProvider*                       pose_provider;
    std::vector< std::pair<double,double> > positions;

    void onDraw3D( glv::GLV& g );

};


}   // ::Visualisers
}   // ::L3

#endif
