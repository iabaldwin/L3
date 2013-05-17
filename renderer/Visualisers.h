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
 *  Iterator renderer
 */
template <typename T>
//struct IteratorRenderer : LabelledLeaf
struct IteratorRenderer : Leaf
{
    IteratorRenderer( boost::shared_ptr< L3::Iterator<T> > iterator ) : iterator(iterator )
    {
    }

    boost::weak_ptr< L3::Iterator<T> > iterator;
   
    void onDraw3D( glv::GLV& g );
};

/*
 *  Swathe renderer
 */
struct SwatheRenderer : Leaf
{
    SwatheRenderer( L3::SwatheBuilder* SWATHE_BUILDER ); 
    
    unsigned int        current_alloc;
    L3::SwatheBuilder*  swathe_builder;
        
    boost::shared_array< glv::Color >   pose_colors;
    boost::shared_array< glv::Point3 >  pose_vertices;
    boost::shared_array< glv::Color >   point_colors  ;
    boost::shared_array< glv::Point3 >  point_vertices;

    double                                      x,y;
    boost::shared_ptr<L3::PointCloud<double> >  point_cloud;
    std::auto_ptr<L3::Projector<double> >       projector;

    void realloc( int size );

    void onDraw3D( glv::GLV& g );

};

/*
 *  Experience renderer 
 */
struct ExperienceRenderer : Leaf
{
    ExperienceRenderer( boost::shared_ptr<L3::Experience> EXPERIENCE );

    int                                 pt_limit, pt_counter, sample_counter; 
    boost::shared_ptr<L3::Experience>   experience;
    boost::shared_array< glv::Point3 >  point_vertices;
    boost::shared_array< glv::Color >   point_colors;
    
    boost::shared_array< glv::Point3 >  experience_nodes_vertices;
    boost::shared_array< glv::Color >   experience_nodes_colors;
    
    L3::PoseProvider*                   pose_provider;

    void addPoseProvider( L3::PoseProvider* provider );

    void onDraw3D( glv::GLV& g );

};

/*
 *  Pose windower renderer
 */
struct PoseWindowerRenderer : Leaf
{

    PoseWindowerRenderer( L3::PoseWindower* windower ) : pose_windower(windower)
    {
        
    };

    int                                 pt_limit, pt_counter, sample_counter; 
    boost::shared_ptr<L3::Experience>   experience;
    glv::Color*                         point_colors;
    glv::Point3*                        point_vertices;
    L3::PoseWindower*                   pose_windower;

    void onDraw3D( glv::GLV& g );

};

/*
 *  Real or synthesized pose provider
 */
struct PoseProviderRenderer : Leaf
{
    PoseProviderRenderer( L3::PoseProvider* provider ) ;

    boost::shared_array< glv::Color >       colors;
    boost::shared_array< glv::Point3 >      vertices;

    int                                     counter, history;
    L3::PoseProvider*                       pose_provider;
    std::vector< std::pair<double,double> > positions;

    void onDraw3D( glv::GLV& g );

};

/*
 *Scan renderer
 */
struct ScanRenderer : Leaf
{

    ScanRenderer( L3::SwatheBuilder* builder ) : swathe_builder(builder)
    {
    
        point_colors.reset( new glv::Color[541] );
        point_vertices.reset( new glv::Point3[541] );
  
        calibration.reset( new L3::SE3( 0, 0, -1.57, 0, 0, 0 ) );

        cloud.reset( new L3::PointCloud<double>() );

        projector.reset( new L3::Projector<double>( calibration.get(), cloud.get() ) );
    }

    boost::shared_array< glv::Color >   point_colors;
    boost::shared_array< glv::Point3 >  point_vertices;
    
    L3::SwatheBuilder*  swathe_builder;
    boost::shared_ptr< L3::Projector<double> > projector;

    boost::shared_ptr< L3::SE3 >    calibration;
    boost::shared_ptr< L3::PointCloud<double> > cloud;
    
    void onDraw3D( glv::GLV& g );
    
};

/*
 *  Estimator Renderer
 */
struct EstimatorRenderer : Leaf
{
    EstimatorRenderer( L3::Estimator::Estimator<double>* ESTIMATOR ) : estimator(estimator)
    {

    }
    
    L3::Estimator::Estimator<double>* estimator;

    void onDraw3D( glv::GLV& g );


};


/*
 *  Predictor Renderer
 */
struct PredictorRenderer : Leaf
{
    PredictorRenderer( boost::shared_ptr< L3::Estimator::PoseEstimates > estimator ) : estimator(estimator)
    {

    }
    
    boost::shared_ptr< L3::Estimator::PoseEstimates > estimator;

    void onDraw3D( glv::GLV& g );


};



}   // ::Visualisers
}   // ::L3

#endif
