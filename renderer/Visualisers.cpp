#include "Visualisers.h"

#include <boost/scoped_array.hpp>

namespace L3
{
namespace Visualisers
{
    /*
     *  Pose chain
     */
    PoseChainRenderer::PoseChainRenderer( std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > >* poses ) 
    {
        for( std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > >::iterator it=poses->begin(); it < poses->end(); it+= 100 )
            coords.push_back( boost::shared_ptr<L3::Visualisers::CoordinateSystem>( new L3::Visualisers::CoordinateSystem( *(it->second) ) ) );
    }

    void PoseChainRenderer::onDraw3D(glv::GLV& g)
    { 
        for ( std::deque< boost::shared_ptr<L3::Visualisers::CoordinateSystem> >::iterator it = coords.begin(); it!= coords.end(); it++ )
            (*it)->onDraw3D( g );
    }

    
    /*
     *  Iterator
     */
    template <typename T>
    void IteratorRenderer<T>::onDraw3D( glv::GLV& g )
    {
        boost::shared_ptr< L3::Iterator<T> > iterator_ptr = iterator.lock();

        if( !iterator_ptr )
            return;
        
        SelectableLeaf::onDraw3D(g);

        std::deque< std::pair< double, boost::shared_ptr<T> > > window;

        iterator_ptr->getWindow( window );
        
        if( window.empty() )
            return;
        
        typename L3::Iterator<T>::WINDOW_ITERATOR it = window.begin();
        
        while( it != window.end() )
            L3::Visualisers::CoordinateSystem( *(it++->second) ).onDraw3D( g );

        current_x = window.back().second->X();
        current_y = window.back().second->Y();
        current_z = window.back().second->Z();
    }

    /*
     *  Swathe
     */
    SwatheRenderer::SwatheRenderer( L3::SwatheBuilder* SWATHE_BUILDER )  : swathe_builder(SWATHE_BUILDER), current_alloc(500)
    {
        // Projector  
        L3::SE3 calibration( 0, 0, 0, 0, -1.57, 0 ); 
       
        point_cloud.reset( new L3::PointCloud<double>() );
        projector.reset( new L3::Projector<double>( &calibration, point_cloud.get() ) );
  
        //TODO : FIX
        point_colors.reset( new glv::Color[10*100000] );
        point_vertices.reset( new glv::Point3[10*100000] );
 
    }
    
    void SwatheRenderer::realloc( int size )
    {
        pose_colors.reset( new glv::Color[size] );
        pose_vertices.reset( new glv::Point3[size] );

        current_alloc = size;
    }

    void SwatheRenderer::onDraw3D( glv::GLV& g )
    {
        // Do projection
        projector->project( swathe_builder->swathe );

        if ( point_cloud->num_points > current_alloc )
            realloc( point_cloud->num_points );

        PointCloud<double>::ITERATOR point_iterator = point_cloud->begin();

        int counter = 0;

        while( point_iterator < point_cloud->end() )
        {
            point_colors[counter].set( .5, .5, .5, 255 );
            point_vertices[counter++]( point_iterator->x , point_iterator->y , point_iterator->z);
            point_iterator+=10; 
        }
        
        glv::draw::paint( glv::draw::Points, point_vertices.get(), point_colors.get(), counter );
    
    }

    /*
     *  Experience
     */
    ExperienceRenderer::ExperienceRenderer( boost::shared_ptr<L3::Experience> experience ) : experience(experience), pose_provider(NULL)
    {
        pt_limit = 10*10000;

        point_vertices.reset( new glv::Point3[pt_limit] );
        point_colors.reset( new glv::Color[pt_limit] );
  
        experience_nodes_vertices.reset( new glv::Point3[experience->sections.size()] );
        experience_nodes_colors.reset( new glv::Color[experience->sections.size()] );
    }

    void ExperienceRenderer::addPoseProvider( L3::PoseProvider* provider )
    {
        pose_provider = provider;
    }

    void ExperienceRenderer::onDraw3D( glv::GLV& g )
    {
        // Plot experience section locations
        std::deque<L3::experience_section>::iterator it = experience->sections.begin();

        while( it != experience->sections.end() )
        {
            experience_nodes_vertices[ std::distance( experience->sections.begin(), it ) ]( it->x, it->y, 0 );
            experience_nodes_colors[ std::distance( experience->sections.begin(), it ) ].set( 255, 0, 0 );
            it++;
        }
        glv::draw::paint( glv::draw::Points, experience_nodes_vertices.get(), experience_nodes_colors.get(), experience->sections.size());

        // Update experience
        L3::SE3 update = (*pose_provider)();
        experience->update( update.X(), update.Y() );
    }

    /*
     * Real, or synthesized, pose provider
     */
    PoseProviderRenderer::PoseProviderRenderer( L3::PoseProvider* provider ) : pose_provider(provider),
                                                                                counter(0), 
                                                                                history(20)
    {
        positions.resize( history );
   
        vertices.reset( new glv::Point3[history] );
        colors.reset( new glv::Color[history] );
    }


    void PoseProviderRenderer::onDraw3D( glv::GLV& g )
    {
        L3::SE3 pose = (*pose_provider)();

        positions[ counter++%(positions.size()) ] = std::make_pair( pose.X(), pose.Y() );

        for ( int it = 0; it <history; it++ )
            vertices[it]( positions[it].first, positions[it].second, 0.0 );

        glv::draw::paint( glv::draw::Points, vertices.get(), colors.get(), positions.size() );

    }

    /*
     *  Pose windower
     */

    void PoseWindowerRenderer::onDraw3D( glv::GLV& g )
    {
        std::deque< std::pair< double, boost::shared_ptr<L3::SE3> > >::iterator it = pose_windower->window->begin();

        while( it != pose_windower->window->end() )
            CoordinateSystem( *(it++)->second ).onDraw3D(g );
    }

    /*
     *  Scan renderer
     */
    void ScanRenderer::onDraw3D( glv::GLV& g )
    {
        // Project just 1 scan
        if ( swathe_builder->swathe.size() == 0 )
            return;
        
        std::vector< std::pair< boost::shared_ptr<L3::Pose>, boost::shared_ptr<L3::LIDAR> > > single_scan( swathe_builder->swathe.begin(), swathe_builder->swathe.begin()+1 );
        projector->project( single_scan );

        for( int i=0; i<cloud->num_points; i++ )
            point_vertices[i]( cloud->points[i].x, cloud->points[i].y, cloud->points[i].z );
        
        glv::draw::paint( glv::draw::Points, point_vertices.get(), point_colors.get(), cloud->num_points );
    }

    /*
     *  Predictor Renderer
     */
    void PredictorRenderer::onDraw3D( glv::GLV& g )
    {
        L3::ReadLock( estimator->mutex );

        for ( std::vector< L3::SE3 >::iterator it = estimator->estimates.begin();
                it != estimator->estimates.end(); 
                it++ )
            CoordinateSystem( *(it ), 2.0 ).onDraw3D(g);

    }


}
}

// Explicit Instantiations
template L3::Visualisers::IteratorRenderer<L3::SE3>::IteratorRenderer( boost::shared_ptr< L3::Iterator<L3::SE3> >);
