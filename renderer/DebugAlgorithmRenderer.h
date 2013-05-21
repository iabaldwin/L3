#ifndef L3_VISUALISERS_DEBUG_RENDERER_ALGORITHM
#define L3_VISUALISERS_DEBUG_RENDERER_ALGORITHM

#include "L3.h"

namespace L3
{
namespace Visualisers
{

    struct DebugAlgorithmRenderer : glv::Table
    {

        DebugAlgorithmRenderer( boost::shared_ptr< L3::Estimator::PassThrough<double> > algorithm ) 
            : glv::Table( "x x," ),
                algorithm(algorithm)
        {
            int width = 180;
            
            // Two bar histograms
            boost::shared_ptr< HistogramVertexRenderer > vertex_renderer_swathe( new HistogramVertexRenderer( glv::Rect(width,width), boost::shared_ptr< Histogram<double> >() ) );
            vertex_renderers.push_back( vertex_renderer_swathe ); 

            boost::shared_ptr< HistogramVertexRenderer > vertex_renderer_experience( new HistogramVertexRenderer( glv::Rect(width,width), boost::shared_ptr< Histogram<double> >() ) );
            vertex_renderers.push_back( vertex_renderer_experience ); 

            (*this) << *vertex_renderer_swathe << *vertex_renderer_experience;

            // Two density histograms
            boost::shared_ptr< HistogramDensityRenderer > density_renderer_swathe(  new HistogramDensityRenderer( glv::Rect( width, width), boost::shared_ptr< Histogram<double > >() ));
            density_renderers.push_back( density_renderer_swathe );
            
            boost::shared_ptr< HistogramDensityRenderer > density_renderer_experience( new HistogramDensityRenderer( glv::Rect( width, width), boost::shared_ptr< Histogram<double > >() ) );
            density_renderers.push_back( density_renderer_experience );

            (*this) << *density_renderer_swathe << *density_renderer_experience;
        
        }
       
        std::deque< boost::shared_ptr< HistogramVertexRenderer > >    vertex_renderers;
        std::deque< boost::shared_ptr< HistogramDensityRenderer > >   density_renderers;

        void setInstance( boost::shared_ptr< L3::Estimator::PassThrough<double> > algo)
        {
            this->algorithm = algo;
           
            boost::shared_ptr< L3::Estimator::PassThrough<double> > algo_ptr = this->algorithm.lock();

            if( !algo_ptr )
                return;

            this->density_renderers[0]->hist = algo_ptr->data.swathe_histogram;
            this->density_renderers[1]->hist = algo_ptr->data.experience_histogram;
       

            this->vertex_renderers[0]->hist = algo_ptr->data.swathe_histogram;
            this->vertex_renderers[1]->hist = algo_ptr->data.experience_histogram;
        }

        boost::weak_ptr< L3::Estimator::PassThrough<double> > algorithm ;

    };

}
}

#endif

