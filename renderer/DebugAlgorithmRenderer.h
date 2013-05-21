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
            boost::shared_ptr< glv::View > view1( new glv::View( glv::Rect(width,width) ) );
            views.push_back( view1 ); 

            boost::shared_ptr< glv::View > view2( new glv::View( glv::Rect(width,width) ) );
            views.push_back( view2 ); 

            (*this) << *view1 << *view2;

            // Two density histograms
            boost::shared_ptr< HistogramDensityRenderer > renderer_swathe(  new HistogramDensityRenderer( glv::Rect( width, width), boost::shared_ptr< Histogram<double > >() ));
            renderers.push_back( renderer_swathe );
            
            boost::shared_ptr< HistogramDensityRenderer > renderer_experience( new HistogramDensityRenderer( glv::Rect( width, width), boost::shared_ptr< Histogram<double > >() ) );
            renderers.push_back( renderer_experience );

            (*this) << *renderer_swathe << *renderer_experience;
        
        }
       
        std::deque< boost::shared_ptr< glv::View > >                views;
        std::deque< boost::shared_ptr< HistogramDensityRenderer > > renderers;

        void setInstance( boost::shared_ptr< L3::Estimator::PassThrough<double> > algo)
        {
            this->algorithm = algo;
           
            boost::shared_ptr< L3::Estimator::PassThrough<double> > algo_ptr = this->algorithm.lock();

            if( !algo_ptr )
                return;

            this->renderers[0]->hist = algo_ptr->data.swathe_histogram;
            this->renderers[1]->hist = algo_ptr->data.experience_histogram;
        
        }
        boost::weak_ptr< L3::Estimator::PassThrough<double> > algorithm ;

    };

}
}

#endif

