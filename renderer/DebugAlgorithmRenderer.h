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
            // Two bar histograms
            
            
            // Two density histograms
            int width = 180;
            boost::shared_ptr< HistogramDensityRenderer > renderer_swathe(  new HistogramDensityRenderer( glv::Rect( width, width), boost::shared_ptr< Histogram<double > >() ));
            renderers.push_back( renderer_swathe );
            
            boost::shared_ptr< HistogramDensityRenderer > renderer_experience( new HistogramDensityRenderer( glv::Rect( width, width), boost::shared_ptr< Histogram<double > >() ) );
            renderers.push_back( renderer_experience );

            (*this) << *renderer_swathe << *renderer_experience;
        }
       
        std::deque< boost::shared_ptr< HistogramDensityRenderer > > renderers;

        boost::weak_ptr< L3::Estimator::PassThrough<double> > algorithm ;

    };

}
}

#endif

