#include "Visualisers.h"

namespace L3
{
namespace Visualisers
{

    PoseChainRenderer::PoseChainRenderer( std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > >* poses ) 
    {
        std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > >::iterator it;
        
        for( it=poses->begin(); it < poses->end(); it+= 100 )
        {
            coords.push_back( boost::shared_ptr<L3::Visualisers::CoordinateSystem>( new L3::Visualisers::CoordinateSystem( (*it->second) ) ) );
        }
    }

    void PoseChainRenderer::onDraw3D(glv::GLV& g)
    { 
        for ( std::deque< boost::shared_ptr<L3::Visualisers::CoordinateSystem> >::iterator it = coords.begin(); it!= coords.end(); it++ )
            (*it)->onDraw3D( g );
    }


}
}

