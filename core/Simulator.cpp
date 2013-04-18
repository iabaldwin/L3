#include "Simulator.h"

namespace L3
{
    namespace Simulator
    {

        bool LHLVGenerator::update( double t )
        {
            std::vector<double> data( L3::Sizes<L3::LHLV>::elements, 0.0 );

            data[9] = 10.0;
            data[5] = .1;
            data[3] = .1;

            window.push_back( std::make_pair( t, boost::make_shared< L3::LHLV >( data ) ) );

            std::deque< std::pair< double, boost::shared_ptr<L3::LHLV > > >::iterator it = window.begin();

        }
    }
}

