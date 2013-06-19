#ifndef L3_VISUALISERS_WIDGETS_H
#define L3_VISUALISERS_WIDGETS_H

#include <GLV/glv.h>

#include "L3.h"

#include <deque>
#include <boost/shared_ptr.hpp>

namespace L3
{
namespace Visualisers
{

    struct DensityController : glv::View
    {
        DensityController() : glv::View( glv::Rect(180,20 ) )
        {
            this->disable( glv::DrawBorder );
        }

        std::deque< boost::shared_ptr< glv::Label > > labels;

        std::deque< boost::shared_ptr< glv::Widget > > widgets;

        void attachHistogram( boost::shared_ptr< L3::Histogram<double> > histogram );

    };

}
}

#endif

