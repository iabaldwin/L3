#include "RenderCore.h"

namespace L3
{
namespace Visualisers
{
    bool CustomTable::onEvent( glv::Event::t e, glv::GLV& g )
    {
        glv::Table::onEvent(e,g);

        if ( (int)e == TABLE_TOGGLE ) 
            this->toggle( glv::Property::Visible );

    }
}
}
