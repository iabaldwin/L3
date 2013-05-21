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


    TableToggler::TableToggler( std::deque< boost::shared_ptr< glv::Table > > * tables ) : tables(tables), current_table(0)
        {

            for( std::deque< boost::shared_ptr< glv::Table > >::iterator it = tables->begin();
                    it != tables->end();
                    it++ )
            (*it)->disable( glv::Property::Visible );

            (*tables)[0]->enable( glv::Property::Visible );
        
            this->disable( glv::DrawBorder | glv::CropChildren | glv::FocusHighlight  | glv::Visible );
        }


    bool TableToggler::onEvent( glv::Event::t type, glv::GLV& g )
    {
        if ( (int)type == TABLE_TOGGLE ) 
        {
            (*tables)[current_table]->disable( glv::Property::Visible );
         
            current_table++;

            if( current_table == tables->size() )
                current_table = 0;
 
            (*tables)[current_table]->enable( glv::Property::Visible );

           
        }

    }

}
}
