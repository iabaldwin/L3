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

        return false;
    }


    TableToggler::TableToggler(  const glv::Rect& rect, std::deque< boost::shared_ptr< glv::Table > > * tables ) : tables(tables), current_table(0), glv::View(rect)
    {
        for( std::deque< boost::shared_ptr< glv::Table > >::iterator it = tables->begin();
                it != tables->end();
                it++ )
            (*it)->disable( glv::Property::Visible );

        (*tables)[0]->enable( glv::Property::Visible );

        this->enable( glv::DrawBorder );

        num_tables = tables->size();

        page_pointer.reset( new glv::Buttons( rect, num_tables, 1 ,false, true));
        this->operator<<( *page_pointer );

        page_pointer->maximize();
        page_pointer->setValue( true, 0, 0 );
    }

    bool TableToggler::onEvent( glv::Event::t type, glv::GLV& g )
    {
        if ( (int)type == TABLE_TOGGLE ) 
        {

            int current_tables = tables->size();

            if( current_tables != num_tables )
            {
                //page_pointer->remove();
                //page_pointer.reset( new glv::Buttons( rect, current_tables, 1 ,false, true));
                //this->operator<<( *page_pointer );
                //page_pointer->maximize();

                num_tables = current_tables;
                page_pointer->data().resize( glv::Data::BOOL, num_tables, 1 );
            }

            (*tables)[current_table]->disable( glv::Property::Visible );

            current_table++;

            if( current_table == tables->size() )
                current_table = 0;

            (*tables)[current_table]->enable( glv::Property::Visible );

            page_pointer->setValue( true, current_table, 0 );

        }

        return false;
    }

}
}
