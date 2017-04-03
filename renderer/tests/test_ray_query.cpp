#include <glv.h>
#include <glv_binding.h>
#include <glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "Components.h"
#include "Controls.h"
//#include "QueryInterface.h"

#include "Imagery.h"

//struct tmp : L3::Visualisers::SelectableLeaf, L3::Visualisers::Controllable
//{

    //tmp() : L3::Visualisers::SelectableLeaf(50,50,50)
    //{

    //}

    //void onDraw3D( glv::GLV& g )
    //{
        //L3::Visualisers::SelectableLeaf::onDraw3D(g);
        
        //glv::Point3 pts[1000];
        //glv::Color colors[1000];

        //for( int i=0; i<1000; i++ )
            //pts[i]( control_x + random()%100-50, control_y + random()%100-50, control_z + random()%100-50 );

        //glv::draw::paint( glv::draw::Points, pts, colors, 1000 );
   
    
        //for( std::list< L3::Visualisers::SelectableLeaf* >::iterator it = selectables.begin();
                //it != selectables.end();
                //it++ )
        //{
            //double delta_x = double(random()%10)/10.0;
            //(*it)->current_x += delta_x;
            //(*it)->current_y += double(random()%10)/10.0;
        //}
    
    //}

    //std::list< L3::Visualisers::SelectableLeaf* > selectables;
    
    //tmp& operator<<( L3::Visualisers::SelectableLeaf* selectable )
    //{
        //selectables.push_front( selectable );

        //return *this;
    //}
//};

//int main (int argc, char ** argv)
//{

    ////glv::GLV top;
    //L3::Visualisers::L3GLV top;

    //glv::Window win(1400, 800, "Soaring");

    //// Colors
    //top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    //L3::Visualisers::Composite              composite( glv::Rect( 1000, 600) );
    //L3::Visualisers::BasicPanController     controller(composite.position);
    //L3::Visualisers::Grid                   grid;
            
    //L3::Visualisers::MouseQuerySelect query(  &composite );
    ////L3::Visualisers::WASDManager selection_manager( &query );

    //// Add Boxes
    //L3::Visualisers::SelectableLeaf* renderer = new L3::Visualisers::SelectableLeaf( 20, 20, 20 );
    //L3::Visualisers::SelectableLeaf* renderer2 = new L3::Visualisers::SelectableLeaf( 5, 10, 15 );
    //tmp* renderer3 = new tmp();


    //L3::Configuration::Begbroke begbroke;
    //begbroke.loadDatum();

    //boost::shared_ptr< L3::Visualisers::LocaleRenderer > locale_renderer = L3::Visualisers::LocaleRendererFactory::buildLocale( begbroke );



    //(*renderer3) <<  renderer << renderer2;

    //top << ( composite << grid << *renderer << *renderer2 << *renderer3  << *locale_renderer);

    //win.setGLV(top);
    //glv::Application::run();
//}


