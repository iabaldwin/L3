#ifndef L3_VISUALISERS_CONTROLS_H
#define L3_VISUALISERS_CONTROLS_H

namespace L3
{
namespace Visualisers
{

struct Action
{
    virtual void operator()( glv::View* v )= 0;
};

struct Maximise : Action
{
    virtual void operator()( glv::View* v )
    {
        v->maximize();
        v->bringToFront();
    }
};

struct Toggle: Action
{
    virtual void operator()( glv::View* v )
    {
        if ( v->enabled( glv::Property::Maximized ) )
            v->restore();
        else
            v->maximize();
    }
};

struct EventController : glv::EventHandler
{
    EventController( glv::View* view, glv::Event::t type, Action* action  ) 
        :  view(view), 
            action(action),
            last_down(0.0)
    {
        view->addHandler( type, *this );
    }

    glv::View*  view;
    Action*     action;
    
    double last_down;
    L3::Timing::ChronoTimer t;
 
    virtual bool onEvent( glv::View& v, glv::GLV& g)
    {
        if (( t.elapsed() - last_down ) < .5 )
        {
            std::cout << action << ":" << view << std::endl;

            (*action)( view ); 
            // Debouncer 
            last_down = 0.0; 
        }
        else
            last_down  = t.elapsed();

        return false;
    }
};

struct DoubleClickController : EventController
{

    DoubleClickController( glv::View* view, Action* action ) 
        : EventController( view, glv::Event::MouseDown, action )
    {
    }

};

struct DoubleClickMaximiseToggle : DoubleClickController
{
    DoubleClickMaximiseToggle( glv::View* view ) : DoubleClickController( view, new Toggle() )
    {
    }
};


//struct DataDumper : EventController
//{

    //DataDumper( std::list < L3::Dumpable* > dump_targets ) : targets(dump_targets)
    //{

    //}

    //std::list < L3::Dumpable* > targets;

	//bool onEvent( glv::Event::t e, glv::GLV& g)
    //{

        //const glv::Keyboard& k = g.keyboard();
        //int key = k.key();

        //// Special switch key
        //if (key == 'd')
            //std::for_each( targets.begin(), targets.end(), std::mem_fun( &Dumpable::dump ) );

        //return true;
    //}

//};

}
}

#endif

