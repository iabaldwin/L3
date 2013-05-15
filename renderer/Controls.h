#ifndef L3_VISUALISERS_CONTROLS_H
#define L3_VISUALISERS_CONTROLS_H

#include "RenderCore.h"
#include "Components.h"
#include "QueryInterface.h"

#include <set>

namespace L3
{
namespace Visualisers
{

struct Action
{
    virtual void operator()( glv::View* v )= 0;
};

struct NoAction : Action
{
    void operator()( glv::View* v )
    {
    }
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

struct MouseQuery : EventController
{
    MouseQuery( glv::View3D* view ) 
        : EventController( view, static_cast< glv::Event::t >( SELECT_CLICK ), new NoAction() ) ,
            composite( dynamic_cast< L3::Visualisers::Composite* >(view))
    {
        interface.reset( new L3::Visualisers::QueryInterface() );
   
        (*composite) << *interface;
    }

    boost::shared_ptr< L3::Visualisers::QueryInterface > interface;

    //boost::weak_ptr< L3::Visualisers::Composite > composite;
    L3::Visualisers::Composite* composite;

    //std::set< L3::Visualisers::SelectableLeaf* > current_leafs;
    std::map< L3::Visualisers::SelectableLeaf*, btRigidBody*  > current_leafs;

    virtual bool onEvent( glv::View& v, glv::GLV& g)
    {
        std::list< L3::Visualisers::SelectableLeaf* > leafs;
        
        // Are they query-able?
        for( std::list< L3::Visualisers::Leaf* >::iterator it = composite->components.begin();
                it != composite->components.end();
                it++ )
        {
            if ( L3::Visualisers::SelectableLeaf* leaf = dynamic_cast<L3::Visualisers::SelectableLeaf*>( *it ) )
                leafs.push_back( leaf );
        }

        if ( leafs.size() == 0 )
            return false;

        for( std::list< L3::Visualisers::SelectableLeaf* >::iterator it = leafs.begin();
                it != leafs.end();
                it++ )
        {
            std::map< L3::Visualisers::SelectableLeaf*, btRigidBody *>::iterator query = current_leafs.find( *it );

            if( query == current_leafs.end() )
            {
                // We don't have it and we need it
                interface->m_dynamicsWorld->addRigidBody( (*it)->box_body.get() );
                current_leafs.insert( std::make_pair(*it, (*it)->box_body.get() ) );
            }
                 
        }

        // Query them
        const glv::Mouse& m = g.mouse();
      
        int viewport[4];
        glGetIntegerv(GL_VIEWPORT,viewport);

        double x1,y1,z1;
        double x2,y2,z2;

        // Project, twice
        gluUnProject( m.x(),
                      viewport[3] - m.y(),
                      0.0,
                      composite->model,
                      composite->projection,
                      viewport,
                      &x1,&y1,&z1);

        gluUnProject( m.x(),
                      viewport[3] - m.y(),
                      1.0,
                      composite->model,
                      composite->projection,
                      viewport,
                      &x2,&y2,&z2);

        std::list<const btCollisionObject*> hit_results;
       
        interface->query( x1, x2, y1, y2, z1, z2, hit_results );
        
        std::cout << hit_results.size() << std::endl;

        return false;
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

