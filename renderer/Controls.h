#ifndef L3_VISUALISERS_CONTROLS_H
#define L3_VISUALISERS_CONTROLS_H

#include "RenderCore.h"
#include "Components.h"
//#include "QueryInterface.h"

#include <set>

namespace L3
{
namespace Visualisers
{

struct Action
{
    virtual void apply( glv::View* v ) = 0;
    virtual void invert( glv::View* v ){}; 

};

struct NoAction : Action
{
    void apply( glv::View* v )
    {
    }
};

struct SelectAction : Action
{

    // Apply the action
    void apply( glv::View* view )
    {
        // we are gauranteed of this
        //if( L3::Visualisers::SelectableLeaf* ptr = reinterpret_cast< L3::Visualisers::SelectableLeaf* >( view ) )
            //ptr->selected = true;
    }

    void invert(glv::View* view )
    {
        //if( L3::Visualisers::SelectableLeaf* ptr = reinterpret_cast< L3::Visualisers::SelectableLeaf* >( view ) )
            //ptr->selected = false;
    }

};


struct HighLightAction : Action
{
    void apply( glv::View* v )
    {
    }
};

struct Maximise : Action
{
    virtual void apply( glv::View* v )
    {
        v->maximize();
        v->bringToFront();
    }
};

struct Toggle: Action
{
    virtual void apply( glv::View* v )
    {
        if ( v->enabled( glv::Property::Maximized ) )
            v->restore();
        else
            v->maximize();
    }
};

template <typename T>
struct Increment : Action
{

    Increment( T& t ) : t(t)
    {

    }

    T& t ;

    void apply( glv::View* )
    {
        t++;
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
        if (( t.elapsed() - last_down ) < .2 )
        {
            action->apply( view ); 
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

template <typename T>
struct Incrementer : EventController
{
    Incrementer( glv::View* view, T t  ) 
        : EventController( view,  static_cast< glv::Event::t >( DBG_X ), new Increment<T>(t))
    {
    }
};

struct MouseQuery : EventController
{
    MouseQuery( glv::View3D* view, Action* action  ) 
        //: EventController( view, static_cast< glv::Event::t >( SELECT_CLICK ), action ) ,
        : EventController( view, glv::Event::MouseDown, action ) ,
            composite( dynamic_cast< L3::Visualisers::Composite* >(view))
    {
        //interface.reset( new L3::Visualisers::QueryInterface() );
       
        // DBG
        //(*composite) << *interface;
    }

    //boost::shared_ptr< L3::Visualisers::QueryInterface > interface;

    L3::Visualisers::Composite* composite;
    //boost::weak_ptr< L3::Visualisers::Composite > composite;

    //std::map< L3::Visualisers::SelectableLeaf*, btRigidBody*  > current_leafs;

    virtual bool onEvent( glv::View& v, glv::GLV& g)
    {
        //const glv::Keyboard& k = g.keyboard();

        //if ( !k.ctrl() )
            //return false; 

        //std::list< L3::Visualisers::SelectableLeaf* > leafs;

        //// Are they query-able?
        //for( std::list< L3::Visualisers::Leaf* >::iterator it = composite->components.begin();
                //it != composite->components.end();
                //it++ )
        //{
            //if ( L3::Visualisers::SelectableLeaf* leaf = dynamic_cast<L3::Visualisers::SelectableLeaf*>( *it ) )
                //leafs.push_back( leaf );
        //}

        //// No leafs are selectable in the view
        //if ( leafs.empty() )
            //return false;

        //for( std::list< L3::Visualisers::SelectableLeaf* >::iterator it = leafs.begin();
                //it != leafs.end();
                //it++ )
        //{
            //std::map< L3::Visualisers::SelectableLeaf*, btRigidBody *>::iterator query = current_leafs.find( *it );

            //if( query == current_leafs.end() )
            //{
                //// We don't have it and we need it
                ////interface->m_dynamicsWorld->addRigidBody( (*it)->box_body.get() );
                //current_leafs.insert( std::make_pair(*it, (*it)->box_body.get() ) );
            //}

        //}

        //// Query them
        //const glv::Mouse& m = g.mouse();

        //double x1,y1,z1;
        //double x2,y2,z2;

        //int viewport[4];

        //viewport[0] = composite->left();
        //viewport[1] = composite->top();
        //viewport[2] = composite->right();
        //viewport[3] = composite->bottom();

        //double relative_x = m.x();
        //double relative_y = viewport[3]-m.y(); 

        //if( (relative_x > viewport[2] ) || (relative_y < 0 ) )
            //return false;

        //// Project, twice
        //gluUnProject( relative_x, 
                //relative_y, 
                //0.0,                      // Frustrum begin
                //composite->model,
                //composite->projection,
                //viewport,
                //&x1,&y1,&z1);

        //gluUnProject( relative_x,
                //relative_y, 
                //1.0,                      // Frustrum end
                //composite->model,
                //composite->projection,
                //viewport,
                //&x2,&y2,&z2);

        //std::list<const btCollisionObject*> hit_results;

        //interface->query( x1, x2, y1, y2, z1, z2, hit_results );

        //// Deselect everything
        //for( std::map< L3::Visualisers::SelectableLeaf*, btRigidBody*  >::iterator leaf_iterator =  current_leafs.begin();
                //leaf_iterator != current_leafs.end();
                //leaf_iterator++ )
        //{
            //action->invert( reinterpret_cast<glv::View*>(leaf_iterator->first ) );
        //}

        //// Find the corresponding views
        //for( std::list<const btCollisionObject*>::iterator it =  hit_results.begin();
                //it != hit_results.end();
                //it++ )
        //{
            //// Action them, bimap needed
            //for( std::map< L3::Visualisers::SelectableLeaf*, btRigidBody*  >::iterator leaf_iterator =  current_leafs.begin();
                    //leaf_iterator != current_leafs.end();
                    //leaf_iterator++ )
            //{
                //if( *it == leaf_iterator->second )
                    //action->apply( reinterpret_cast<glv::View*>(leaf_iterator->first ) );
            //}

        //}

        return false; //Don't bubble
    }

};

struct MouseQuerySelect : MouseQuery
{
    MouseQuerySelect( glv::View3D* view ) 
        : MouseQuery( view, new SelectAction() )
    {
    }


};

struct InputManager : glv::EventHandler
{
    //std::map< L3::Visualisers::SelectableLeaf*, btRigidBody*  >* controllables;;
};

struct WASDController : InputManager
{
    bool onEvent( glv::View& v, glv::GLV& g )
    {
        //const glv::Keyboard& k = g.keyboard();
        //int key = k.key();

        //double x = 0; 
        //double y = 0; 
        //double delta = 10.0;

        //float multiplier = 1.0;
        //if (k.shift())
            //multiplier = 2.0;

        //switch (key)
        //{
            //case 'w':
                //y-=delta*multiplier;
                //break;
            
            //case 's':
                //y+=delta*multiplier;
                //break;

            //case 'd':
                //x-=delta*multiplier;
                //break;

            //case 'a':
                //x+=delta*multiplier;
                //break;
            
            //default: 
                //break;
        //};

        ////for( std::map< L3::Visualisers::SelectableLeaf*, btRigidBody*  >::iterator leaf_iterator = controllables->begin();
            ////leaf_iterator != controllables->end(); 
            ////leaf_iterator++ )
        ////{
            ////// Is the selector also controllable?
            ////if ( L3::Visualisers::Controllable* ptr = dynamic_cast< L3::Visualisers::Controllable* >( leaf_iterator->first ) )
            ////{
                ////if( leaf_iterator->first->selected )
                ////{
                    ////ptr->control_x += x;
                    ////ptr->control_y += y;
                    //////ptr->control_z += z;
                    ////leaf_iterator->second->translate( btVector3(x,y,0) );
                ////}
            ////}
        ////}
   
        return false;
    }
};

struct SelectionManager
{
    SelectionManager( MouseQuerySelect* selector, InputManager* input ) : select(select), input(input)
    {
        //selector->view->addHandler( glv::Event::KeyDown, *input );
        //input->controllables = &selector->current_leafs;
    }

    InputManager*       input;
    MouseQuerySelect*   select;

};

struct WASDManager : SelectionManager
{
    WASDManager( MouseQuerySelect* selector) : SelectionManager( selector, new WASDController() )
    {
    }

};

struct CompositeLeafViewToggle : glv::Buttons
{

    CompositeLeafViewToggle( boost::shared_ptr< L3::Visualisers::Leaf > leaf, std::string name, const glv::Rect& r= glv::Rect(), int nx=1, int ny=1,bool momentary=false, bool mutExc=false )
        : glv::Buttons( r, nx, ny, momentary, mutExc ), leaf(leaf)
    {

        label = boost::make_shared< glv::Label >( name );
        label->pos( glv::Place::CL, 5, 0 ).anchor( glv::Place::CR ); 

        *this << *label;
    }
        
    boost::shared_ptr< glv::Label > label;

    boost::weak_ptr< L3::Visualisers::Leaf > leaf;

    bool toggle_val;

    bool onEvent( glv::Event::t e, glv::GLV& g )
    {
        glv::Buttons::onEvent(e,g);

        boost::shared_ptr< L3::Visualisers::Leaf > leaf_ptr = leaf.lock();

        if ( leaf_ptr )
            leaf_ptr->visible = getValue();

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

