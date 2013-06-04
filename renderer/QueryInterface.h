#ifndef L3_VISUALISERS_BULLET_INTERFACE
#define L3_VISUALISERS_BULLET_INTERFACE

#include "L3.h"

/*
 *Used for ray-query
 */
//#include "btBulletDynamicsCommon.h"
//#include "BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"
//#include "Components.h"
//#include "/Users/ian/code/thirdparty/bullet/Demos/OpenGL/GLDebugDrawer.h"


static GLDebugDrawer sDebugDraw;

namespace L3
{
namespace Visualisers
{

    struct Box
    {
        //Box( btCollisionShape* box_shape, btRigidBody* box_body ) 
            //: box_shape(box_shape),
                //box_body(box_body)
        {
            //bounds = new glv::Point3[2*8];
        }
  
        //btRigidBody* box_body;
        //btCollisionShape* box_shape;
        
        glv::Point3* bounds;

        btVector3 current;

        void update( double x, double y, double z )
        {
            btTransform transform = box_body->getCenterOfMassTransform();
            transform.setOrigin(btVector3( x, y, z ) );
            box_body->setCenterOfMassTransform( transform );

            btVector3 aabbMin, aabbMax;

            // Move to current location
            //box->getAabb( aabbMin, aabbMax);
            //box->getAabb( transform, aabbMin, aabbMax);
            box_shape->getAabb( transform, aabbMin, aabbMax);

            // Lower
            bounds[0]( aabbMin[0], aabbMin[1], aabbMin[2] );
            bounds[1]( aabbMin[0], aabbMax[1], aabbMin[2] );
            bounds[2]( aabbMax[0], aabbMax[1], aabbMin[2] );
            bounds[3]( aabbMax[0], aabbMin[1], aabbMin[2] );

            // Upper 
            bounds[4]( aabbMin[0], aabbMin[1], aabbMax[2] );
            bounds[5]( aabbMin[0], aabbMax[1], aabbMax[2] );
            bounds[6]( aabbMax[0], aabbMax[1], aabbMax[2] );
            bounds[7]( aabbMax[0], aabbMin[1], aabbMax[2] );

            // Sides
            bounds[8]( aabbMin[0], aabbMin[1], aabbMin[2] );
            bounds[9]( aabbMin[0], aabbMin[1], aabbMax[2] );
            
            bounds[10]( aabbMin[0], aabbMax[1], aabbMin[2] );
            bounds[11]( aabbMin[0], aabbMax[1], aabbMax[2] );

            bounds[12]( aabbMax[0], aabbMax[1], aabbMin[2] );
            bounds[13]( aabbMax[0], aabbMax[1], aabbMax[2] );

            bounds[14]( aabbMax[0], aabbMin[1], aabbMin[2] );
            bounds[15]( aabbMax[0], aabbMin[1], aabbMax[2] );

        }
            
    };

    struct SelectableLeaf : L3::Visualisers::Leaf
    {
        SelectableLeaf( int x_size, int y_size, int z_size ) : selected(false)
        {
            box_shape.reset( new btBoxShape( btVector3(x_size, y_size, z_size) ) );
            box_body.reset( new btRigidBody( 0.0f, new btDefaultMotionState, box_shape.get(), btVector3(0,0,0) ) );
            box.reset( new Box( box_shape.get(), box_body.get() ));
        }

        bool selected;
        boost::shared_ptr< Box >                box;
        boost::shared_ptr< btRigidBody >        box_body;
        boost::shared_ptr< btCollisionShape >   box_shape;
  
        double current_x, current_y, current_z;

        void onDraw3D( glv::GLV& g )
        {
            box->update( current_x, current_y, current_z);
          
            glv::Color c = selected ? glv::Color( .7, .7, .7 ) : glv::Color( .2, .2, .2 ) ;
                
            glv::Color colors[16];

            std::fill( colors, colors+sizeof(colors)/sizeof( glv::Color), c );

            // Lower
            glv::draw::paint( glv::draw::LineLoop, box->bounds, colors, 4 );

            // Upper
            glv::draw::paint( glv::draw::LineLoop, box->bounds+4, colors, 4 );
       
            // Sides
            glv::draw::paint( glv::draw::Lines, box->bounds+8, colors, 8 );
        }
    };

    struct QueryInterface : L3::Visualisers::Leaf
    {
        
        QueryInterface()
        {
            m_collisionConfiguration.reset( new btDefaultCollisionConfiguration() );
            m_dispatcher.reset( new btCollisionDispatcher(m_collisionConfiguration.get() ) );
            m_broadphase.reset( new btDbvtBroadphase() );
            sol.reset( new btSequentialImpulseConstraintSolver() );
            m_dynamicsWorld.reset( new btDiscreteDynamicsWorld( m_dispatcher.get(), m_broadphase.get(), sol.get(), m_collisionConfiguration.get() ) );
            m_dynamicsWorld->setGravity(btVector3(0,-10,0));
        }

        btVector3 from;
        btVector3 to;

        std::deque< btVector3 > hits;

        void onDraw3D( glv::GLV& g );


        void query( double x1, double x2, 
                    double y1, double y2,
                    double z1, double z2,
                    std::list<const btCollisionObject*>& hit_results );

        boost::shared_ptr< btDefaultCollisionConfiguration > m_collisionConfiguration;
        
        boost::shared_ptr< btSequentialImpulseConstraintSolver > sol;
        
        boost::shared_ptr< btDiscreteDynamicsWorld > m_dynamicsWorld;
        
        boost::shared_ptr< btBroadphaseInterface >  m_broadphase;
        
        boost::shared_ptr< btCollisionDispatcher > m_dispatcher;
   
    };
}
}

#endif

