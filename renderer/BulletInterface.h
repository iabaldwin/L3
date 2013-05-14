#ifndef L3_VISUALISERS_BULLET_INTERFACE
#define L3_VISUALISERS_BULLET_INTERFACE

#include "L3.h"

/*
 *Used for ray-query
 */
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"
#include "Components.h"
#include "/Users/ian/code/thirdparty/bullet/Demos/OpenGL/GLDebugDrawer.h"


static GLDebugDrawer sDebugDraw;

namespace L3
{
namespace Visualisers
{

    struct Box
    {
        struct bounds
        {
            float x,y,z;
        };

        bounds lower, upper;

        Box( float x_lower, float y_lower, float z_lower, 
                float x_upper, float y_upper, float z_upper )
        {

            lower.x = x_lower;
            lower.y = y_lower;
            lower.z = z_lower;
            
            upper.x = x_upper;
            upper.y = y_upper;
            upper.z = z_upper;
        }
        
    };

    struct BoxRenderer : L3::Visualisers::Leaf
    {

        BoxRenderer() 
        {
            box = new Box( -10, -10, -10, 10, 10, 10 );
            box_shape = new btBoxShape( btVector3(10,10,10) );
            box_body  = new btRigidBody( 10.0f, new btDefaultMotionState, box_shape, btVector3(0,0,0) );
        }

        Box* box;
        btRigidBody* box_body;
        btCollisionShape* box_shape;
        
        void onDraw3D( glv::GLV& g )
        {
            glv::Point3 vertices[16];
            glv::Color  colors[16];

            // Lower
            vertices[0]( box->lower.x, box->lower.y, box->lower.z );
            vertices[1]( box->lower.x, box->upper.y, box->lower.z );
            vertices[2]( box->upper.x, box->upper.y, box->lower.z );
            vertices[3]( box->upper.x, box->lower.y, box->lower.z );

            glv::draw::paint( glv::draw::LineLoop, vertices, colors, 4 );

            // Upper
            vertices[0]( box->lower.x, box->lower.y, box->upper.z );
            vertices[1]( box->lower.x, box->upper.y, box->upper.z );
            vertices[2]( box->upper.x, box->upper.y, box->upper.z );
            vertices[3]( box->upper.x, box->lower.y, box->upper.z );

            glv::draw::paint( glv::draw::LineLoop, vertices, colors, 4 );
       
        }
    };

    struct BulletInterface : L3::Visualisers::Leaf
    {

        BulletInterface()
        {
            m_collisionConfiguration.reset( new btDefaultCollisionConfiguration() );

            m_dispatcher.reset( new btCollisionDispatcher(m_collisionConfiguration.get() ) );

            m_broadphase.reset( new btDbvtBroadphase() );

            sol.reset( new btSequentialImpulseConstraintSolver() );

            m_dynamicsWorld.reset( new btDiscreteDynamicsWorld( m_dispatcher.get(), m_broadphase.get(), sol.get(), m_collisionConfiguration.get() ) );

            m_dynamicsWorld->setGravity(btVector3(0,-10,0));

            addShape();

            t.begin();
        }

        L3::Timing::ChronoTimer t;

        void onDraw3D( glv::GLV& g )
        {
            btVector3 blue(0,0,1);
            btVector3 pos(0,0,0);
            
            if (m_dynamicsWorld)
            {
            
                float ms = t.elapsed();
                m_dynamicsWorld->stepSimulation( t.elapsed()/ 1000000.f);
                m_dynamicsWorld->debugDrawWorld();
            }


            doQuery();

            for( std::list< L3::Visualisers::Leaf* >::iterator it = leafs.begin();
                    it != leafs.end();
                    it++ )
                (*it)->onDraw3D( g );

            t.begin();
           
        }

        void addShape()
        {
            
            // Add Box
            BoxRenderer* renderer = new L3::Visualisers::BoxRenderer();
            leafs.push_back( renderer );
            m_dynamicsWorld->addRigidBody( renderer->box_body );
        }

        void doQuery()
        {
            m_dynamicsWorld->updateAabbs();
            m_dynamicsWorld->computeOverlappingPairs();

            btVector3 red(1,0,0);
            {
                btVector3 from(-30,-30,30);
                btVector3 to(0,0,0);
                sDebugDraw.drawLine(from,to,btVector4(0,0,0,1));
                
                btCollisionWorld::AllHitsRayResultCallback allResults(from,to);
                
                allResults.m_flags |= btTriangleRaycastCallback::kF_KeepUnflippedNormal;
                m_dynamicsWorld->rayTest(from,to,allResults);

                for (int i=0;i<allResults.m_hitFractions.size();i++)
                {
                    btVector3 p = from.lerp(to,allResults.m_hitFractions[i]);
                    sDebugDraw.drawSphere(p,1,red);
                }
            }
        }

        std::deque< boost::shared_ptr< btRigidBody >  > bodies;

        boost::shared_ptr< btDefaultCollisionConfiguration > m_collisionConfiguration;

        boost::shared_ptr< btCollisionDispatcher > m_dispatcher;

        boost::shared_ptr< btBroadphaseInterface >  m_broadphase;
   
        boost::shared_ptr< btSequentialImpulseConstraintSolver > sol;
   
        boost::shared_ptr< btDiscreteDynamicsWorld > m_dynamicsWorld;
   
        btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
                     
        boost::shared_ptr< btDefaultMotionState > myMotionState;
             
        boost::shared_ptr< btCollisionShape > groundShape;
             
        btTransform groundTransform;
   
        std::list< L3::Visualisers::Leaf* > leafs;
    };
}
}

#endif

