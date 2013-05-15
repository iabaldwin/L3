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
        
        Box( btRigidBody* box ) : box(box)
        {
            bounds = new glv::Point3[2*8];
       
            update();
        }
  
        btRigidBody* box;
        
        glv::Point3* bounds;

        void update()
        {
            btVector3 aabbMin, aabbMax;

            box->getAabb( aabbMin, aabbMax);

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

        SelectableLeaf( int size ) : highlighted(true)
        {
            box_shape.reset( new btBoxShape( btVector3(size,size,size) ) );
            box_body.reset( new btRigidBody( 0.0f, new btDefaultMotionState, box_shape.get(), btVector3(0,0,0) ) );
            box.reset( new Box( box_body.get() ));
        }

        bool highlighted;
        boost::shared_ptr< Box >                box;
        boost::shared_ptr< btRigidBody >        box_body;
        boost::shared_ptr< btCollisionShape >   box_shape;
    
        void translate( const btVector3& vec )
        {
            box_body->translate(vec);
            box->update();
        }

        void onDraw3D( glv::GLV& g )
        {
            glv::Color c = highlighted ? glv::Color( .7, .7, .7 ) : glv::Color( .2, .2, .2 ) ;
                
            glv::Color  colors[16];

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
            //addShape();
            //t.begin();
        }

        L3::Timing::ChronoTimer t;

        void onDraw3D( glv::GLV& g )
        {
            glv::Point3 vertices[2];
            glv::Color  colors[2];

            vertices[0]( from[0], from[1], from[2] );
            vertices[1]( to[0], to[1], to[2] );

            glv::draw::lineStippling(true);
            glv::draw::lineWidth(2);
            glv::draw::lineStipple(4, 0xAAAA );
            glv::draw::paint( glv::draw::Lines, vertices, colors, 2 );
            glv::draw::lineStippling(false);

            sDebugDraw.drawLine(from,to,btVector4(0,0,0,1));
            
            //btVector3 blue(0,0,1);
            //btVector3 pos(0,0,0);
            
            //if (m_dynamicsWorld)
            //{
            
                //float ms = t.elapsed();
                ////m_dynamicsWorld->stepSimulation( t.elapsed()/ 1000000.f);
                //m_dynamicsWorld->debugDrawWorld();
            //}

            //renderer->translate( btVector3(.1, 0, 0 ) );

            //doQuery();

            //for( std::list< L3::Visualisers::Leaf* >::iterator it = leafs.begin();
                    //it != leafs.end();
                    //it++ )
                //(*it)->onDraw3D( g );

            //t.begin();
           
        }

        //SelectableLeaf* renderer;
        //SelectableLeaf* renderer2;
        
        //void addShape()
        //{
            
            //// Add Box
            //renderer = new L3::Visualisers::SelectableLeaf( 20 );
            //renderer2 = new L3::Visualisers::SelectableLeaf( 5 );

            //leafs.push_back( renderer );
            //leafs.push_back( renderer2 );
            //m_dynamicsWorld->addRigidBody( renderer->box_body.get() );
            //m_dynamicsWorld->addRigidBody( renderer2->box_body.get() );
        //}

        btVector3 from;
        btVector3 to;
                

        void query( double x1, double x2, 
                    double y1, double y2,
                    double z1, double z2,
                    std::list<const btCollisionObject*>& hit_results )
        {
            m_dynamicsWorld->updateAabbs();
            m_dynamicsWorld->computeOverlappingPairs();

            hit_results.clear();

            btVector3 red(1,0,0);
            {
                //from = btVector3(-30,-70,30);
                //to = btVector3(0,0,0);
                
                from = btVector3(x1, y1, z1);
                to = btVector3( x2, y2, z2 );
                
                btCollisionWorld::AllHitsRayResultCallback allResults(from,to);
                
                allResults.m_flags |= btTriangleRaycastCallback::kF_KeepUnflippedNormal;
                m_dynamicsWorld->rayTest(from,to,allResults);

                for (int i=0;i<allResults.m_hitFractions.size();i++)
                {
                    btVector3 p = from.lerp(to,allResults.m_hitFractions[i]);
                    sDebugDraw.drawSphere(p,1,red);
               
                    hit_results.push_back( allResults.m_collisionObjects[i] );
                }
            }
        }

        boost::shared_ptr< btDefaultCollisionConfiguration > m_collisionConfiguration;
        
        boost::shared_ptr< btSequentialImpulseConstraintSolver > sol;
        
        boost::shared_ptr< btDiscreteDynamicsWorld > m_dynamicsWorld;
        
        btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
                     
        boost::shared_ptr< btBroadphaseInterface >  m_broadphase;
        
        boost::shared_ptr< btDefaultMotionState > myMotionState;
        
        boost::shared_ptr< btCollisionDispatcher > m_dispatcher;

        std::deque< boost::shared_ptr< btRigidBody >  > bodies;

        boost::shared_ptr< btCollisionShape > groundShape;
             
        std::list< L3::Visualisers::Leaf* > leafs;
        
        btTransform groundTransform;
   
    };
}
}

#endif

