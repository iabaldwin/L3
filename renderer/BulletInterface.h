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
             
             // Add the ground
             groundShape.reset( new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.))) );
             m_collisionShapes.push_back(groundShape.get());
      
             // Align it to the world
             groundTransform.setIdentity();
             groundTransform.setOrigin(btVector3(0,-50,0));
             myMotionState.reset( new btDefaultMotionState(groundTransform));
	
             addShape();

             t.begin();
        }

        L3::Timing::ChronoTimer t;

        void onDraw3D( glv::GLV& g )
        {
            btVector3 blue(0,0,1);
            btVector3 pos(0,0,0);
            
            //sDebugDraw.drawSphere(pos,10.1,blue);
            //sDebugDraw.drawTriangle(pos,10.1,blue);
            //sDebugDraw.draw3dText( btVector3(40,40,40), "Duck" );

            
            if (m_dynamicsWorld)
            {
            
                float ms = t.elapsed();
                m_dynamicsWorld->stepSimulation( t.elapsed()/ 1000000.f);
                m_dynamicsWorld->debugDrawWorld();
            }


            doQuery();

            t.begin();
        }

        void addShape()
        {

            btScalar mass(0);

            btVector3 localInertia(0,0,0);

            btRigidBody::btRigidBodyConstructionInfo rbInfo( mass, myMotionState.get(), groundShape.get(), localInertia);
            //btRigidBody* body = new btRigidBody(rbInfo);
            boost::shared_ptr< btRigidBody > body( new btRigidBody(rbInfo) );
            body->setRollingFriction(1);
            body->setFriction(1);
            //add the body to the dynamics world
            m_dynamicsWorld->addRigidBody(body.get());


            bodies.push_back( body );

            btVector3 quad[] = {
                btVector3(0,1,-1),
                btVector3(0,1,1),
                btVector3(0,-1,1),
                btVector3(0,-1,-1)};


            btTriangleMesh* mesh = new btTriangleMesh();
            mesh->addTriangle(quad[0],quad[1],quad[2],true);
            mesh->addTriangle(quad[0],quad[2],quad[3],true);

            btBvhTriangleMeshShape* trimesh = new btBvhTriangleMeshShape(mesh,true,true);

            m_collisionShapes.push_back(trimesh);


            btVector3 convexPoints[]={ btVector3(-1,-1,-1),btVector3(-1,-1,1),btVector3(-1,1,1),btVector3(-1,1,-1),
            btVector3(2,0,0)};


            btCollisionShape* colShapes[5] = {
                new btConvexHullShape(&convexPoints[0].getX(),sizeof(convexPoints)/sizeof(btVector3),sizeof(btVector3)),
                new btSphereShape(1),
                new btCapsuleShape(0.2,1),
                new btCylinderShape(btVector3(0.2,1,0.2)),
                new btBoxShape(btVector3(1,1,1))
            };


            for (int i=0;i<5;i++)
            {
                btTransform startTransform;
                startTransform.setIdentity();
                startTransform.setOrigin(btVector3((i-3)*5,1,0));


                m_collisionShapes.push_back(colShapes[i]);

                btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,0,colShapes[i],localInertia);
                rbInfo.m_startWorldTransform = startTransform;
                btRigidBody* body = new btRigidBody(rbInfo);
                body->setRollingFriction(0.03);
                body->setFriction(1);
            
                m_dynamicsWorld->addRigidBody(body);

            }

        }

        void doQuery()
        {
                static float up = 0.f;
            static float dir = 1.f;

            //add some simple animation
            up+=0.01*dir;

            if (btFabs(up)>2)
            {
                dir*=-1.f;
            }

            btTransform tr = m_dynamicsWorld->getCollisionObjectArray()[1]->getWorldTransform();
            static float angle = 0.f;
            angle+=0.01f;
            tr.setRotation(btQuaternion(btVector3(0,1,0),angle));
            m_dynamicsWorld->getCollisionObjectArray()[1]->setWorldTransform(tr);
   
            m_dynamicsWorld->updateAabbs();
            m_dynamicsWorld->computeOverlappingPairs();

            btVector3 red(1,0,0);
            
            {
                btVector3 from(-30,1+up,0);
                btVector3 to(30,1,0);
                sDebugDraw.drawLine(from,to,btVector4(0,0,0,1));
                btCollisionWorld::AllHitsRayResultCallback allResults(from,to);
                allResults.m_flags |= btTriangleRaycastCallback::kF_KeepUnflippedNormal;
                m_dynamicsWorld->rayTest(from,to,allResults);

                for (int i=0;i<allResults.m_hitFractions.size();i++)
                {
                    btVector3 p = from.lerp(to,allResults.m_hitFractions[i]);
                    sDebugDraw.drawSphere(p,10,red);
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
    };
}
}

#endif

