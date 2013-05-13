#ifndef L3_VISUALISERS_BULLET_INTERFACE
#define L3_VISUALISERS_BULLET_INTERFACE

#include "L3.h"

/*
 *Used for ray-query
 */
#include "btBulletDynamicsCommon.h"


namespace L3
{
namespace Visualisers
{

    struct BulletInterface
    {

        BulletInterface()
        {
   
            m_collisionConfiguration.reset( new btDefaultCollisionConfiguration() );

            m_dispatcher.reset( new btCollisionDispatcher(m_collisionConfiguration.get() ) );
       
            m_broadphase.reset( new btDbvtBroadphase() );
         
            sol.reset( new btSequentialImpulseConstraintSolver() );
       
            //m_dynamicsWorld.reset( new btDiscreteDynamicsWorld( m_dispatcher.get(), m_broadphase.get(),  m_solver,m_collisionConfiguration) );
            m_dynamicsWorld.reset( new btDiscreteDynamicsWorld( m_dispatcher.get(), m_broadphase.get(), sol.get(), m_collisionConfiguration.get() ) );
       
             m_dynamicsWorld->setGravity(btVector3(0,-10,0));
       
              
             btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(50.)));
       
             m_collisionShapes.push_back(groundShape);
       
             btTransform groundTransform;
             groundTransform.setIdentity();
             groundTransform.setOrigin(btVector3(0,-50,0));
       
             myMotionState.reset( new btDefaultMotionState(groundTransform));
        }

        boost::shared_ptr< btDefaultCollisionConfiguration > m_collisionConfiguration;

        boost::shared_ptr< btCollisionDispatcher > m_dispatcher;

        boost::shared_ptr< btBroadphaseInterface >  m_broadphase;
   
        boost::shared_ptr< btSequentialImpulseConstraintSolver > sol;
   
        boost::shared_ptr< btDiscreteDynamicsWorld > m_dynamicsWorld;
   
        btAlignedObjectArray<btCollisionShape*> m_collisionShapes;
                     
        boost::shared_ptr< btDefaultMotionState > myMotionState;
    };
}
}

#endif

