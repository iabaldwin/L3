#include "QueryInterface.h"

namespace L3
{
    namespace Visualisers
    {
        void QueryInterface::onDraw3D( glv::GLV& g )
        {
            // This is only for debugging
            glv::Point3 vertices[2];
            glv::Color  colors[2];

            vertices[0]( from[0], from[1], from[2] );
            vertices[1]( to[0], to[1], to[2] );

            //glv::draw::lineStippling(true);
            //glv::draw::lineWidth(2);
            //glv::draw::lineStipple(4, 0xAAAA );
            //glv::draw::paint( glv::draw::Lines, vertices, colors, 2 );
            //glv::draw::lineStippling(false);

            //sDebugDraw.drawLine(from,to,btVector4(0,0,0,1));

            //btVector3 red(1,0,0);
            //for( std::deque< btVector3 >::iterator it = hits.begin(); it != hits.end(); it++ )
                //sDebugDraw.drawSphere( *it,1,red);

        }


        void QueryInterface::query( double x1, double x2, 
                                    double y1, double y2,
                                    double z1, double z2,
                                    std::list<const btCollisionObject*>& hit_results )
        {
            std::cout << x1 << ":" << x2 << ":" << y1 << ":" << y2 << ":" <<  z1 << ":" << z1 << std::endl;
            // Update all axis-aligned bounding boxes
            m_dynamicsWorld->updateAabbs();
            m_dynamicsWorld->computeOverlappingPairs();

            hit_results.clear();

            // Create query
            from = btVector3(x1, y1, z1);
            to = btVector3( x2, y2, z2 );

            btCollisionWorld::AllHitsRayResultCallback allResults(from,to);

            allResults.m_flags |= btTriangleRaycastCallback::kF_KeepUnflippedNormal;
           
            // Query
            m_dynamicsWorld->rayTest(from,to,allResults);

            // Dbg
            hits.clear();

            for (int i=0;i<allResults.m_hitFractions.size();i++)
            {
                btVector3 p = from.lerp(to,allResults.m_hitFractions[i]);

                hits.push_back( p );

                hit_results.push_back( allResults.m_collisionObjects[i] );
            }
        }

    }
}
