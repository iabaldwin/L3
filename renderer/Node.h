#ifndef L3_VISUALISATION_NODE_H
#define L3_VISUALISATION_NODE_H

namespace L3
{
namespace Visualisers
{

struct Node
{

    virtual void update() = 0;

};

struct Controllable : Node
{

    Controllable( boost::shared_ptr< L3::Pose > POSE )  : pose(POSE)
    {

    }
   
    void update()
    {
        L3::SE3* ptr = dynamic_cast<L3::SE3*>( &*pose );
      
        if( ptr )
        {
            double increment = ptr->R();
            increment += L3::Utils::Math::degreesToRadians( 1 );
           
            //ptr->getHomogeneous()*=L3::SE3( 0, 0, 0, increment, 0, 0 ).getHomogeneous();

            Eigen::Matrix4f tmp( ptr->getHomogeneous() ); 

            ptr->setHomogeneous( L3::SE3( 0, 0, 0, increment, 0, 0 ).getHomogeneous()*=tmp );

            

            //ptr->R( increment );
        }
    }

    boost::shared_ptr<L3::Pose> pose;
};


}   // Visualisers
}   // L3

#endif
