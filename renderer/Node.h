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
        //pose->x += .50;
        //pose->r += L3::Utils::Math::degreesToRadians( 1 );
       
        L3::SE3* ptr = dynamic_cast<L3::SE3*>( &*pose );
      
        if( ptr )
            ptr->r += L3::Utils::Math::degreesToRadians( 1 );

        pose->_update();
    }

    boost::shared_ptr<L3::Pose> pose;
};


}   // Visualisers
}   // L3

#endif
