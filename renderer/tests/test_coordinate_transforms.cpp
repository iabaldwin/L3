#include <iostream>

#include <glv.h>
#include <glv_binding.h>
#include <glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "Components.h"

struct test_leaf : L3::Visualisers::Leaf
{
    test_leaf()
    {
        x = 20;
        y= 20;
        z = 0;
        r = 0;
        p = 0;
        q = M_PI/4.0;
    }

    float x,y,z,r,p,q;

    void onDraw3D(glv::GLV& g)
    {
       
        // Random pose
        L3::SE3 pose( x,y,z,r,p,q );
        L3::Visualisers::CoordinateSystem( pose, 10 ).onDraw3D(g); 

        Eigen::Matrix4f delta = Eigen::Matrix4f::Identity();

        delta(0,3) = 20;

        pose.getHomogeneous() *= delta;
        L3::Visualisers::CoordinateSystem( pose, 10 ).onDraw3D(g); 

    }
};

int main (int argc, char ** argv)
{

    glv::GLV top;
    glv::Window win(1400, 800, "Soaring");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    L3::Visualisers::Composite          composite;
    L3::Visualisers::Grid               grid;
    L3::Visualisers::BasicPanController controller(composite.position);

    test_leaf leaf;
    
    top << ( composite << leaf << grid );

    composite.stretch(1,1);

    win.setGLV(top);
    glv::Application::run();
}


