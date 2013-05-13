#ifndef L3_VISUALISERS_TEXT_H
#define L3_VISUALISERS_TEXT_H

#include <iostream>
#include "libglf/glf.h"

#include <boost/filesystem.hpp>
#include <boost/shared_array.hpp>

#include "Components.h"

namespace L3
{
namespace Visualisers
{

struct Text3D : Leaf
{
    Text3D();

    void setText( std::string text );

    std::string text;

    int font_descriptor;
    
    void onDraw3D( glv::GLV& g );

};

struct LeafLabel : Text3D
{

    LeafLabel( Leaf* leaf ) : leaf(leaf)
    {
    }

    void onDraw3D( glv::GLV& g )
    {
        glPushMatrix();

        glRotatef( 90, 1, 0, 0 );

        glTranslatef( 0, 20, 0 );

        Text3D::onDraw3D( g );

        glPopMatrix();
    }

    Leaf* leaf;
};

}
}
#endif

