#ifndef L3_VISUALISERS_TEXT_H
#define L3_VISUALISERS_TEXT_H

#include <GLV/glv.h>
#include <OGLFT/OGLFT.h> // Note: this will depend on where you've installed OGLFT

#include "Components.h"

namespace L3
{
namespace Visualisers
{

struct Text3D : Leaf
{
    Text3D();
    void onDraw3D( glv::GLV& g );

//OGLFT::Monochrome* face;
OGLFT::Texture* face;

};

}
}
#endif

