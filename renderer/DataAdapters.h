#ifndef L3_VISUAL_ADAPTERS_H
#define L3_VISUAL_ADAPTERS_H

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include "L3.h"

namespace L3
{
namespace Visualisers
{


class Adapter
{

    static glv::Data Adapt( std::vector<L3::LHLV*> lhlv )
    {
    }


    static glv::Data Adapt( L3::LHLV* lhlv )
    {
    }

};

}
}

#endif 
