#ifndef L3_VISUAL_ADAPTERS_H
#define L3_VISUAL_ADAPTERS_H

#include <glv.h>
#include <glv_binding.h>
#include <glv_util.h>

#include "L3.h"

namespace L3
{
namespace Visualisers
{

class Adapter
{
  public:
    template <typename T>
      static glv::Data Adapt( std::vector<T*> t )
      {
        glv::Data data;

        data.resize( glv::Data::DOUBLE, T::NUM_ELEMENTS, t.size() );

        typename std::vector<T*>::iterator it = t.begin();

        glv::Indexer i(data.size(1));

        while ( it != t.end() )
        {
          std::vector<double>::iterator it2 = (*it)->data.begin();

          while( it2 != (*it)->data.end() )
          {
            data.assign( *it2, i[0], i[1] );
            it2++;
          }

          it++;
        }

        return data;
      }
};

}
}

#endif 
