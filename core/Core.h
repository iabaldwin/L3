#ifndef L3_CORE_H
#define L3_CORE_H

namespace L3
{

struct Observer
{
    virtual bool update( double ) = 0;
};

}

#endif
