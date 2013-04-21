#include "Misc.h"

#include <cxxabi.h>

namespace L3
{
namespace Misc
{

    std::list <std::string> getDatasetConfigurations()
    {
        boost::filesystem::directory_iterator itr( boost::filesystem::path( "/Users/ian/code/datasets/configuration/missions/" ) );
        std::list <std::string> datasets;

        while( itr != boost::filesystem::directory_iterator() )
        {
            if ( isdigit( itr->path().leaf().string()[0] ))
            {
                datasets.push_front( itr->path().string() );
            }

            itr++;
        }

        return datasets;

    }

    const char* demangle(const char* name)
    {
        char buf[1024];
        size_t size =1024;
        int status;
        char* res = abi::__cxa_demangle (name,
                buf,
                &size,
                &status);
        return res;
    }



}
}

