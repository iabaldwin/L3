#include "Misc.h"
#include <execinfo.h>
#include <cxxabi.h>

namespace L3
{
namespace Misc
{
    std::list <std::string> getDatasetConfigurations()
    {

        char* HOME;
        HOME = getenv ("HOME");
        if (HOME==NULL)
            return std::list<std::string>();

        boost::filesystem::directory_iterator itr( boost::filesystem::path( std::string(HOME) + "/datasets/configuration/missions/" ) );
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

    void print_stack_trace()
    {
        void* callstack[128];
        int i, frames = backtrace(callstack, 128);
        char** strs = backtrace_symbols(callstack, frames);
        for (i = 0; i < frames; ++i) {
            printf("%s\n", strs[i]);
        }
        free(strs);
    }

}
}

