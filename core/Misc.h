#pragma once

#include <iostream>
#include <boost/filesystem.hpp>

namespace L3
{
namespace Misc
{

/*
 *  Configuration
 */
    
std::list <std::string> getDatasetConfigurations();

/*
 *  Code related
 */
const char* demangle(const char* name);

void print_stack_trace();

}
}
