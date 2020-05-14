#pragma once

#include <iostream>
#include <boost/filesystem.hpp>

namespace L3
{
  namespace Misc
  {
    std::list <std::string> getDatasetConfigurations();

    const char* demangle(const char* name);

    void print_stack_trace();

  } // Misc
} // L3
