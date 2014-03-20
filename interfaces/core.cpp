#include <L3.h>
#include <Core.h>

#include <boost/python.hpp>

void run(void)
{
    boost::shared_ptr< L3::Dataset > dataset( new L3::Dataset(""));
}

BOOST_PYTHON_MODULE(Core)
{
    using namespace boost::python;
    def("run", run);
}
