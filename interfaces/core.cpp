#include <L3.h>
#include <Core.h>
#include <Windower.h>
#include <Iterator.h>

#include <boost/python.hpp>
#include <boost/noncopyable.hpp>

boost::shared_ptr<int> run()
{
    return boost::shared_ptr<int>( new int(10) );
}

BOOST_PYTHON_MODULE(Core)
{
    using namespace boost::python;
  
    def( "run", run );

    //class_<L3::Dataset, boost::noncopyable>( "Dataset", init<std::string>() )
        //.def( "validate", &L3::Dataset::validate )
        //.def( "load", &L3::Dataset::load )
        //.def( "path", &L3::Dataset::path )
        //.def( "name", &L3::Dataset::name );

    class_<L3::SlidingWindow<L3::SE3>, boost::noncopyable, boost::shared_ptr< L3::SlidingWindow<L3::SE3> > >( "SlidingWindow",init<std::string, float>()  );

    //class_< L3::ConstantTimeIterator<L3::SE3>, boost::shared_ptr< L3::ConstantTimeIterator<L3::SE3> >, boost::shared_ptr< L3::SlidingWindow<L3::SE3> > >( "ConstantTimeIterator", init< boost::shared_ptr< L3::SlidingWindow<L3::SE3> > >());
    //class_< L3::ConstantTimeIterator<double> >( "ConstantTimeIterator" )
        //.def( init< boost::shared_ptr< L3::SlidingWindow<double> >, int>() );
}
