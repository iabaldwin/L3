#ifndef L3_ABSTRACT_FACTORY
#define L3_ABSTRACT_FACTORY

#include <sstream>
#include <vector>

#include "Core.h"
#include "Datatypes.h"

namespace L3
{

template <typename T>
struct MaskPolicy
{
    boost::shared_ptr<T> operator()( boost::shared_ptr<T> t ) 
    {
        return t;
    }

};
 
template <>
struct MaskPolicy< L3::LMS151 >
{
    MaskPolicy( float cull = 5 ) : cull(cull)
    {

    }

    float cull;

    boost::shared_ptr<L3::LMS151> operator()( boost::shared_ptr<L3::LMS151> t ) ;
    

};
    

template <typename T>
class AbstractFactory
{
    public:
        static std::pair< double, boost::shared_ptr<T> > produce( std::string& str, MaskPolicy<T> policy = MaskPolicy<T>() );
        static std::pair< double, boost::shared_ptr<T> > produce( std::vector<double> elements, MaskPolicy<T> policy = MaskPolicy<T>());
};

//template <>
//class AbstractFactory<L3::SE3>
//{
    //public:
        //static std::pair< double, boost::shared_ptr<L3::SE3> > produce( std::string& str );
        //static std::pair< double, boost::shared_ptr<L3::SE3> > produce( std::vector<double> elements );
//};

//template <>
//class AbstractFactory<L3::LIDAR>
//{
    //public:
        //static std::pair< double, boost::shared_ptr<L3::LIDAR> > produce( std::string& str );
        //static std::pair< double, boost::shared_ptr<L3::LIDAR> > produce( std::vector<double> elements );
//};

template <>
class AbstractFactory<L3::Pose>
{
    public:
        static std::pair< double, boost::shared_ptr<L3::Pose> > produce( std::string& str );
        static std::pair< double, boost::shared_ptr<L3::Pose> > produce( std::vector<double> elements );
};


}
#endif
