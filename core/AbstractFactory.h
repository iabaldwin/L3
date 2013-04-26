#ifndef L3_ABSTRACT_FACTORY
#define L3_ABSTRACT_FACTORY

#include <sstream>
#include <vector>

#include "Core.h"
#include "Datatypes.h"

namespace L3
{

template <typename T>
class AbstractFactory
{
    public:
        static std::pair< double, boost::shared_ptr<T> > produce( std::string& str );
        static std::pair< double, boost::shared_ptr<T> > produce( std::vector<double> elements );
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
