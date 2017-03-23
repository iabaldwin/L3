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
    /**
     * @brief A masking policy for incoming data
     *
     * @param t SharedPtr to data
     *
     * @return SharedPtr to masked data
     */
    virtual boost::shared_ptr<T> operator()(boost::shared_ptr<T> t) 
    {
        return t;
    }

};
 
template <>
struct MaskPolicy<L3::LMS151>
{
    MaskPolicy(float cull = 4) : cull(cull) {
    }

    float cull;

    virtual boost::shared_ptr<L3::LMS151> operator()(boost::shared_ptr<L3::LMS151> t) ;

};
 
template <typename T> 
struct RoadMaskPolicy : MaskPolicy<T> 
{
    RoadMaskPolicy() {
        MaskPolicy<T>::cull = 0.0;
    }

};

template <typename T>
class AbstractFactory
{
    public:
        static std::pair<double, boost::shared_ptr<T>> produce(std::string& str, MaskPolicy<T>* policy = NULL);
        static std::pair<double, boost::shared_ptr<T>> produce(std::vector<double> elements, MaskPolicy<T>* policy = NULL);

};

template <>
class AbstractFactory<L3::Pose>
{
    public:
        static std::pair<double, boost::shared_ptr<L3::Pose>> produce(std::string& str);
        static std::pair<double, boost::shared_ptr<L3::Pose>> produce(std::vector<double> elements);
};

}

#endif
