#ifndef L3_ABSTRACT_FACTORY
#define L3_ABSTRACT_FACTORY

#include <sstream>
#include <boost/shared_ptr.hpp>
#include "Datatypes.h"

namespace L3
{

template <typename T>
class AbstractFactory
{
    public:
        static std::pair< double, boost::shared_ptr<T> > fromString( std::string& str )
        {
            std::stringstream ss( str );
            std::vector< double > elements;
            
            double tmp;
            
            while ( ss >> tmp )
            {
                elements.push_back( tmp ); 
            }

            return std::make_pair( elements[0], boost::shared_ptr<T>( new T( elements ) ) );

        }
};

template <>
class AbstractFactory<L3::LIDAR>
{
    public:
        static std::pair< double, boost::shared_ptr<L3::LIDAR> > fromString( std::string& str )
        {
            std::stringstream ss( str );
            std::vector< double > elements;

            double tmp;
            
            while ( ss >> tmp )
            {
                elements.push_back( tmp ); 
            }
            
            return std::make_pair( elements[0], boost::shared_ptr<L3::LIDAR>( new LMS151( elements ) ) );
        }
};

template <>
class AbstractFactory<L3::Pose>
{
    public:
        static std::pair< double, boost::shared_ptr<L3::Pose> > fromString( std::string& str )
        {
            std::stringstream ss( str );
            std::vector< double > elements;
            
            double tmp;
            
            while ( ss >> tmp )
            {
                elements.push_back( tmp ); 
            }

            switch (elements.size())
            {
                case 4:
                    return std::make_pair( elements[0], boost::shared_ptr<L3::SE2>( new L3::SE2( elements ) ) );

                case 7:
                    return std::make_pair( elements[0], boost::shared_ptr<L3::SE3>( new L3::SE3( elements ) ) );
           
                default:
                    std::copy( elements.begin(), 
                                elements.end(), 
                                std::ostream_iterator<double>( std::cout, " " ) );
                    throw std::exception();
            }
        
        }
};


}
#endif
