#ifndef L3_ABSTRACT_FACTORY
#define L3_ABSTRACT_FACTORY

#include <sstream>
#include "Datatypes.h"

namespace L3
{

template <typename T>
class AbstractFactory
{

    public:
        static T* fromString( std::string& str )
        {
            std::stringstream ss( str );
            std::vector< double > elements;
            
            double tmp;
            
            while ( ss >> tmp )
            {
                elements.push_back( tmp ); 
            }

            return new T( elements );
        }
};

template <>
class AbstractFactory<L3::LIDAR>
{

    public:
        static L3::LIDAR* fromString( std::string& str )
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
                case 541:
                    return new L3::LMS151( elements );

                case 542:
                    elements.erase( elements.begin() );
                    return new L3::LMS151( elements );

                default:
                    throw std::exception();
            }
        
        }
};



template <>
class AbstractFactory<L3::Pose>
{

    public:
        static L3::Pose* fromString( std::string& str )
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
                    return new L3::SE2( elements );

                case 6:
                    return new L3::SE3( elements );

                case 7:
                    elements.erase( elements.begin() );
                    return new L3::SE3( elements );
           
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
