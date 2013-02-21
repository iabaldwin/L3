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
        static std::pair< double, boost::shared_ptr<T> > produce( std::string& str )
        {
            std::stringstream ss( str );
            std::vector< double > elements;
            
            double tmp;
            
            while ( ss >> tmp )
            {
                elements.push_back( tmp ); 
            }

            return AbstractFactory<T>::produce( elements );

        }
        
        static std::pair< double, boost::shared_ptr<T> > produce( std::vector<double>& elements )
        {
            return std::make_pair( elements[0], boost::shared_ptr<T>( new T( elements ) ) );
        }
};

template <>
class AbstractFactory<L3::LIDAR>
{
    public:
        static std::pair< double, boost::shared_ptr<L3::LIDAR> > produce( std::string& str )
        {
            std::stringstream ss( str );
            std::vector< double > elements;

            double tmp;
            
            while ( ss >> tmp )
            {
                elements.push_back( tmp ); 
            }
            
            return AbstractFactory<L3::LIDAR>::produce( elements );
        }

        static std::pair< double, boost::shared_ptr<L3::LIDAR> > produce( std::vector<double>& elements )
        {
            assert( elements.size() > 0 );

            double time = elements[0];
            elements.erase( elements.begin() );
            switch (elements.size())
            {
                case 541:
                    return std::make_pair( time, boost::shared_ptr<L3::LIDAR>( new LMS151( elements ) ) );

                default:
                    throw std::exception();

            }

        }

};

template <>
class AbstractFactory<L3::Pose>
{
    public:
        static std::pair< double, boost::shared_ptr<L3::Pose> > produce( std::string& str )
        {
            std::stringstream ss( str );
            std::vector< double > elements;
            
            double tmp;
            
            while ( ss >> tmp )
            {
                elements.push_back( tmp ); 
            }

            return AbstractFactory<L3::Pose>::produce( elements );
        }

        static std::pair< double, boost::shared_ptr<L3::Pose> > produce( std::vector<double>& elements )
        {
            assert( elements.size() > 0 );
            
            double time = elements[0];
            elements.erase( elements.begin() );

            switch (elements.size())
            {
                case 3:
                    return std::make_pair( time, boost::shared_ptr<L3::SE2>( new L3::SE2( elements ) ) );

                case 6:
                    return std::make_pair( time, boost::shared_ptr<L3::SE3>( new L3::SE3( elements ) ) );

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
