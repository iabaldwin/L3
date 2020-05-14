#pragma once

#include <sstream>
#include <vector>

#include "Core.h"
#include "Datatypes.h"

namespace L3
{
  constexpr int MINIMUM_VALID_TIMESTAMP = 1325376000;

  template <typename T>
    struct MaskPolicy
    {
      virtual boost::shared_ptr<T> operator()(boost::shared_ptr<T> t) {
        return t;
      }
    };

  template <>
    struct MaskPolicy<L3::LMS151>
    {
      MaskPolicy(float cull = 4) : cull(cull) {
      }

      float cull{-1};;

      virtual boost::shared_ptr<L3::LMS151> operator()( boost::shared_ptr<L3::LMS151> t )
      {
        for( std::vector<float>::iterator it = t->ranges.begin();
            it != t->ranges.end();
            it++ )
        {
          if ( *it < this->cull )
            *it = 0.0;
        }

        return t;
      }
    };

  template <typename T>
    class AbstractFactory
    {
      public:
        static std::pair< double, boost::shared_ptr<T> > produce( std::string& str, MaskPolicy<T>* policy = NULL)
        {
          std::stringstream ss( str );
          std::vector< double > elements;

          double tmp;

          while ( ss >> tmp )
            elements.push_back( tmp );

          return AbstractFactory<T>::produce(elements, policy);
        }

        static std::pair< double, boost::shared_ptr<T> > produce( std::vector<double> elements, MaskPolicy<T>* mask_policy = NULL)
        {
          double time = elements.at(0);
          CHECK_GE(time, MINIMUM_VALID_TIMESTAMP) << time;
          elements.erase( elements.begin() );
          if( mask_policy )
            return std::make_pair(time, (*mask_policy)( boost::make_shared<T>( elements ) ) );
          else
            return std::make_pair(time, boost::make_shared<T>( elements ) );
        }
    };

  template <>
    class AbstractFactory<L3::Pose>
    {
      public:
        static std::pair< double, boost::shared_ptr<L3::Pose> >  produce( std::string& str )
        {
          std::stringstream ss( str );
          std::vector< double > elements;

          double tmp;

          while ( ss >> tmp )
            elements.push_back( tmp );

          return AbstractFactory<L3::Pose>::produce( elements );
        }

        static std::pair< double, boost::shared_ptr<L3::Pose> > produce( std::vector<double> elements )
        {
          CHECK( elements.size() > 0 );

          double time = elements[0];
          elements.erase( elements.begin() );

          switch (elements.size())
          {
            case 3:
              return std::make_pair( time, boost::make_shared<L3::SE2>( elements ) );

            case 6:
              return std::make_pair( time, boost::make_shared<L3::SE3>( elements ) );

            default:
              throw std::exception();
          }
        }
    };

} // L3
