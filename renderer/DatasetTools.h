#pragma once

#include <glv.h>

#include "L3.h"

namespace L3
{
  namespace Visualisers
  {

    struct ExperienceBuilder : glv::EventHandler
    {
      ExperienceBuilder( boost::shared_ptr< L3::Dataset > dataset, double& time ) 
        : dataset(dataset),
        time(time),
        counter(0)
      {

      }

      int counter;
      double experience_start, experience_end;

      double& time;
      boost::weak_ptr< L3::Dataset > dataset;

      bool onEvent( glv::View& view , glv::GLV& g) 
      {
        const glv::Keyboard& k = g.keyboard();
        int key = k.key();

        if ( key == 32 )
        {
          boost::shared_ptr< L3::Dataset > dataset_ptr = dataset.lock();

          if( dataset_ptr )
          {
            counter++; 

            if( counter == 1 )
            {
              experience_start =  (time - dataset_ptr->start_time );
            }
            else if( counter == 2 )
            {
              experience_end =  (time - dataset_ptr->start_time );

              std::cout << experience_start << ":" << experience_end << std::endl;

              L3::ExperienceBuilder( *dataset_ptr, experience_start, experience_end );
            }
            else
              counter =0;
          }
        }
        return false;
      }
    };

  } // Visualisers
} // L3
