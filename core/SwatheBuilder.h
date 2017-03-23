#pragma once

#include "Core.h"
#include "Datatypes.h"
#include "Definitions.h"
#include "PoseWindower.h"

namespace L3
{
  class SwatheBuilder : public TemporalObserver
  {

    public:

      SWATHE swathe;

    protected:

      L3::PoseWindower*           pose_windower;
      L3::Iterator<L3::LMS151>*   LIDAR_iterator;
  };

  class RawSwatheBuilder : public SwatheBuilder
  {
    public:
      RawSwatheBuilder(  L3::PoseWindower* windower,  L3::Iterator<L3::LMS151>* iterator )
      {
        this->pose_windower = windower;
        this->LIDAR_iterator = iterator;
      }

      Comparator< std::pair< double, boost::shared_ptr<L3::SE3> > >       pose_comparator;
      Comparator< std::pair< double, boost::shared_ptr<L3::LMS151> > >    LIDAR_comparator;

      bool update( double );

  };

  class BufferedSwatheBuilder : public SwatheBuilder
  {
    public:

      BufferedSwatheBuilder(  L3::PoseWindower* windower,  L3::Iterator<L3::LMS151>* iterator ) :
        previous_update(0.0)
    {
      this->pose_windower = windower;
      this->LIDAR_iterator = iterator;
    }

      Comparator< std::pair< double, boost::shared_ptr<L3::LMS151> > > LIDAR_comparator;

      bool update( double );


    private:

      double previous_update;

      std::deque< std::pair< double, boost::shared_ptr< L3::LMS151 > > > _window_buffer;
  };


}
