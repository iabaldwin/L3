#include "Performance.h"
#include "Datatypes.h"

namespace L3
{
  void RelativeDisplacement::update() {
    double current = timer.elapsed();

    if ((current - previous_update) > 1/update_frequency) {
      boost::shared_ptr< SE3 > pose_ptr = pose.lock();

      if(pose_ptr) {
        displacement = L3::Math::SE2Metric(*pose_ptr, experience->getClosestPose(*pose_ptr));
      }
      previous_update = current;
    }
  }
}
