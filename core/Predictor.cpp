#include "Predictor.h"
#include "Core.h"

#include <Eigen/LU>
#include <deque>

namespace L3
{
  bool Predictor::update(double t) {
    return true;
  }

  bool Predictor::predict(const L3::SE3& current) {
    return true;
  }
}

