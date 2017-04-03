#include "Utils.h"

namespace L3
{
namespace Utils
{
namespace Math
{

  double degreesToRadians(double degrees)
  {
    return (M_PI/180.0)*degrees;
  }

  double radiansToDegrees(double radians)
  {
    return (180.0/M_PI)*radians;
  }


  L3::SE3 poseFromRotation(const Eigen::Matrix4f& mat)
  {
    double x = mat(0, 3);
    double y = mat(1, 3);
    double z = mat(2, 3);

    double q = atan2(mat(1,0), mat(0, 0));
    double r = atan2(-1* mat(2, 0), sqrt(pow(mat(2,1),2)+ pow(mat(2,2) ,2)));
    double p = atan2(mat(2,1), mat(2,2));

    return L3::SE3(x,y,z,r,p,q);
  }

} // namespace Math
} // namespace Utils
} // namespace L3
