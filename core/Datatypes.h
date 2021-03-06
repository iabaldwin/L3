#pragma once

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <list>
#include <iterator>

#include <Eigen/Core>

namespace L3
{

  class Pose
  {
    public:

      virtual ~Pose(){}

      Eigen::Matrix4f& getHomogeneous(){ return homogeneous; };
      Pose& setHomogeneous(Eigen::Matrix4f homogeneous){ this->homogeneous = homogeneous; return *this; };

    protected:

      virtual void updateHomogeneous()  = 0;

      Eigen::Matrix4f homogeneous;
  };

  class SE2 : public Pose
  {

    public:

      SE2(double X, double Y, double Q);
      SE2(std::vector<double> input);

      double& X()
      {
        return x_;
      }

      void X(double x)
      {
        x_ = x;
      }

      double& Y()
      {
        return y_;
      }

      void Y(double y)
      {
        y_ = y;
      }


    private:

      double x_,y_,q_;

      void updateHomogeneous();
  };

  class SE3 : public Pose
  {

    public:

      static SE3 ZERO();
      static SE3 UNIT_X();
      static SE3 UNIT_Y();
      static SE3 UNIT_Z();

      SE3();
      SE3(double X, double Y, double Z, double R, double P, double Q);
      SE3(const std::vector<double> v);

      double  X() const;
      void    X(double x);
      double  Y() const;
      void    Y(double y);
      double  Z() const;
      void    Z(double z);
      double  R() const;
      void    R(double r);
      double  P() const;
      void    P(double P);
      double  Q() const;
      void    Q(double q);


    private:

      double x,y,z;
      double r,p,q;

      void updateHomogeneous();

  };

  bool operator==(const L3::SE3& lhs,  const L3::SE3& rhs);

  struct LHLV
  {
    typedef std::vector<double>::iterator ITERATOR;

    LHLV(const std::vector<double> v)
    {
      data.assign(v.begin(), v.end());
    }

    std::vector< double > data;
  };

  struct LIDAR
  {
    std::vector<float> ranges;
    std::vector<float> reflectances;

    LIDAR(double start, double end, int scans) :
      angle_start(start), angle_end(end), num_scans(scans)
    {}

    double angle_start{-1};
    double angle_end{-1};
    double angle_spacing{-1};
    int num_scans{-1};
  };

  struct LMS151 : LIDAR
  {
    LMS151() : LIDAR(-1*M_PI/4, M_PI+(M_PI/4), 541)
    {
      angle_spacing = (angle_end - angle_start) / (double)num_scans;
      ranges.resize(num_scans, 0);
      reflectances.resize(num_scans, 0);
    }

    LMS151(std::vector<double> vec)  : LIDAR(-1*M_PI/4, M_PI+(M_PI/4), 541)
    {
      angle_spacing = (angle_end - angle_start) / (double)num_scans;
      ranges.resize(num_scans, -1);
      ranges.assign(vec.begin(), vec.begin()+(541));
      reflectances.resize(num_scans, -1);
      reflectances.assign(vec.begin()+(541+1), vec.end());
    }

    void print(std::ostream& o) const {
      o << angle_start << ":" << angle_end << ":" << angle_spacing;
      o << std::endl;
      std::copy(ranges.begin(), ranges.end(), std::ostream_iterator<double>(o, " "));
      o << std::endl;
      std::copy(reflectances.begin(), reflectances.end(), std::ostream_iterator<double>(o, " "));
      o << std::endl;
    }
  };

  struct SMVelocity
  {
    typedef std::vector<double>::iterator ITERATOR;

    SMVelocity(const std::vector<double> v)
    {
      data.assign(v.begin(), v.end());
    }

    std::vector< double > data;
  };

  template <typename T>
    struct Sizes
    {
      const static int elements = 0;
    };

  template <>
    struct Sizes<L3::SE3>
    {
      const static int elements = 6+1;
    };

  template <>
    struct Sizes<L3::LMS151>
    {
      const static int elements = (2*541)+1;
    };

  template <>
    struct Sizes<L3::LHLV>
    {
      const static int elements = 11+1;
    };

  template <>
    struct Sizes<L3::SMVelocity>
    {
      const static int elements = 4+1;
    };

  /*
   *  Frequencies
   */
  template <typename T>
    struct Frequency
    {
      const static int frequency = 0;
    };

  /*
   *  I/O
   */
  std::ostream& operator<<(std::ostream& o, const L3::SE2& pose);
  std::ostream& operator<<(std::ostream& o, const L3::SE3& pose);
  std::ostream& operator<<(std::ostream& o, const L3::LIDAR& scan);
  std::ostream& operator<<(std::ostream& o, const L3::LMS151& scan);
  std::ostream& operator<<(std::ostream& o, const L3::LHLV& lhlv);

  /*
   *  Intrinsic math
   */
  namespace Math
  {
    double norm(const L3::SE3& a, const L3::SE3& b);
    double SE2Metric(const L3::SE3& a, const L3::SE3& b);

  } //Math
} //L3
