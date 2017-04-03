#include "ViewController.h"

namespace L3
{
namespace Visualisers
{
  control_t::control_t() : x(0), y(0), z(0), r(0), p(0), q(0)
  {
    updateHomogeneous();
  }

  control_t::control_t( float x, float y, float z, float r, float p, float q ) : x(x), y(y), z(z), r(r), p(p), q(q)
  {
    updateHomogeneous();
  }

  control_t& control_t::updateEuler()
  {
    return *this;
  }

  control_t& control_t::translateZ( float z )
  {
    this->z = z;
    this->updateHomogeneous();
    return *this;
  }

  control_t& control_t::updateHomogeneous()
  {
    double _q = L3::Utils::Math::degreesToRadians( q );
    Eigen::Matrix4f Rz = Eigen::Matrix4f::Identity();
    Rz(0,0) = cos( _q );
    Rz(0,1) = -1*sin( _q );
    Rz(1,0) = sin( _q );
    Rz(1,1) = cos( _q );

    double _p = L3::Utils::Math::degreesToRadians( p );
    Eigen::Matrix4f Rx = Eigen::Matrix4f::Identity();
    Rx(1,1) = cos( _p );
    Rx(1,2) = -1*sin( _p );
    Rx(2,1) = sin( _p );
    Rx(2,2) = cos( _p );

    double _r = L3::Utils::Math::degreesToRadians( r );
    Eigen::Matrix4f Ry = Eigen::Matrix4f::Identity();
    Ry(0,0) = cos( _r );
    Ry(0,2) = sin( _r );
    Ry(2,0) = -1*sin( _r );
    Ry(2,2) = cos( _r );

    homogeneous = Rz*Ry*Rx;

    // SE3
    homogeneous(0,3) = x;
    homogeneous(1,3) = y;
    homogeneous(2,3) = z;

    return *this;
  }

  bool CompositeController::MouseDragController::onEvent(glv::View&, glv::GLV& g)
  {
    if ( g.keyboard().shift() )
    {
      double x = (double)(g.mouse().x() - origin_x) /100;
      double y = (double)(g.mouse().y() - origin_y) /100;

      current.updateHomogeneous();

      Eigen::Matrix4f mat = current.homogeneous;

      mat(0,3) = 0.0;
      mat(1,3) = 0.0;
      mat(2,3) = 0.0;

      Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();

      trans(0,3) = x;
      trans(1,3) = -1*y;

      Eigen::Matrix4f res = mat.inverse() * trans;

      current.x += res(0,3);
      current.y += res(1,3);

    }
    else
    {
      current.q += (double)(g.mouse().x() - origin_x) /100;

      double r = (double)(g.mouse().y() - origin_y) /100;

      if ( ( current.r > -65 && r < 0 ) || ( current.r < 65 && r > 0 ) )
        current.r += r;
    }

    //TODO
    return true;
  }

}
}
