#pragma once

#include <vector>
#include "boost/tuple/tuple.hpp"

#include "Utils.h"

/*
 *Helpers
 */
struct randomate 
{ 
  randomate( int MODULO )  : modulo(MODULO)
  {}

  int modulo;

  int operator()( )
  {
    return rand()  % modulo;
  }
};

/*
 *L3
 */
namespace L3
{
  /*
   *Point types
   */
  template< typename T>
    struct Point
    {
      Point() : x(0), y(0), z(0)
      {
      }

      Point( T X, T Y, T Z ) : x(X), y(Y), z(Z)
      {
      }

      T x,y,z;

    };

  template< typename T>
    struct PointE : Point<T>
  {

    T x,y,z;
    T e;

    PointE() : Point<T>(0,0,0),  e(0)
    {
    }


    PointE( T X, T Y, T Z, T E ) : Point<T>(x,y,z), e(E)
    {
    }

  };



  template< typename T>
    struct PointRGB : Point<T>
  {

    T x,y,z;
    T r,g,b;

    PointRGB() : Point<T>(0,0,0),  r(0), g(0), b(0)
    {
    }


    PointRGB( T X, T Y, T Z, T R, T G, T B ) : Point<T>(x,y,z), r(R), g(G), b(B)
    {
    }

  };


  /*
   *Cloud types
   */
  template< typename T>
    struct PointCloud  : Lockable
  {

    PointCloud() : num_points(0), points(NULL)
    {
    }

    size_t num_points;
    L3::Point<T>* points;

    ~PointCloud()
    {
      delete [] points;
    }

    typedef L3::Point<T>* ITERATOR;
    typedef L3::Point<T>* const CONST_ITERATOR;

    ITERATOR begin()
    {
      return points;
    }

    ITERATOR end()
    {
      return (points+num_points);
    }

  };

  template< typename T>
    struct PointCloudE  : Lockable
  {

    PointCloudE() : num_points(0), points(NULL)
    {
    }

    size_t num_points;
    L3::PointE<T>* points;

    ~PointCloudE()
    {
      delete [] points;
    }

    typedef L3::PointE<T>* ITERATOR;
    typedef L3::PointE<T>* const CONST_ITERATOR;

    ITERATOR begin()
    {
      return points;
    }

    ITERATOR end()
    {
      return (points+num_points);
    }

  };

  template <typename T>
    void allocate( PointCloud<T>* cloud, size_t size  );

  template< typename T >
    boost::tuple<T,T,T> mean( PointCloud<T>* cloud );

  template< typename T >
    boost::tuple<T,T,T> mean( PointCloudE<T>* cloud );

  template< typename T >
    boost::tuple<T,T,T> min( PointCloud<T>* cloud );

  template< typename T >
    boost::tuple<T,T,T> max( PointCloud<T>* cloud );

  template <typename T>
    bool join( std::list< boost::shared_ptr<L3::PointCloud<T> > > clouds, boost::shared_ptr<L3::PointCloud<T> >& result  );

  template <typename T>
    bool join( std::list< boost::shared_ptr<L3::PointCloudE<T> > > clouds, boost::shared_ptr<L3::PointCloudE<T> >& result  );

  template <typename T>
    bool sample( PointCloud<T>* input,  PointCloud<T>* output, int size, bool allocate=true );

  template <typename T>
    void centerPointCloud( PointCloud<T>* cloud );

  template <typename T>
    void transform( PointCloud<T>* cloud, L3::SE3 * pose );

  template <typename T>
    void translate( PointCloud<T>* cloud, L3::SE3 const * pose );

  template <typename T>
    std::ostream& operator<<( std::ostream& o, const Point<T>& point );

  template <typename T>
    std::ostream& operator<<( std::ostream& o, const PointRGB<T>& point );

  template <typename T>
    std::ostream& operator<<( std::ostream& o, const PointCloud<T>& cloud );

  template <typename T>
    bool copy( PointCloud<T>* src, PointCloud<T>* dest );

  template <typename T>
    bool copy( PointCloudE<T>* src, PointCloudE<T>* dest );

  template <typename T>
    void gaussianCloud( PointCloud<T>* cloud, double x_variance=10.0, double y_variance=10.0 );

} // L3
