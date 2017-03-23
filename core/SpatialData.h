#pragma once

#include <map>
#include <list>
#include <deque>
#include <fstream>
#include <iostream>

#include "Core.h"
#include "Datatypes.h"
#include "PointCloud.h"

#include <Poco/Thread.h>

namespace L3
{
  struct spatial_data
  {
    int id;
    double x,y;
    unsigned int stream_position;
    unsigned int payload_size;
  };

  struct SelectionPolicy
  {
    virtual bool operator()( std::deque< spatial_data >* sections, double x, double y, std::list<unsigned int>& required_sections, const int window ) 
    {
      return false;
    }
  };

  struct KNNPolicy : SelectionPolicy
  {
    bool operator()( std::deque< spatial_data>* sections, double x, double y, std::list<unsigned int>& required_sections, const int window );
  };

  struct StrictlyRetrospectivePolicy : SelectionPolicy
  {
    bool operator()( std::deque< spatial_data>* sections, double x, double y, std::list<unsigned int>& required_sections, const int window );
  };

  struct RetrospectiveWithLookaheadPolicy: SelectionPolicy
  {
    bool operator()( std::deque< spatial_data>* sections, double x, double y, std::list<unsigned int>& required_sections, const int window );
  };

  struct SpatialQuery : SpatialObserver, Poco::Runnable, Lockable
  {
    SpatialQuery( std::deque< spatial_data > sections, 
        std::string fname, 
        boost::shared_ptr< SelectionPolicy > policy, 
        int window_size = 2 ) :
      sections(sections), 
      policy(policy),
      window(window_size),
      running(true),
      _x(0.0), _y(0.0)
    {
    }

    std::ifstream                           data;
    Poco::Thread                            thread;
    std::deque< spatial_data >              sections;
    boost::shared_ptr< SelectionPolicy >    policy; 

    int                                     window;
    bool                                    running;
    double                                  _x,_y;


    bool update( double x, double y )
    {
      _x = x;
      _y = y;
      return true;
    }

  };
}
