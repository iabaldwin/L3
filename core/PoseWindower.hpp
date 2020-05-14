#pragma once

#include <iterator> 
#include <deque> 

namespace L3
{
  template <typename InputIterator >
    bool Inverter::invert(InputIterator start, InputIterator end) {
      if (std::distance(start, end) == 0) {
        return false;
      }

      // Destructive resize 
      chain.assign(start, end);

      // End point
      Eigen::Matrix4f last_pose = chain.back().second->getHomogeneous(); 

      Eigen::Matrix4f delta(last_pose.inverse());

      // Transform
      std::deque< std::pair< double, boost::shared_ptr<L3::SE3> > >::iterator it = chain.begin();

      while(it != chain.end()) {
        Eigen::Matrix4f tmp(it->second->getHomogeneous()); 
        it->second->setHomogeneous(delta * tmp);
        it++;
      }
      return true;
    }
}
