#include "SwatheBuilder.h"

namespace L3
{
  bool RawSwatheBuilder::update(double) {
    //Rebuild swathe completely
    swathe.clear();

    if(LIDAR_iterator->window.size() < pose_windower->window->size()) {
      L3::Iterator<L3::LMS151>::WINDOW_ITERATOR it = LIDAR_iterator->window.begin() ;

      //For each lidar scan, find the nearest pose
      while(it != LIDAR_iterator->window.end()) {
        // Nearest time
        L3::Iterator<L3::SE3>::WINDOW_ITERATOR index = std::lower_bound(pose_windower->window->begin(),
            pose_windower->window->end(),
            it->first,
            pose_comparator);

        if (index == pose_windower->window->end())  {
          break;
        }

        if ((index->first - it->first) == 0) {
          swathe.push_back(std::make_pair(index->second, it->second));
        }
        it++;
      }
    } else {
      L3::Iterator<L3::SE3>::WINDOW_ITERATOR it = pose_windower->window->begin() ;

      // For each lidar pose, find the nearest scan
      while(it != pose_windower->window->end()) {
        // Nearest time
        L3::Iterator<L3::LMS151>::WINDOW_ITERATOR index = std::lower_bound(LIDAR_iterator->window.begin(),
            LIDAR_iterator->window.end(),
            it->first,
            LIDAR_comparator);

        if ((index->first - it->first) == 0) {
          swathe.push_back(std::make_pair(it->second, index->second));
        }
        it++;
      }
    }
    return true;
  }

  bool BufferedSwatheBuilder::update(double) {
    if(LIDAR_iterator->window.empty() || pose_windower->window->empty()) {
      return false;
    }

    // Rebuild swathe completely
    swathe.clear();

#ifndef NDEBUG
    LOG(INFO) << LIDAR_iterator->window.size() << ":" << pose_windower->window->size();
#endif

    // Find the new data, between the last update time and now
    L3::Iterator<L3::LMS151>::WINDOW_ITERATOR index = std::lower_bound(LIDAR_iterator->window.begin(),
        LIDAR_iterator->window.end(),
        previous_update,
        LIDAR_comparator);

    if(index->first == previous_update) {
      index++;
    }

    _window_buffer.insert(_window_buffer.end(), index, LIDAR_iterator->window.end());

#ifndef NDEBUG
    for(std::deque< std::pair< double, boost::shared_ptr< L3::LMS151 > > >::iterator it = (_window_buffer.begin()+1);
        it != _window_buffer.end();
        it++) {
      double previous = (it-1)->first;
      double current = it->first;

      if((current-previous) > .2)  {
        LOG(ERROR) << "BIG gap";
        exit(EXIT_FAILURE);
      }
      if ((current-previous) < .001) {
        LOG(WARNING) << "SMALL gap";
        LOG(WARNING) << current-previous;
        exit(EXIT_FAILURE);
      }
    }
#endif
    L3::Iterator<L3::SE3>::WINDOW_ITERATOR it = pose_windower->window->begin();

    // For each lidar pose, find the nearest scan
    while(it != pose_windower->window->end()) {
      // Nearest time
      L3::Iterator<L3::LMS151>::WINDOW_ITERATOR index =
        std::lower_bound(_window_buffer.begin(),
            _window_buffer.end(),
            it->first,
            LIDAR_comparator);

      if (index == _window_buffer.end()) {
        LOG(ERROR) << "Lookup failure";
        LOG(ERROR) << "LIDAR (" << LIDAR_iterator->window.size() << ")";
        for(std::deque< std::pair< double, boost::shared_ptr< L3::LMS151 > > >::iterator lidar_it = LIDAR_iterator->window.begin();
            lidar_it != LIDAR_iterator->window.end();
            lidar_it++) {
          LOG(ERROR) << lidar_it->first;
          LOG(ERROR) << "-------------";
        }

        LOG(ERROR) << "BUFFER (" <<  _window_buffer.size() << ")";
        for(std::deque< std::pair< double, boost::shared_ptr< L3::LMS151 > > >::iterator buf_it = _window_buffer.begin();
            buf_it != _window_buffer.end();
            buf_it++) {
          LOG(ERROR) << buf_it->first;
          LOG(ERROR) << "-------------";
        }

        LOG(ERROR) << "Pose window(" <<  pose_windower->window->size() << ")";
        for(std::deque< std::pair< double, boost::shared_ptr< L3::SE3 > > >::iterator _iterator = pose_windower->window->begin();
            _iterator != pose_windower->window->end();
            _iterator++) {
          LOG(ERROR) << _iterator->first;
          LOG(ERROR) << "-------------";
        }

        LOG(ERROR) << "Query time: "  << it->first;
        LOG(ERROR) << "Previous time: " << previous_update;

        exit(EXIT_FAILURE);
      }

      if ((index->first - it->first) == 0) {
        swathe.push_back(std::make_pair(it->second, index->second));
      }
      it++;
    }

    previous_update = _window_buffer.back().first;

    index = std::lower_bound(_window_buffer.begin(),
        _window_buffer.end(),
        pose_windower->window->front().first,
        LIDAR_comparator);

    _window_buffer.erase(_window_buffer.begin(), index);

    return true;
  }
} // L3
