#include "Iterator.h"
#include "Misc.h"

namespace L3
{
  template <typename T>
    bool ConstantTimeIterator<T>::update(double time) {
      // Obtain the pointer to the windower
      boost::shared_ptr< L3::SlidingWindow<T> > windower_ptr = this->windower.lock();
      CHECK_NOTNULL(windower_ptr);
      if (!windower_ptr) {
        return false;
      }

      windower_ptr->update(time);

      // Lock the windower
      windower_ptr->mutex.lock();

      // Find the element with the closest time to *now*
      typename Iterator<T>::BUFFERED_WINDOW_ITERATOR it = std::lower_bound(windower_ptr->window.begin(),
          windower_ptr->window.end(),
          time,
          _pair_comparator);

      if (it == windower_ptr->window.end())  {
        LOG(INFO) << std::setprecision(4) << std::fixed <<  time << "->" << windower_ptr->window.front().first << ":" << windower_ptr->window.back().first;
        return false;
      }

      double data_swathe_length = 0;

      typename Iterator<T>::WINDOW_ITERATOR it_back_iterator = it;

      L3::WriteLock lock(this->mutex);
      this->window.clear();

      // Copy, as we alter the swathe_length variable
      double swathe_length_local = swathe_length;

      // Working backwards, build up the data swathe
      while(data_swathe_length < swathe_length_local) {
        // At the beginning? Is this all we have?
        if (it_back_iterator == windower_ptr->window.begin())
          break;

        //this->window.push_front(*it_back_iterator);
        this->window.push_front(std::make_pair(it_back_iterator->first, boost::make_shared<T>(*(it_back_iterator->second)) ));

        // Compute dt
        data_swathe_length = (*it).first - (*it_back_iterator).first ;

        // Continue
        it_back_iterator--;
      }
      windower_ptr->mutex.unlock();

#ifndef NDEBUG
      if(!this->window.empty()) {
        if(time - this->window.back().first > 1.0/50.0) {
          LOG(ERROR) << "Timing issue: " <<  time - this->window.back().first;
        }
      }
#endif
      lock.unlock();
      return true;
    }

} // L3

// Explicit Instantiations
template class L3::ConstantTimeIterator<L3::SMVelocity>;
template class L3::ConstantTimeIterator<L3::SE3>;
template class L3::ConstantTimeIterator<L3::LHLV>;
template class L3::ConstantTimeIterator<L3::LMS151>;
