#include "Windower.h"

namespace L3
{
  template <typename T>
    SlidingWindow<T>::~SlidingWindow() {
      if (input_stream.is_open()) {
        input_stream.close();
      }
      stop();
    }

  template <typename T>
    void SlidingWindow<T>::stop() {
      running = false;
    }

  template <typename T>
    bool SlidingWindow<T>::update(double time) {
      assert(initialised);
      current_time = time;

      mutex.lock();
      double diff = window.back().first - current_time;
      mutex.unlock();

      // Need more data?
      if ( diff < this->proximity)  {
        read_required = true;
      }
      return true;
    }

  template <typename T>
    std::deque< std::pair< double, boost::shared_ptr<T> > > SlidingWindow<T>::getWindow() {
      mutex.lock();
      temp = window;   
      mutex.unlock();
      return temp;
    }


  template <typename T>
    void SlidingWindow<T>::run() {
      if (!initialised) {
        throw std::exception();
      }

      while(running) {
        if (read_required) {
          read_required = false;  
          read();
          purge();
          if (!good()) {
            stop(); // Is the stream finished?
          }
        }
        usleep(.5*1e6);
      }
    }

  template <typename T>
    int SlidingWindow<T>::read() {
      int i;
      std::string line; 

      typename std::deque< std::pair< double, boost::shared_ptr<T> > > tmp;

      for (i=0; i<STACK_SIZE; i++) {
        // Is the stream good?
        if (!good()) {
          break;
        }

        std::getline(input_stream, line);

        // Empty newlines?
        if (line.size() == 0) {
          break;
        }
        tmp.push_back(L3::AbstractFactory<T>::produce(line));
      }

      mutex.lock();
      window.insert(window.end(), tmp.begin(), tmp.end()); 
      mutex.unlock();
      return i;
    }

  template <typename T>
    bool SlidingWindow<T>::initialise() {
      input_stream.open(target.c_str()); 

#ifndef NDEBUG
      L3::Timing::SysTimer t;
      std::cout << "Buffering...";
      t.begin();
#endif
      double duration = 0;

      while (duration < window_duration) {
        int entries_read = read();

        if (entries_read != STACK_SIZE) {
          // End of stream, this is all we have
          return false;
        }
        duration = window.back().first - window.front().first;
      }
#ifndef NDEBUG
      std::cout << window.size() << " entries read in " << t.elapsed() << "s" << std::endl;
#endif

      initialised = true;
      return initialised;
    }

  template <typename T>
    void SlidingWindow<T>::purge() {
      mutex.lock(); 
      while((current_time  - window.front().first > window_duration) && (window.size() != 0)) {
        window.pop_front();
      }
      mutex.unlock();
    }

  template <typename T>
    bool SlidingWindow<T>::good() {
      return input_stream.good() ? true : false; 
    };

  template <typename T>
    bool SlidingWindowBinary<T>::initialise()
    {
      this->input_stream.open(this->target.c_str(), std::ios::binary); 

#ifndef NDEBUG
      L3::Timing::SysTimer t;
      std::cout << "Binary:" << typeid(*this).name() << "Buffering...";
      t.begin();
#endif
      double duration = 0;

      while (duration < SlidingWindow<T>::window_duration) {
        int entries_read = read();

        if (entries_read != this->STACK_SIZE) {
          // End of stream, this is all we have
          return false;
        }
        duration = this->window.back().first - this->window.front().first;
#ifndef NDEBUG
        std::cout << entries_read << ":" << this->STACK_SIZE << ":" << duration << std::endl;
#endif
      }
#ifndef NDEBUG
      std::cout << this->window.size() << " entries read (" << this->window.size() << ") in " << t.elapsed() << "s" << std::endl;
#endif
      this->initialised = true;
      return this->initialised;
    }

  template <typename T>
    int SlidingWindowBinary<T>::read() {
      int i;

      typename std::deque< std::pair< double, boost::shared_ptr<T> > > tmp;

      for (i=0; i< this->STACK_SIZE; i++) {
        // Is the stream good?
        if (!this->good()) {
          break;
        }
        this->input_stream.read((char*)(&entry[0]), required*sizeof(double));

        tmp.push_back(L3::AbstractFactory<T>::produce(entry, &this->DEFAULT_MASK_POLICY));
      }

      this->mutex.lock();
      this->window.insert(this->window.end(), tmp.begin(), tmp.end()); 
      this->mutex.unlock();
      return i;
    }
}

template bool L3::SlidingWindow<L3::LMS151>::initialise();
template void L3::SlidingWindow<L3::LMS151>::run();
template bool L3::SlidingWindow<L3::LMS151>::good();
template int L3::SlidingWindow<L3::LMS151>::read();
template void L3::SlidingWindow<L3::LMS151>::stop();
template bool L3::SlidingWindow<L3::LMS151>::update(double);
template void L3::SlidingWindow<L3::LMS151>::purge();
template L3::SlidingWindow<L3::LMS151>::~SlidingWindow();
template std::deque< std::pair< double, boost::shared_ptr<L3::LMS151> > > L3::SlidingWindow<L3::LMS151>::getWindow();

template void L3::SlidingWindow<L3::SE3>::run();
template bool L3::SlidingWindow<L3::SE3>::good();
template int L3::SlidingWindow<L3::SE3>::read();
template void L3::SlidingWindow<L3::SE3>::stop();
template bool L3::SlidingWindow<L3::SE3>::update(double);
template L3::SlidingWindow<L3::SE3>::~SlidingWindow();
template std::deque< std::pair< double, boost::shared_ptr<L3::SE3> > > L3::SlidingWindow<L3::SE3>::getWindow();

template void L3::SlidingWindow<L3::LHLV>::run();
template bool L3::SlidingWindow<L3::LHLV>::good();
template int L3::SlidingWindow<L3::LHLV>::read();
template void L3::SlidingWindow<L3::LHLV>::stop();
template bool L3::SlidingWindow<L3::LHLV>::update(double);
template L3::SlidingWindow<L3::LHLV>::~SlidingWindow();
template std::deque< std::pair< double, boost::shared_ptr<L3::LHLV> > > L3::SlidingWindow<L3::LHLV>::getWindow();

template void L3::SlidingWindow<L3::Pose>::run();
template bool L3::SlidingWindow<L3::Pose>::good();
template int L3::SlidingWindow<L3::Pose>::read();
template bool L3::SlidingWindow<L3::Pose>::update(double);
template L3::SlidingWindow<L3::Pose>::~SlidingWindow();
template std::deque< std::pair< double, boost::shared_ptr<L3::Pose> > > L3::SlidingWindow<L3::Pose>::getWindow();


template void L3::SlidingWindow<L3::SMVelocity>::run();
template bool L3::SlidingWindow<L3::SMVelocity>::good();
template int L3::SlidingWindow<L3::SMVelocity>::read();
template bool L3::SlidingWindow<L3::SMVelocity>::update(double);
template L3::SlidingWindow<L3::SMVelocity>::~SlidingWindow();
template std::deque< std::pair< double, boost::shared_ptr<L3::SMVelocity> > > L3::SlidingWindow<L3::SMVelocity>::getWindow();

template bool L3::SlidingWindowBinary<L3::LMS151>::initialise();
template int L3::SlidingWindowBinary<L3::LMS151>::read();
template bool L3::SlidingWindowBinary<L3::SMVelocity>::initialise();
template int L3::SlidingWindowBinary<L3::SMVelocity >::read();
template bool L3::SlidingWindowBinary<L3::SE3>::initialise();
template int L3::SlidingWindowBinary<L3::SE3 >::read();
template bool L3::SlidingWindowBinary<L3::LHLV>::initialise();
template int L3::SlidingWindowBinary<L3::LHLV >::read();
