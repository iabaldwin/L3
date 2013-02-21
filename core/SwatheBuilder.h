#ifndef L3_SWATHE_BIULDER_H
#define L3_SWATHE_BIULDER_H

#include "Datatypes.h"
#include "Definitions.h"
#include "Core.h"

template <typename T>
struct Comparator
{
    bool operator()( T* t, const double f )
    {
        return ( t->first < f );
    }
};


namespace L3
{
    class SwatheBuilder : public Observer
    {
        public:
            SwatheBuilder( L3::Iterator<L3::SE3>* pose_it, L3::Iterator<L3::LMS151>* LIDAR_it ) :
                pose_iterator( pose_it ), LIDAR_iterator( LIDAR_it ), window_duration(0.0) 
            {
            }

            L3::Tools::Timer t;
            bool update( double time )
            {
#ifndef NDEBUG
                t.begin();
#endif
                if (!pose_iterator->update( time ))
                    return false;

                if (!LIDAR_iterator->update( time ))
                    return false;

#ifndef NDEBUG
                //std::cout << __PRETTY_FUNCTION__ << ":" << t.end() << std::endl;
#endif

                /*
                 *  For each LIDAR scan, find the time
                 *  and associated pose
                 */
         
                L3::Iterator<L3::LMS151>::WINDOW_ITERATOR it = LIDAR_iterator->window.begin() ;

                swathe.clear();

                Comparator<  std::pair< double, boost::shared_ptr<L3::SE3> > > c;

                // For each lidar scan, find the nearest pose
                t.begin(); 
                while( it != LIDAR_iterator->window.end() )
                {
                    // Nearest time
                    L3::Iterator<L3::SE3>::WINDOW_ITERATOR index = std::lower_bound( pose_iterator->buffered_window.begin(), pose_iterator->buffered_window.end(), it->first, c );
             
                    if ( index == pose_iterator->buffered_window.end() ) // Bad, bad bad bad
                        throw std::exception(); 
                    else
                        swathe.push_back( std::make_pair( index->second, it->second ) );

                    it++;
                }

                //std::cout << t.end() << std::endl;
                window_duration = LIDAR_iterator->window.back().first - LIDAR_iterator->window.front().first;
               
                return true;
                    
            }

            double window_duration;

            double duration()
            {
                return window_duration;
            }



            SWATHE swathe;

        private:

            L3::Iterator<L3::SE3>*     pose_iterator;
            L3::Iterator<L3::LMS151>*    LIDAR_iterator;

    };

}

#endif

