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
        //return ( f< t->first);
    }
};


namespace L3
{
    class SwatheBuilder : public Observer
    {
        public:
            SwatheBuilder( L3::Iterator<L3::Pose>* pose_it, L3::Iterator<L3::LIDAR>* LIDAR_it ) :
                pose_iterator( pose_it ), LIDAR_iterator( LIDAR_it ) 
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
                std::cout << __PRETTY_FUNCTION__ << ":" << t.end() << std::endl;
#endif

                /*
                 *  For each LIDAR scan, find the time
                 *  and associated pose
                 */
         
                L3::Iterator<L3::LIDAR>::WINDOW_ITERATOR it = LIDAR_iterator->window.begin() ;

                std::cout.precision( 15 );
            
                swathe.clear();

                Comparator<  std::pair< double, boost::shared_ptr<L3::Pose> > > c;

                // For each lidar scan, find the nearest pose
                while( it != LIDAR_iterator->window.end() )
                {
                    // Nearest time
                    L3::Iterator<L3::Pose>::WINDOW_ITERATOR index = std::lower_bound( pose_iterator->buffered_window.begin(), pose_iterator->buffered_window.end(), it->first, c );
             
                    if ( index == pose_iterator->buffered_window.end() ) // Bad, bad bad bad
                        throw std::exception(); 
                    else
                        swathe.push_back( std::make_pair( index->second, it->second ) );
             
                    it++;
                }

                return true;
                    
            }

            SWATHE swathe;
        
        private:

            L3::Iterator<L3::Pose>*     pose_iterator;
            L3::Iterator<L3::LIDAR>*    LIDAR_iterator;

    };

}

#endif

