#include "Integrator.h"
#include <deque>

namespace L3
{
    template <typename InputIterator, typename OutputIterator >
        void trajectoryAccumulate( InputIterator begin, InputIterator end, OutputIterator output )
    {
        InputIterator current_in = begin;

        // Compute current time
        double current_time = current_in++->first;

        // Initialise
        double dt = 0.0;
        *output++ = std::make_pair( current_time, boost::make_shared<L3::SE3>( 0,0,0,0,0,0 ) );

        double x=0, y=0, z=0;
        double r=0, p=0, q=0;

        double w1=0, w2=0, w3=0;
        double lin_vel=0, x_vel=0, y_vel=0, z_vel=0;

        boost::shared_ptr< L3::SE3 > previous_pose;

        while ( current_in != end )
        {
            // Compute the update
            dt = current_in->first - current_time;
            current_time = current_in->first;

            // Compute roll, pitch, yaw
            w1 = current_in->second->data[5];
            w2 = current_in->second->data[4];
            w3 = current_in->second->data[3];

            // Get the SE3 pose
            previous_pose = boost::dynamic_pointer_cast< L3::SE3 >( (output-1)->second );

            assert( previous_pose );

            r = previous_pose->R() + w1*dt;
            p = previous_pose->P() + w2*dt;
            q = previous_pose->Q() + w3*dt;

            // Compute x_bar, y_bar, z_bar 
            lin_vel = current_in->second->data[9];

            //x_vel = lin_vel * cos(q);  
            //y_vel = lin_vel * sin(q);  
            //z_vel = lin_vel * sin(p);  

            x_vel = lin_vel * sin(q);  
            y_vel = lin_vel * cos(q);  
            z_vel = lin_vel * sin(p);  

            // Compute x y z
            x = previous_pose->X() + -1*x_vel*dt;
            y = previous_pose->Y() + y_vel*dt;
            z = previous_pose->Z() + z_vel*dt;

            // Log
            *output++ = std::make_pair( current_time, boost::make_shared<L3::SE3>( x, y, z, r, p, q ) ) ; 

            // Continue
            current_in++;
        }
    }
}

template void L3::trajectoryAccumulate<std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::LHLV> >, std::pair<double, boost::shared_ptr<L3::LHLV> >&, std::pair<double, boost::shared_ptr<L3::LHLV> >*>, std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::SE3> >, std::pair<double, boost::shared_ptr<L3::SE3> >&, std::pair<double, boost::shared_ptr<L3::SE3> >*> >(std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::LHLV> >, std::pair<double, boost::shared_ptr<L3::LHLV> >&, std::pair<double, boost::shared_ptr<L3::LHLV> >*>, std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::LHLV> >, std::pair<double, boost::shared_ptr<L3::LHLV> >&, std::pair<double, boost::shared_ptr<L3::LHLV> >*>, std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::SE3> >, std::pair<double, boost::shared_ptr<L3::SE3> >&, std::pair<double, boost::shared_ptr<L3::SE3> >*>);
template void L3::trajectoryAccumulate<std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::LHLV> >, std::pair<double, boost::shared_ptr<L3::LHLV> >&, std::pair<double, boost::shared_ptr<L3::LHLV> >*>, __gnu_cxx::__normal_iterator<std::pair<double, boost::shared_ptr<L3::SE3> >*, std::vector<std::pair<double, boost::shared_ptr<L3::SE3> >, std::allocator<std::pair<double, boost::shared_ptr<L3::SE3> > > > > >(std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::LHLV> >, std::pair<double, boost::shared_ptr<L3::LHLV> >&, std::pair<double, boost::shared_ptr<L3::LHLV> >*>, std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::LHLV> >, std::pair<double, boost::shared_ptr<L3::LHLV> >&, std::pair<double, boost::shared_ptr<L3::LHLV> >*>, __gnu_cxx::__normal_iterator<std::pair<double, boost::shared_ptr<L3::SE3> >*, std::vector<std::pair<double, boost::shared_ptr<L3::SE3> >, std::allocator<std::pair<double, boost::shared_ptr<L3::SE3> > > > >);
