#include "Integrator.h"
#include <deque>
#include <iterator>

namespace L3
{
    template <typename InputIterator, typename OutputIterator >
        OutputIterator trajectoryAccumulate( InputIterator begin, InputIterator end, OutputIterator output, double& distance, double required_distance )
    {
        if( std::distance( begin, end) == 0 )
            return output;

        // Input copy
        InputIterator current_in = begin;

        // Compute current time
        double current_time = current_in++->first;

        // Distance
        distance = 0.0; 

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

            r = previous_pose->R() + w1*dt;
            p = previous_pose->P() + w2*dt;
            q = previous_pose->Q() + (-1*w3)*dt;

            // Compute x_bar, y_bar, z_bar 
            lin_vel = current_in->second->data[9];

            x_vel = lin_vel * sin(q);  
            y_vel = lin_vel * cos(q);  
            z_vel = lin_vel * sin(p);  

            // Compute x y z
            x = previous_pose->X() + -1*x_vel*dt;
            y = previous_pose->Y() + y_vel*dt;
            z = previous_pose->Z() + z_vel*dt;

            // Log
            boost::shared_ptr< L3::SE3 > current_pose =  boost::make_shared<L3::SE3>( x, y, z, r, p, q );
            
            double inter_pose_distance = L3::Math::norm(*current_pose, *previous_pose );
            
            *output = std::make_pair( current_time, current_pose );

            distance += inter_pose_distance;
      
            if( distance > required_distance )
                break;

            // Continue
            output++;
            current_in++;
        }

        return output;
    }

    template <typename InputIterator, typename OutputIterator >
        void reverseTrajectoryAccumulate( InputIterator begin, InputIterator end, OutputIterator output, double required_increment, double total_distance, int& written  )
        {
            if( std::distance( begin, end) == 0 )
                return;

            OutputIterator writer = output;

            // Input copy
            InputIterator current_in = begin;

            // Compute current time
            double previous_time = current_in++->first;

            // Distance
            double distance = 0.0; 
            double incremental_distance = 0.0; 
            written = 0;


            // Initialise
            double dt = 0.0, current_time;
            boost::shared_ptr< L3::SE3 > previous_pose = boost::make_shared<L3::SE3>( 0,0,0,0,0,0 );
            *writer++ = std::make_pair( previous_time, previous_pose );
            written++;

            double x=0, y=0, z=0;
            double r=0, p=0, q=0;

            double w1=0, w2=0, w3=0;
            double lin_vel=0, x_vel=0, y_vel=0, z_vel=0;

            while ( current_in != end )
            {
                // Compute the update
                current_time = current_in->first;
                dt = current_time - previous_time;

                // Compute roll, pitch, yaw
                w1 = current_in->second->data[5];
                w2 = current_in->second->data[4];
                w3 = current_in->second->data[3];

                r = previous_pose->R() + w1*dt;
                p = previous_pose->P() + w2*dt;
                q = previous_pose->Q() + (-1*w3)*dt;

                // Compute x_bar, y_bar, z_bar 
                lin_vel = current_in->second->data[9];

                x_vel = lin_vel * sin(q);  
                y_vel = lin_vel * cos(q);  
                z_vel = lin_vel * sin(p);  

                // Compute x y z
                x = previous_pose->X() + -1*x_vel*dt;
                y = previous_pose->Y() + y_vel*dt;
                z = previous_pose->Z() + z_vel*dt;

                // Log
                boost::shared_ptr< L3::SE3 > current_pose =  boost::make_shared<L3::SE3>( x, y, z, r, p, q );

                double inter_pose_distance = L3::Math::norm(*current_pose, *previous_pose );

                incremental_distance += inter_pose_distance;
                distance += inter_pose_distance;

                if( incremental_distance > required_increment )
                {
                    *writer++ = std::make_pair( current_time, current_pose );
                    incremental_distance = 0.0;
                }

                if( distance >= total_distance )
                    break;

                previous_pose = current_pose;
                previous_time = current_time;

                // Continue
                current_in++;
                written++;
            
            }
        }
}

template std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::SE3> >, std::pair<double, boost::shared_ptr<L3::SE3> >&, std::pair<double, boost::shared_ptr<L3::SE3> >*> L3::trajectoryAccumulate<std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::LHLV> >, std::pair<double, boost::shared_ptr<L3::LHLV> >&, std::pair<double, boost::shared_ptr<L3::LHLV> >*>, std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::SE3> >, std::pair<double, boost::shared_ptr<L3::SE3> >&, std::pair<double, boost::shared_ptr<L3::SE3> >*> >(std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::LHLV> >, std::pair<double, boost::shared_ptr<L3::LHLV> >&, std::pair<double, boost::shared_ptr<L3::LHLV> >*>, std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::LHLV> >, std::pair<double, boost::shared_ptr<L3::LHLV> >&, std::pair<double, boost::shared_ptr<L3::LHLV> >*>, std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::SE3> >, std::pair<double, boost::shared_ptr<L3::SE3> >&, std::pair<double, boost::shared_ptr<L3::SE3> >*>, double&, double);
template void L3::reverseTrajectoryAccumulate<std::reverse_iterator<std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::LHLV> >, std::pair<double, boost::shared_ptr<L3::LHLV> >&, std::pair<double, boost::shared_ptr<L3::LHLV> >*> >, std::back_insert_iterator<std::deque<std::pair<double, boost::shared_ptr<L3::SE3> >, std::allocator<std::pair<double, boost::shared_ptr<L3::SE3> > > > > >(std::reverse_iterator<std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::LHLV> >, std::pair<double, boost::shared_ptr<L3::LHLV> >&, std::pair<double, boost::shared_ptr<L3::LHLV> >*> >, std::reverse_iterator<std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::LHLV> >, std::pair<double, boost::shared_ptr<L3::LHLV> >&, std::pair<double, boost::shared_ptr<L3::LHLV> >*> >, std::back_insert_iterator<std::deque<std::pair<double, boost::shared_ptr<L3::SE3> >, std::allocator<std::pair<double, boost::shared_ptr<L3::SE3> > > > >, double, double, int&);
