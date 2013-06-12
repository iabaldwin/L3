#include  "VelocityProvider.h"
#include "Utils.h"

// Smoothing
#include "itpp/signal/filter.h"
#include "itpp/signal/freq_filt.h"
#include "itpp/signal/transforms.h"

//10-20
itpp::vec filter = "0.000091 0.000190 0.000327 0.000505 0.000727 0.000993 0.001301 0.001646 0.002021 0.002416 0.002822 0.003224 0.003610 0.003965 0.004277 0.004534 0.004725 0.004843 0.004883 0.004843 0.004725 0.004534 0.004277 0.003965 0.003610 0.003224 0.002822 0.002416 0.002021 0.001646 0.001301 0.000993 0.000727 0.000505 0.000327 0.000190 0.000091";
//itpp::vec filter = "0.000131 0.001215 0.002201 -0.000000 -0.005505 -0.008035 -0.002673 0.003664 -0.000000 -0.006502    0.008502    0.046832    0.060955    -0.000000    -0.107747    -0.151990    -0.056011    0.114721    0.200000    0.114721    -0.056011    -0.151990    -0.107747    -0.000000    0.060955    0.046832    0.008502    -0.006502    -0.000000    0.003664    -0.002673    -0.008035    -0.005505    -0.000000    0.002201    0.001215    0.000131 ";
//itpp::vec filter = ".333333333  .333333333 .333333333";
//itpp::vec filter = ".1 .1 .1 .1 .1 .1 .1 .1 .1 .1"; 
//itpp::vec filter = "0.000000 0.000022 -0.000058 0.000110 -0.000181 0.000271 -0.000383 0.000517 -0.000673 0.000848 -0.001040 0.001245 -0.001457 0.001669 -0.001873 0.002058 -0.002213 0.002326 -0.002382 0.002367 -0.002267 0.002068 -0.001754 0.001313 -0.000732 0.000000 0.000890 -0.001945 0.003168 -0.004558 0.006111 -0.007821 0.009676 -0.011662 0.013760 -0.015948 0.018202 -0.020495 0.022796 -0.025075 0.027298 -0.029434 0.031450 -0.033314 0.034996 -0.036471 0.037712 -0.038700 0.039418 -0.039854 1.040000 -0.039854 0.039418 -0.038700 0.037712 -0.036471 0.034996 -0.033314 0.031450 -0.029434 0.027298 -0.025075 0.022796 -0.020495 0.018202 -0.015948 0.013760 -0.011662 0.009676 -0.007821 0.006111 -0.004558 0.003168 -0.001945 0.000890 0.000000 -0.000732 0.001313 -0.001754 0.002068 -0.002267 0.002367 -0.002382 0.002326 -0.002213 0.002058 -0.001873 0.001669 -0.001457 0.001245 -0.001040 0.000848 -0.000673 0.000517 -0.000383 0.000271 -0.000181 0.000110 -0.000058 0.000022 0.000000";
//itpp::vec filter = "0.000000 -0.000169 -0.000138 0.000176 0.000357 -0.000000 -0.000533 -0.000395 0.000468 0.000891 -0.000000 -0.001205 -0.000858 0.000983 0.001814 -0.000000 -0.002326 -0.001619 0.001817 0.003289 -0.000000 -0.004082 -0.002801 0.003103 0.005551 0.000000 -0.006756 -0.004598 0.005058 0.008998 -0.000000 -0.010866 -0.007381 0.008118 0.014463 -0.000000 -0.017624 -0.012070 0.013426 0.024278 -0.000000 -0.030938 -0.021894 0.025413 0.048621 -0.000000 -0.074450 -0.061793 0.093166 0.302421 0.400000 0.302421 0.093166 -0.061793 -0.074450 -0.000000 0.048621 0.025413 -0.021894 -0.030938 -0.000000 0.024278 0.013426 -0.012070 -0.017624 -0.000000 0.014463 0.008118 -0.007381 -0.010866 -0.000000 0.008998 0.005058 -0.004598 -0.006756 0.000000 0.005551 0.003103 -0.002801 -0.004082 -0.000000 0.003289 0.001817 -0.001619 -0.002326 -0.000000 0.001814 0.000983 -0.000858 -0.001205 -0.000000 0.000891 0.000468 -0.000395 -0.000533 -0.000000 0.000357 0.000176 -0.000138 -0.000169 0.000000";

typedef std::deque <std::pair< double, Eigen::Matrix4f > >::iterator TRAJECTORY_ITERATOR;
typedef std::deque <std::pair< double, boost::shared_ptr< L3::LHLV > > >::iterator LHLV_ITERATOR;

namespace L3
{

    ScanMatchingVelocityProvider::ScanMatchingVelocityProvider( L3::ScanMatching::Engine* engine  ) 
        : engine(engine)
    {
        _linear_velocity_filter = boost::make_shared< L3::Estimator::AlphaBetaFilter >(.05,0.0001);
        _rotational_velocity_filter = boost::make_shared< L3::Estimator::AlphaBetaFilter >(.05,0.0001);
    }


    bool ScanMatchingVelocityProvider::update( double time )
    {
        return true;

        //if( engine->trajectory.empty() )
            //return false;
   
        //TRAJECTORY_ITERATOR iterator = std::lower_bound(
                //engine->trajectory.begin(),
                //engine->trajectory.end(),
                //previous_update,
                //comparator
                //);

        //previous_update = engine->trajectory.back().first;

        //// Copy the window
        //std::deque< std::pair< double, Eigen::Matrix4f > > trajectory( iterator, engine->trajectory.end() );
        
        //std::deque< std::pair< double, Eigen::Matrix4f > >::iterator it = trajectory.begin();
        //std::advance( it, 1 );

        //std::vector<double> data(4);
        
        //for( ; 
                //it != trajectory.end();
                //it++ )
        //{
            //// Compute instantaneous velocity
            //double distance = (sqrt( pow( it->second( 0,3 ) - (it-1)->second(0,3), 2 ) 
                        //+ pow( it->second( 1,3 ) - (it-1)->second(1,3), 2 ) ));

            //double dt = ( it->first - (it-1)->first );
            
            //double velocity = distance/dt;

            //if( std::isinf( velocity ) || std::isnan( velocity ) )
                //continue;

            //L3::SE3 previous = L3::Utils::Math::poseFromRotation( (it-1)->second );
            //L3::SE3 current = L3::Utils::Math::poseFromRotation( it->second );
            //double rotational_velocity = (previous.Q()-current.Q())/dt;

            //data[0] = velocity;
            //data[3] = rotational_velocity;

            //raw_velocities.push_back( std::make_pair( it->first, data ) );
        //}

        //if( raw_velocities.empty() )
            //return false;
         
        //std::pair< double, std::vector<double> > current = raw_velocities.back();

        //if( current.second[0] > 15 )
        //{
            //std::cout << current.second[0] << std::endl;
            //return false;
        //}

        //std::vector<double> data2( 4 );
        //_linear_velocity_filter->update( current.first, current.second[0] );
        //_rotational_velocity_filter->update( current.first, current.second[3] );
       
        //data2[0] = _linear_velocity_filter->_state.x;
        //data2[3] = _rotational_velocity_filter->_state.x;
        
        //filtered_velocities.push_back( std::make_pair( current.first, data2 ) );
      
        //if( filtered_velocities.back().first - filtered_velocities.front().first > 10.0 )
            //filtered_velocities.pop_front();

        return true;
    }

    
    bool LHLVVelocityProvider::update( double time  )
    {
        LHLV_ITERATOR it = std::lower_bound(
                iterator->window.begin(),
                iterator->window.end(),
                previous_update,
                comparator
                );

        std::transform( it, 
                iterator->window.end(),
                std::back_inserter( filtered_velocities ),
                z );

        if( !filtered_velocities.empty() )
        {
            // Compute trim 
            double trim_val = time - 10.0; 

            VELOCITY_ITERATOR erase_it = std::lower_bound(
                filtered_velocities.begin(),
                filtered_velocities.end(),
                trim_val,
                velocity_comparator 
                );

            filtered_velocities.erase( filtered_velocities.begin(), erase_it );

            previous_update = iterator->window.back().first;
        }
        
        return ( !filtered_velocities.empty() );
    }
}
