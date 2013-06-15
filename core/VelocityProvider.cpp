#include  "VelocityProvider.h"
#include "Utils.h"

// Smoothing
#include "itpp/signal/filter.h"
#include "itpp/signal/freq_filt.h"
#include "itpp/signal/transforms.h"

// Filter tap weights/coefficients
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
    }


    bool ScanMatchingVelocityProvider::update( double time )
    {
        raw_velocities.push_back(  engine->raw_velocity_data );
        filtered_velocities.push_back(  engine->filtered_velocity_data );

        if( raw_velocities.back().first - raw_velocities.front().first > 10.0 )
            raw_velocities.pop_front();

        if( filtered_velocities.back().first - filtered_velocities.front().first > 10.0 )
            filtered_velocities.pop_front();

        return true;
    }

    FilteredScanMatchingVelocityProvider::FilteredScanMatchingVelocityProvider( boost::shared_ptr< L3::ConstantTimeIterator< L3::SMVelocity > > velocity_provider ) 
        : velocity_provider(velocity_provider)
    {
        _linear_velocity_filter = boost::make_shared< L3::Tracking::AlphaBetaFilter >(.1, .01);
        _rotational_velocity_filter = boost::make_shared< L3::Tracking::AlphaBetaFilter >(.05,0.01);
    
        raw_velocity_data.second.resize( 4);
        filtered_velocity_data.second.resize( 4);
    }

    struct sm_zipper : std::unary_function< std::pair<double, boost::shared_ptr<SMVelocity> >, std::pair< double, std::vector<double> > >
    {
    
        sm_zipper()
        {
            data.resize(4);
        }

        std::vector<double> data;

        std::pair< double, std::vector<double> > operator()( std::pair<double, boost::shared_ptr<SMVelocity> > entry )
        {
            return std::make_pair( entry.first, entry.second->data );
        }
    };

    bool FilteredScanMatchingVelocityProvider::update( double time )
    {
        boost::shared_ptr< L3::ConstantTimeIterator< L3::SMVelocity > > velocity_provider_ptr = velocity_provider.lock(); 

        if( !velocity_provider_ptr || velocity_provider_ptr->window.empty()  )
            return false;

        double linear_velocity     = velocity_provider_ptr->window.back().second->data[0];
        double rotational_velocity = velocity_provider_ptr->window.back().second->data[3];
        
        _linear_velocity_filter->update( velocity_provider_ptr->window.back().first, linear_velocity );
        _rotational_velocity_filter->update( velocity_provider_ptr->window.back().first, rotational_velocity );

        raw_velocity_data.first = velocity_provider_ptr->window.back().first;
        raw_velocity_data.second[0] = linear_velocity;
        raw_velocity_data.second[3] = rotational_velocity;

        filtered_velocity_data.first = velocity_provider_ptr->window.back().first;
        filtered_velocity_data.second[0] = _linear_velocity_filter->_state.x;
        //filtered_velocity_data.second[3] = _rotational_velocity_filter->_state.x;
        filtered_velocity_data.second[3] = rotational_velocity;

        raw_velocities.push_back( raw_velocity_data );
        filtered_velocities.push_back( filtered_velocity_data );

        while( filtered_velocities.back().first - filtered_velocities.front().first > 10.0 )
                filtered_velocities.pop_front();

        while( raw_velocities.back().first - raw_velocities.front().first > 10.0 )
                raw_velocities.pop_front();
        
        //filtered_velocities.clear();

        //sm_zipper zipper;

        //std::transform( velocity_provider_ptr->window.begin(), 
                //velocity_provider_ptr->window.end(),
                //std::back_inserter( filtered_velocities ),
                //zipper );

        //if( !filtered_velocities.empty()  )
        //{
            //while( filtered_velocities.back().first - filtered_velocities.front().first > 10.0 )
                //filtered_velocities.pop_front();
        //}

        //return ( !filtered_velocities.empty() );
   
        return true;
    };
    
    bool LHLVVelocityProvider::update( double time  )
    {
        filtered_velocities.clear();

        std::transform( iterator->window.begin(), 
                iterator->window.end(),
                std::back_inserter( filtered_velocities ),
                z );

        //if( !iterator->window.empty())
        //{
            //std::cout << time << std::endl;
            //std::cout << iterator->window.front().first << ':' << iterator->window.back().first << std::endl;
            //std::cout << iterator->window.back().first - time << std::endl; 
            //std::cout << "--------------" << std::endl;

            ////std::cout << filtered_velocities.front().first << ':' << filtered_velocities.back().first << std::endl;
            ////std::cout << time - filtered_velocities.front().first << ':' << filtered_velocities.back().first - time<< std::endl;

        //}
        
        //LHLV_ITERATOR it = std::lower_bound(
                //iterator->window.begin(),
                //iterator->window.end(),
                //previous_update,
                //comparator
                //);

        //std::transform( it, 
                //iterator->window.end(),
                //std::back_inserter( filtered_velocities ),
                //z );

        //if( !filtered_velocities.empty() )
        //{
            //while( filtered_velocities.front().first - filtered_velocities.back().first > 10.0 )
                //filtered_velocities.pop_front();

            //previous_update = iterator->window.back().first;
        //}
        
        return ( !filtered_velocities.empty() );
    }
}
