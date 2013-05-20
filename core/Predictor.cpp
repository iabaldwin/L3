#include "Predictor.h"
#include "Core.h"

#include <Eigen/LU>
#include <deque>

namespace L3
{

    bool Predictor::update( double t )
    {
        // 1. Find the element closest to the last upate
        
        Comparator< std::pair< double, boost::shared_ptr<L3::LHLV> > > comparator;
        std::deque< std::pair< double, boost::shared_ptr<L3::SE3> > > window;
        
        L3::Iterator<L3::LHLV>::WINDOW_ITERATOR index = std::lower_bound( LHLV_iterator->window.begin(), 
                                                                                        LHLV_iterator->window.end(), 
                                                                                        previous_update,
                                                                                        comparator );
        
        window.resize( std::distance( index, LHLV_iterator->window.end() ) );

        // 2. Isolate from here to the end of the chain
        L3::trajectoryAccumulate( index, LHLV_iterator->window.end(), window.begin() );

        previous_update = t;
    }

    bool Predictor::predict( L3::SE3& predicted, L3::SE3& current )
    {
        // Destructive resize 
        //chain.resize( std::distance( start, end ) );

        //// Integrate 
        //L3::trajectoryAccumulate( start, end, chain.begin() );

        //// End point
        //Eigen::Matrix4f last_pose = chain.back().second->getHomogeneous(); 

        //Eigen::Matrix4f delta( last_pose.inverse()  );

        //// Transform
        //std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > >::iterator it = chain.begin();
        //while( it != chain.end() )
        //{
        //Eigen::Matrix4f tmp( it->second->getHomogeneous() ); 

        //it->second->setHomogeneous( delta * tmp );

        //it++;
        //}

        return true;
    }
}

