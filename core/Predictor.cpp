#include "Predictor.h"

#include <Eigen/LU>
#include <deque>


namespace L3
{
    template <typename InputIterator >
        bool Predictor::predict( L3::SE3& predicted, L3::SE3& current, InputIterator start, InputIterator end )
        {
            // Destructive resize 
            chain.resize( std::distance( start, end ) );

            // Integrate 
            L3::trajectoryAccumulate( start, end, chain.begin() );

            // Transform
            std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > >::iterator it = chain.begin();

            // End point
            Eigen::Matrix4f last_pose = chain.back().second->getHomogeneous(); 
            
            Eigen::Matrix4f delta( last_pose.inverse()  );
            
            while( it != chain.end() )
            {
                Eigen::Matrix4f tmp( it->second->getHomogeneous() ); 

                it->second->setHomogeneous( delta * tmp );

                it++;
            }
        }
}

template bool L3::Predictor::predict<std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::LHLV> >, std::pair<double, boost::shared_ptr<L3::LHLV> >&, std::pair<double, boost::shared_ptr<L3::LHLV> >*> >(L3::SE3&, L3::SE3&, std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::LHLV> >, std::pair<double, boost::shared_ptr<L3::LHLV> >&, std::pair<double, boost::shared_ptr<L3::LHLV> >*>, std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::LHLV> >, std::pair<double, boost::shared_ptr<L3::LHLV> >&, std::pair<double, boost::shared_ptr<L3::LHLV> >*>);
