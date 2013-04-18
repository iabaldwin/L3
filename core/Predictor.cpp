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
            L3::trajectoryAccumulate( start,
                                        end, 
                                        chain.begin() );

            // Transform
            std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > >::iterator it = chain.begin();

            // End point
            Eigen::Matrix4f last_pose = chain.back().second->getHomogeneous(); 
            
            //Eigen::Matrix4f translation = Eigen::Matrix4f::Zero();
         
            //translation(0,3) = last_pose(0,3);
            //translation(1,3) = last_pose(1,3);
            //translation(2,3) = last_pose(2,3);

            //last_pose -= translation;

            Eigen::Matrix4f delta( last_pose.inverse()  );
            
            // Zero trajector
            while( it != chain.end() )
            {
                //it->second->getHomogeneous() -= translation;

                Eigen::Matrix4f tmp( it->second->getHomogeneous() ); 

                it->second->setHomogeneous( delta * tmp );

                it++;
            }
        }
}

template bool L3::Predictor::predict<std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::LHLV> >, std::pair<double, boost::shared_ptr<L3::LHLV> >&, std::pair<double, boost::shared_ptr<L3::LHLV> >*> >(L3::SE3&, L3::SE3&, std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::LHLV> >, std::pair<double, boost::shared_ptr<L3::LHLV> >&, std::pair<double, boost::shared_ptr<L3::LHLV> >*>, std::_Deque_iterator<std::pair<double, boost::shared_ptr<L3::LHLV> >, std::pair<double, boost::shared_ptr<L3::LHLV> >&, std::pair<double, boost::shared_ptr<L3::LHLV> >*>);
