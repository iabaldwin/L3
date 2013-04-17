#ifndef L3_PREDICTOR_H
#define L3_PREDICTOR_H

#include "Integrator.h"

namespace L3
{

    struct Predictor
    {

        std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > > chain;
        
        template <typename InputIterator >
            bool predict( L3::SE3& predicted, L3::SE3& current, InputIterator start, InputIterator end )
            {
                // Destructive resize 
                chain.resize( std::distance( start, end ) );

                // Integrate 
                L3::trajectoryAccumulate( start,
                                            end, 
                                            chain.begin() );

                // Transform
                std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > >::iterator it = chain.begin();

                while( it != chain.end() )
                {
                    /*
                     *  TODO:
                     *  This is the issue, as the homogeneous and Euler parameterisations aren't
                     *  linked
                     */
                    //it->second->getHomogeneous()*=current.getHomogeneous();
                    it++;
                }
            }
    };

}

#endif
