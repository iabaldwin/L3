#ifndef L3_ALGORITHM_FACTORY
#define L3_ALGORITHM_FACTORY

#include "Estimator.h"

namespace L3
{
    namespace Estimator
    {

        template <typename T>
            struct AlgorithmFactory
            {
                static boost::shared_ptr< Algorithm<T> > produce( std::string algorithm, 
                                                                    boost::shared_ptr< CostFunction<T> > cost_function = boost::shared_ptr< CostFunction<T> >(),
                                                                    boost::shared_ptr< L3::HistogramPyramid<T> > pyramid = boost::shared_ptr<L3::HistogramPyramid<T> >() )
                {
                    if( algorithm == "ID" )
                        return boost::make_shared< IterativeDescent<T> >( cost_function, pyramid );

                    if( algorithm == "Min" )
                        return boost::make_shared< Minimisation<T> >( cost_function, pyramid );

                    if( algorithm == "Hybrid" )
                        return boost::make_shared< Hybrid<T> >( cost_function, pyramid );

                    return boost::shared_ptr< Algorithm<T> >(); 
                }

            };
    }
}

#endif

