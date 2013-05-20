#ifndef L3_VISUALISERS_ALGORITHM_H
#define L3_VISUALISERS_ALGORITHM_H

namespace L3
{
    namespace Visualisers
    {

        struct AlgorithmRendererFactory
        {

            static boost::shared_ptr< glv::View > Produce( L3::Estimators::Algorithm<double>* algorithm )
            {

                // Iterative descent

            }

        };

    }

}

#endif

