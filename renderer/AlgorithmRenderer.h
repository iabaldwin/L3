#ifndef L3_VISUALISERS_ALGORITHM_H
#define L3_VISUALISERS_ALGORITHM_H

#include <GLV/glv.h>

#include "L3.h"

#include "RenderingUtils.h"

namespace L3
{
    namespace Visualisers
    {

        struct AlgorithmVisualiser : glv::Table
        {
	        AlgorithmVisualiser( const char * arrangement="<", glv::space_t padX=3, glv::space_t padY=3, const glv::Rect& r=glv::Rect(0)) 
                : glv::Table( arrangement, padX, padY, r )
            {

            }

            std::deque < boost::shared_ptr< glv::View > > views;
            std::deque < boost::shared_ptr< glv::Label > > labels;
        };

        struct IterativeDescentVisualiser : AlgorithmVisualiser
        {

            IterativeDescentVisualiser( boost::shared_ptr< L3::Estimator::IterativeDescent<double> > algorithm ) ;

            boost::weak_ptr< L3::Estimator::IterativeDescent<double> > algorithm;

            struct DiscreteEstimatorVisualiser : glv::View3D
            {
                DiscreteEstimatorVisualiser( boost::shared_ptr< L3::Estimator::DiscreteEstimator<double> > estimator );


                boost::weak_ptr< L3::Estimator::DiscreteEstimator<double> > estimator;
                
                glv::Label label;

                ColorInterpolator interpolator;

                virtual void onDraw3D( glv::GLV& g );
            };

            struct DiscreteTranslationVisualiser : DiscreteEstimatorVisualiser
            {
                DiscreteTranslationVisualiser(boost::shared_ptr< L3::Estimator::DiscreteEstimator<double> > estimator ) : DiscreteEstimatorVisualiser(estimator)
                {
                    label.setValue( "Translation");
                }
                void onDraw3D( glv::GLV& g );
            };

            struct DiscreteRotationVisualiser : DiscreteEstimatorVisualiser
            {
                DiscreteRotationVisualiser(boost::shared_ptr< L3::Estimator::DiscreteEstimator<double> > estimator ) : DiscreteEstimatorVisualiser(estimator)
                {
                    label.setValue( "Rotation");
                }
                void onDraw3D( glv::GLV& g );
            };


        };

        struct AlgorithmRendererFactory
        {

            static boost::shared_ptr< AlgorithmVisualiser > Produce( boost::shared_ptr< L3::Estimator::Algorithm<double> > algorithm )
            {
                // Iterative descent
                boost::shared_ptr< L3::Estimator::IterativeDescent<double> > ptr = boost::dynamic_pointer_cast<L3::Estimator::IterativeDescent<double> >( algorithm );

                if( ptr )
                    return boost::dynamic_pointer_cast< AlgorithmVisualiser >( boost::make_shared< IterativeDescentVisualiser >( ptr ) );
           

                return boost::shared_ptr<AlgorithmVisualiser>();

            }

        };

    }

}

#endif

