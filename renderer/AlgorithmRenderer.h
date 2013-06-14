#ifndef L3_VISUALISERS_ALGORITHM_H
#define L3_VISUALISERS_ALGORITHM_H

#include <GLV/glv.h>

#include "L3.h"

#include "RenderingUtils.h"

#include "Components.h"

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

            Updater* updater ;
            std::deque < boost::shared_ptr< glv::View > > views;
            std::deque < boost::shared_ptr< glv::Label > > labels;

            virtual void onDraw( glv::GLV& g ){};
        };


        /*
         *  Component visualisation types
         */

        struct DiscreteEstimatorVisualiser 
        {
            DiscreteEstimatorVisualiser( boost::shared_ptr< L3::Estimator::DiscreteEstimator<double> > estimator, Updater* updater = NULL ) 
                : estimator(estimator)
            {

            }

            boost::weak_ptr< L3::Estimator::DiscreteEstimator<double> > estimator;

            ColorInterpolator interpolator;

            glv::Label label;

        };

        struct DiscreteTranslationVisualiser : DiscreteEstimatorVisualiser, glv::View3D
        {
            DiscreteTranslationVisualiser(boost::shared_ptr< L3::Estimator::DiscreteEstimator<double> > estimator ) 
                : DiscreteEstimatorVisualiser(estimator), glv::View3D( glv::Rect(250,250) )
            {
                (*this) << label;
                label.pos( glv::Place::BL, 0, 0 ).anchor( glv::Place::TL ); 
                label.setValue( "Translation");
            }

            void onDraw3D( glv::GLV& g );
        };

        struct DiscreteRotationVisualiser : DiscreteEstimatorVisualiser, BasicPlottable<double>
        {
            DiscreteRotationVisualiser(boost::shared_ptr< L3::Estimator::DiscreteEstimator<double> > estimator ) : DiscreteEstimatorVisualiser(estimator)
            {
            }


            void onMap( glv::GraphicsData& g, const glv::Data& d, const glv::Indexer& i);
            void update();
        };


        struct TraversalVisualiser : glv::View3D
        {

            TraversalVisualiser( boost::shared_ptr< L3::Estimator::Minimisation<double> > algorithm  ) : algorithm(algorithm), glv::View3D( glv::Rect( 250,250 ))
            {
                *this  << label;
                label.pos( glv::Place::BL, 0, 0 ).anchor( glv::Place::TL ); 
                label.setValue( "Cost-function evaluations");
            }

            glv::Label label;

            void onDraw3D( glv::GLV& g );

            boost::weak_ptr< L3::Estimator::Minimisation<double> > algorithm;

        };

        /*
         *  Algorithm Visualisation types
         */
        struct MinimisationVisualiser : AlgorithmVisualiser
        {
            MinimisationVisualiser( boost::shared_ptr< L3::Estimator::Minimisation<double> > algorithm, Updater* updater = NULL )  
                : algorithm(algorithm)
            {
                // Estimation traversals
                boost::shared_ptr< glv::View > ptr = boost::dynamic_pointer_cast<glv::View>( boost::make_shared< TraversalVisualiser >( algorithm ) );
                views.push_back( ptr );

                this->updater = updater;

                (*this) << *ptr;

                // Number of iterations of the algorithm
                boost::shared_ptr< StatisticsPlottable<int> > plottable( new StatisticsPlottable<int>() ); 
                plottable->setVariable( algorithm->algorithm_iterations );

                plottables.push_back( plottable );

                if( updater )
                    updater->operator<<( plottable.get() );

                boost::shared_ptr< glv::Plot > plot( new glv::Plot( glv::Rect( 525, 80), *plottable ) );
       
                plot->disable( glv::Controllable );

                plot->range(0,100,0);
                plot->range(0,50*10,1);
               
                *plot << iterations_label;
                iterations_label.pos( glv::Place::BL, 0, 0 ).anchor( glv::Place::TL ); 
                iterations_label.setValue( "Algorithm iterations");

                (*this) << *plot;

                views.push_back( plot );
            }

            glv::Label iterations_label;

            ~MinimisationVisualiser()
            {
                if( updater )
                {
                    // Remove myself from the updates list  
                    std::list < Updateable* >::iterator it = std::find( updater->updateables.begin(), updater->updateables.end(), dynamic_cast< L3::Updateable*>(plottables.front().get() ) );

                    if( it != updater->updateables.end() )
                        updater->updateables.erase( it );
                }
            }
                
            std::deque< boost::shared_ptr< glv::View > > views;
            std::deque< boost::shared_ptr< glv::Plottable > > plottables;

            boost::weak_ptr< L3::Estimator::Minimisation<double> > algorithm;

        };

        struct IterativeDescentVisualiser : AlgorithmVisualiser
        {
            IterativeDescentVisualiser( boost::shared_ptr< L3::Estimator::IterativeDescent<double> > algorithm, Updater* updater = NULL ) ;

            boost::weak_ptr< L3::Estimator::IterativeDescent<double> > algorithm;
            std::deque< boost::shared_ptr< glv::Plottable >  > plottables;
            std::deque< boost::shared_ptr< glv::Label >  > labels;

            void onDraw( glv::GLV& g )
            {
                for ( std::deque< boost::shared_ptr< glv::Plottable > >::iterator it = plottables.begin();
                        it != plottables.end();
                        it++ )
                {
                    if( boost::shared_ptr< Updateable > ptr = boost::dynamic_pointer_cast< Updateable >( *it ) )
                        ptr->update();
                }

            }

        };

        struct HybridVisualiser : AlgorithmVisualiser
        {
            HybridVisualiser( boost::shared_ptr< L3::Estimator::Hybrid<double> > algorithm, Updater* updater=NULL ) ;
            
            boost::weak_ptr< L3::Estimator::Hybrid<double> > algorithm;

            std::deque < boost::shared_ptr< glv::View > > views;
            std::deque < boost::shared_ptr< glv::Label > > labels;
            std::deque < boost::shared_ptr< glv::Plottable >  > plottables;

        };


        struct ParticleFilterVisualiser : AlgorithmVisualiser
        {
            ParticleFilterVisualiser( boost::shared_ptr< L3::Estimator::ParticleFilter<double> > algorithm, Updater* updater, boost::shared_ptr< Composite> composite );
      
            ~ParticleFilterVisualiser();

            boost::weak_ptr< Composite > composite;
            boost::shared_ptr< glv::View > weight_visualiser;
            boost::shared_ptr< Leaf > particle_filter_renderer; 

            std::deque < boost::shared_ptr< glv::View > > views;
            std::deque < boost::shared_ptr< glv::Label > > labels;
    
            std::list< Leaf* > leafs;
            std::list< Updateable* > updateables;
        };

        /*
         *  Factory
         */
        struct AlgorithmRendererFactory
        {
            static boost::shared_ptr< AlgorithmVisualiser > Produce( boost::shared_ptr< L3::Estimator::Algorithm<double> > algorithm, Updater* updater, boost::shared_ptr< Composite> composite = boost::shared_ptr< Composite >() )
            {
                // Iterative descent
                if( boost::shared_ptr< L3::Estimator::IterativeDescent<double> > ptr = boost::dynamic_pointer_cast<L3::Estimator::IterativeDescent<double> >( algorithm ) )
                    return boost::dynamic_pointer_cast< AlgorithmVisualiser >( boost::make_shared< IterativeDescentVisualiser >( ptr, updater ) );

                // Optimisation
                if( boost::shared_ptr< L3::Estimator::Minimisation<double> > ptr = boost::dynamic_pointer_cast<L3::Estimator::Minimisation<double> >( algorithm ) )
                    return boost::dynamic_pointer_cast< AlgorithmVisualiser >( boost::make_shared< MinimisationVisualiser >( ptr, updater ) );

                // Hybrid
                if( boost::shared_ptr< L3::Estimator::Hybrid<double> > ptr = boost::dynamic_pointer_cast<L3::Estimator::Hybrid<double> >( algorithm ) )
                    return boost::dynamic_pointer_cast< AlgorithmVisualiser >( boost::make_shared< HybridVisualiser>( ptr, updater ) );

                // PF
                if( boost::shared_ptr< L3::Estimator::ParticleFilter<double> > ptr = boost::dynamic_pointer_cast<L3::Estimator::ParticleFilter<double> >( algorithm ) )
                    return boost::dynamic_pointer_cast< AlgorithmVisualiser >( boost::make_shared< ParticleFilterVisualiser>( ptr, updater, composite ) );

                return boost::shared_ptr<AlgorithmVisualiser>();

            }

        };

    }

}

#endif

