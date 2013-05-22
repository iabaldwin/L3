#include "AlgorithmRenderer.h"

namespace L3
{
    namespace Visualisers
    {

        IterativeDescentVisualiser::IterativeDescentVisualiser( boost::shared_ptr< L3::Estimator::IterativeDescent<double> > algorithm ) 
            : AlgorithmVisualiser( "x x,", 2, 2), algorithm(algorithm)
        {
            
            for( int i=0; i< algorithm->discrete_estimators.size()/2; i++ )
            {
                // Grid
                boost::shared_ptr< glv::View > grid = boost::dynamic_pointer_cast< glv::View >( boost::make_shared< DiscreteTranslationVisualiser >( algorithm->discrete_estimators[i*2] ) );
                grid->enable( glv::DrawBorder ); 
                views.push_back( grid );

                this->operator<<( *grid );

                // Rotation
                boost::shared_ptr< glv::View > rotation = boost::dynamic_pointer_cast< glv::View >( boost::make_shared< DiscreteRotationVisualiser >( algorithm->discrete_estimators[(i*2)+1] ) );
                rotation->enable( glv::DrawBorder ); 
                views.push_back( rotation );

                this->operator<<( *rotation );

            }

            // Add the label
            boost::shared_ptr< glv::Label > algorithm_label = boost::make_shared< glv::Label >( "Iterative Descent Algorithm" );
            labels.push_back( algorithm_label ); 

            this->operator<<( *algorithm_label );

            this->arrange();
        }

        IterativeDescentVisualiser::DiscreteEstimatorVisualiser::DiscreteEstimatorVisualiser( boost::shared_ptr< L3::Estimator::DiscreteEstimator<double> > estimator ) :
            glv::View3D( glv::Rect(275,250) ), estimator(estimator)
        {
            (*this) << label;
            
            label.pos( glv::Place::BL, 0, 0 ).anchor( glv::Place::TL ); 
        }

        void IterativeDescentVisualiser::DiscreteEstimatorVisualiser::onDraw3D( glv::GLV& g )
        {
            
        }
   
        /*
         *  Translation
         */
        void IterativeDescentVisualiser::DiscreteTranslationVisualiser::onDraw3D( glv::GLV& g )
        {
            boost::shared_ptr< L3::Estimator::DiscreteEstimator<double> > ptr = estimator.lock();

            if( !ptr )
                return;
            
            glv::draw::rotate( -15, 0, 0 );
            glv::draw::translate( -1*ptr->pose_estimates->position->X(), -1*ptr->pose_estimates->position->Y(), -65 );


            L3::ReadLock lock( ptr->mutex );

            glv::Point3 vertices[ ptr->pose_estimates->estimates.size()];
            glv::Color  colors[ ptr->pose_estimates->estimates.size()];

            for( int i=0; i<ptr->pose_estimates->estimates.size(); i++ )
            {
                vertices[i]( ptr->pose_estimates->estimates[i].X(),
                                ptr->pose_estimates->estimates[i].Y(),
                                ptr->pose_estimates->costs[i]*20 );
          
                colors[i].set( fabs(ptr->pose_estimates->costs[i]*5 ) );
            }
          

            glv::draw::paint( glv::draw::Points, vertices, colors, ptr->pose_estimates->estimates.size() );

        }
    
    
        /*
         *  Rotation
         */
        void IterativeDescentVisualiser::DiscreteRotationVisualiser::onDraw3D( glv::GLV& g )
        {
            boost::shared_ptr< L3::Estimator::DiscreteEstimator<double> > ptr = estimator.lock();

            glv::draw::translate( -1*ptr->pose_estimates->position->X(), -1*ptr->pose_estimates->position->Y(), -70 );

            if( !ptr )
                return;

            L3::ReadLock lock( ptr->mutex );

            //glv::Point3 vertices[ ptr->pose_estimates->estimates.size()];
            //glv::Color  colors[ ptr->pose_estimates->estimates.size()];

            //for( int i=0; i<ptr->pose_estimates->estimates.size(); i++ )
            //{
                //vertices[i]( ptr->pose_estimates->estimates[i].X(),
                                //ptr->pose_estimates->estimates[i].Y(),
                                //ptr->pose_estimates->costs[i] );
            //}

            //glv::draw::paint( glv::draw::Points, vertices, colors, ptr->pose_estimates->estimates.size() );


        }

    }

}
