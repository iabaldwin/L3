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
            
            glv::draw::translate( -1*ptr->pose_estimates->position->X(), -1*ptr->pose_estimates->position->Y(), -55 );

            //L3::ReadLock lock_a( ptr->mutex );
            L3::ReadLock lock( ptr->pose_estimates->mutex );
            std::vector<L3::SE3> estimates( ptr->pose_estimates->estimates.begin(), ptr->pose_estimates->estimates.end() );
            std::vector<double> costs( ptr->pose_estimates->costs.begin(), ptr->pose_estimates->costs.end() );
            lock.unlock();

            glv::Point3 vertices[ estimates.size()];
            glv::Color  colors[ estimates.size()];

            // Find min
            double min_val = *(std::min_element( costs.begin(), costs.end() ) );
            double max_val = *(std::max_element( costs.begin(), costs.end() ) );

            for( int i=0; i< estimates.size(); i++ )
            {
                double z_val = costs[i];

                //z_val -= (min_val-.5);

                z_val *= 1.0/max_val;

                vertices[i]( estimates[i].X(),
                                estimates[i].Y(),
                                z_val );
                
                colors[i].set( fabs( z_val ) );
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
