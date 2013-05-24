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
                boost::shared_ptr< glv::Plottable > rotation = boost::dynamic_pointer_cast< glv::Plottable >( boost::make_shared< DiscreteRotationVisualiser >( algorithm->discrete_estimators[(i*2)+1] ) );
                boost::shared_ptr< glv::Plot > plot( new glv::Plot( glv::Rect( 250,250), *rotation) );
             
                plottables.push_back( rotation );
                views.push_back( plot );
                
                plot->enable( glv::DrawBorder ); 

                boost::shared_ptr< glv::Label > rotation_label( new glv::Label() );

                (*plot) << *rotation_label;
                rotation_label->pos( glv::Place::BL, 0, 0 ).anchor( glv::Place::TL ); 
                rotation_label->setValue( "Rotation");

                labels.push_back( rotation_label );

                this->operator<<( *plot );
            }

            // Add the label
            boost::shared_ptr< glv::Label > algorithm_label = boost::make_shared< glv::Label >( "Iterative Descent Algorithm" );
            labels.push_back( algorithm_label ); 

            this->operator<<( *algorithm_label );

            this->arrange();
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

            /*
             *  Find Bounds
             *  Q. Why is this the other way around?
             *  A. We work with negative costs, by default. Max-value is therefore -1*actual_min 
             */
            
            double max_val = *(std::min_element( costs.begin(), costs.end() ) );
            double min_val = *(std::max_element( costs.begin(), costs.end() ) );

            glv::Color interpolated;
            for( int i=0; i< estimates.size(); i++ )
            {
                double z_val = fabs(costs[i]);

                z_val /= fabs(max_val);

                vertices[i]( estimates[i].X(),
                                estimates[i].Y(),
                                z_val );
         
                interpolated = interpolator( z_val );

                colors[i] = interpolated;
            }
         
            glv::draw::enable( glv::draw::Blend );
            glv::draw::paint( glv::draw::Points, vertices, colors, ptr->pose_estimates->estimates.size() );
            glv::draw::disable( glv::draw::Blend );

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
