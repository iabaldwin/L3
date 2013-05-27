#include "AlgorithmRenderer.h"

namespace L3
{
    namespace Visualisers
    {

        void TraversalVisualiser::onDraw3D( glv::GLV& g )
        {
            boost::shared_ptr< L3::Estimator::Minimisation<double> > algorithm_ptr = algorithm.lock();
          
            if( !algorithm_ptr)
                return;

            L3::ReadLock lock( algorithm_ptr->mutex );
            std::vector< L3::SE3 > evaluations( algorithm_ptr->evaluations );
            lock.unlock();
            
            far(150);

            //glv::draw::translate( -1*evaluations[0].X(), -1*evaluations[0].Y(), -5 );
            glv::draw::translate( -1*evaluations.back().X(), -1*evaluations.back().Y(), -5 );

            glv::Point3 vertices[evaluations.size()];
            glv::Color  colors[evaluations.size()];

            int counter=0;
            for( std::vector< L3::SE3 >::iterator it = evaluations.begin();
                    it != evaluations.end();
                    it++ )
            {
                vertices[counter++]( it->X(), it->Y(), 0.0 );
            }
            
            glv::draw::pointSize(3);
            glv::draw::paint( glv::draw::Points, vertices, colors, counter );
            glv::draw::pointSize(1);
            glv::draw::paint( glv::draw::LineStrip, vertices, colors, counter );
            
        
        }

        IterativeDescentVisualiser::IterativeDescentVisualiser( boost::shared_ptr< L3::Estimator::IterativeDescent<double> > algorithm ) 
            : AlgorithmVisualiser( "x x,", 2, 2), algorithm(algorithm)
        {
            
            for( int i=0; i< algorithm->discrete_estimators.size(); i++ )
            {

                if( boost::shared_ptr< L3::Estimator::GridEstimates> ptr = boost::dynamic_pointer_cast< L3::Estimator::GridEstimates > ( algorithm->discrete_estimators[i]->pose_estimates ) )
                {

                    // Grid
                    boost::shared_ptr< glv::View > grid = boost::dynamic_pointer_cast< glv::View >( boost::make_shared< DiscreteTranslationVisualiser >( algorithm->discrete_estimators[i] ) );
                    grid->enable( glv::DrawBorder ); 
                    views.push_back( grid );

                    this->operator<<( *grid );

                }
                else if( boost::shared_ptr< L3::Estimator::RotationEstimates> ptr = boost::dynamic_pointer_cast< L3::Estimator::RotationEstimates > ( algorithm->discrete_estimators[i]->pose_estimates ) )
                {
                    // Rotation
                    boost::shared_ptr< glv::Plottable > rotation = boost::dynamic_pointer_cast< glv::Plottable >( boost::make_shared< DiscreteRotationVisualiser >( algorithm->discrete_estimators[i] ) );
                    boost::shared_ptr< glv::Plot > plot( new glv::Plot( glv::Rect( 250,250), *rotation) );

                    rotation->stroke( 2.0 );
                    rotation->color( glv::Color( 1, 0, 0 ) );


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
                else
                    std::cout << "Unknown"<< std::endl;


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
        void DiscreteTranslationVisualiser::onDraw3D( glv::GLV& g )
        {
            boost::shared_ptr< L3::Estimator::DiscreteEstimator<double> > ptr = estimator.lock();

            if( !ptr )
                return;
            
            glv::draw::translate( -1*ptr->pose_estimates->position->X(), -1*ptr->pose_estimates->position->Y(), -15 );

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

                //z_val -= fabs(min_val);

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
        void DiscreteRotationVisualiser::update()
        {
            boost::shared_ptr< L3::Estimator::DiscreteEstimator<double> > ptr = estimator.lock();

            if( !ptr )
                return;
         
            L3::ReadLock lock( ptr->pose_estimates->mutex );
            std::vector<L3::SE3> estimates( ptr->pose_estimates->estimates.begin(), ptr->pose_estimates->estimates.end() );
            std::vector<double> costs( ptr->pose_estimates->costs.begin(), ptr->pose_estimates->costs.end() );
            lock.unlock();
            
            mData.resize( glv::Data::DOUBLE, 2, costs.size() );
            glv::Indexer i( mData.size(1));

            int counter = 0;

            double mean = 0.0;
            for( int i=0; i<estimates.size(); i++ )
                mean += estimates[i].Q();
            mean /= estimates.size();

            while( i()  )
            {
                mData.assign( estimates[counter].Q()-mean, 0, counter );
                mData.assign( costs[counter], 1, counter); 
                counter++;
            }
        }

        void DiscreteRotationVisualiser::onMap( glv::GraphicsData& g, const glv::Data& d, const glv::Indexer& i)
        {
            int counter = 0;
            while(i()){
                double x = d.at<double>( 0, counter );
                double y = d.at<double>( 1, counter );
                g.addVertex(x, y);

                counter++;

            }
        }

        /*
         *  Hybrid
         */
        HybridVisualiser::HybridVisualiser( boost::shared_ptr< L3::Estimator::Hybrid<double> > algorithm ) : algorithm(algorithm)
        {

            for( int i=0; i< algorithm->discrete_estimators.size(); i++ )
            {


                if( boost::shared_ptr< L3::Estimator::GridEstimates> ptr = boost::dynamic_pointer_cast< L3::Estimator::GridEstimates > ( algorithm->discrete_estimators[i]->pose_estimates ) )
                {

                    // Grid
                    boost::shared_ptr< glv::View > grid = boost::dynamic_pointer_cast< glv::View >( boost::make_shared< DiscreteTranslationVisualiser >( algorithm->discrete_estimators[i] ) );
                    grid->enable( glv::DrawBorder ); 
                    views.push_back( grid );

                    this->operator<<( *grid );

                }
                else if( boost::shared_ptr< L3::Estimator::RotationEstimates> ptr = boost::dynamic_pointer_cast< L3::Estimator::RotationEstimates > ( algorithm->discrete_estimators[i]->pose_estimates ) )
                {
                    // Rotation
                    boost::shared_ptr< glv::Plottable > rotation = boost::dynamic_pointer_cast< glv::Plottable >( boost::make_shared< DiscreteRotationVisualiser >( algorithm->discrete_estimators[i] ) );
                    boost::shared_ptr< glv::Plot > plot( new glv::Plot( glv::Rect( 250,250), *rotation) );

                    rotation->stroke( 2.0 );
                    rotation->color( glv::Color( 1, 0, 0 ) );


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
            }

            boost::shared_ptr< glv::View > minimisation_visualiser = boost::dynamic_pointer_cast< glv::View >( boost::make_shared< TraversalVisualiser >( algorithm->minimisation ) );
            views.push_back( minimisation_visualiser );
       
            this->operator<<( *minimisation_visualiser );
        }


    }
}
