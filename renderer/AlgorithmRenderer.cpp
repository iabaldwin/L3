#include "AlgorithmRenderer.h"

#include "RenderingUtils.h"

namespace L3
{
    namespace Visualisers
    {

        void TraversalVisualiser::onDraw3D( glv::GLV& g )
        {
            boost::shared_ptr< L3::Estimator::Minimisation<double> > algorithm_ptr = algorithm.lock();
          
            if( !algorithm_ptr)
                return;

            ColorInterpolator interpolator;

            // Note : this is going to be an issue
            L3::ReadLock lock( algorithm_ptr->mutex );
            std::vector< L3::SE3 > evaluations( algorithm_ptr->evaluations.begin(), algorithm_ptr->evaluations.end() );
            lock.unlock();
            
            far(150);

            glv::draw::translate( -1*evaluations.back().X(), -1*evaluations.back().Y(), -5 );

            glv::Point3 vertices[evaluations.size()];
            glv::Color  colors[evaluations.size()];

            int counter=0;
            for( std::vector< L3::SE3 >::iterator it = evaluations.begin();
                    it != evaluations.end();
                    it++ )
            {
                vertices[counter](it->X(), it->Y(), 0.0 );
                colors[counter].set( interpolator( double(counter)/evaluations.size() ) );
                counter++;
            }
            
            glv::draw::pointSize(3);
            glv::draw::paint( glv::draw::Points, vertices, colors, counter );
            glv::draw::pointSize(1);
            glv::draw::lineWidth(.1);
            glv::draw::paint( glv::draw::LineStrip, vertices, colors, counter );
        
        }

        IterativeDescentVisualiser::IterativeDescentVisualiser( boost::shared_ptr< L3::Estimator::IterativeDescent<double> > algorithm, Updater* updater ) 
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

            //L3::ReadLock lock( ptr->pose_estimates->mutex );
            std::vector<L3::SE3> estimates( ptr->pose_estimates->estimates.begin(), ptr->pose_estimates->estimates.end() );
            std::vector<double> costs( ptr->pose_estimates->costs.begin(), ptr->pose_estimates->costs.end() );
            //lock.unlock();

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

                z_val = (z_val - fabs(min_val))/(fabs(max_val) - fabs(min_val));

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
         
            //L3::ReadLock lock( ptr->pose_estimates->mutex );
            std::vector<L3::SE3> estimates( ptr->pose_estimates->estimates.begin(), ptr->pose_estimates->estimates.end() );
            std::vector<double> costs( ptr->pose_estimates->costs.begin(), ptr->pose_estimates->costs.end() );
            //lock.unlock();
           
            L3::WriteLock writer( this->mutex );
            mData.resize( glv::Data::DOUBLE, 2, costs.size() );
            glv::Indexer i( mData.size(1));

            int counter = 0;

            double mean = 0.0;
            for( int i=0; i<estimates.size(); i++ )
                mean += estimates[i].Q();
            mean /= estimates.size();

            double max_val = *(std::min_element( costs.begin(), costs.end() ) );
            double min_val = *(std::max_element( costs.begin(), costs.end() ) );

            while( i()  )
            {
                double z_val = fabs(costs[counter]);

                z_val = (z_val - fabs(min_val))/(fabs(max_val) - fabs(min_val));

                mData.assign( estimates[counter].Q()-mean, 0, counter );
                mData.assign( z_val, 1, counter); 
               
                counter++;
            }
                
        }

        void DiscreteRotationVisualiser::onMap( glv::GraphicsData& g, const glv::Data& d, const glv::Indexer& i)
        {
            L3::WriteLock reader( this->mutex );
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
        HybridVisualiser::HybridVisualiser( boost::shared_ptr< L3::Estimator::Hybrid<double> > algorithm, Updater* updater  ) : algorithm(algorithm)
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

        struct ParticleVisualiser : glv::View3D
        {

            ParticleVisualiser( boost::shared_ptr< L3::Estimator::ParticleFilter<double> > algorithm ) : glv::View3D( glv::Rect(400,400)), algorithm(algorithm)
            {
            }

            boost::weak_ptr< L3::Estimator::ParticleFilter<double> > algorithm;

            void onDraw3D( glv::GLV& g) 
            {
                boost::shared_ptr< L3::Estimator::ParticleFilter<double> > algorithm_ptr = algorithm.lock();

                if( !algorithm_ptr )
                    return;

                L3::ReadLock lock(algorithm_ptr->mutex);
                std::vector< L3::SE3 > hypotheses( algorithm_ptr->hypotheses.begin(), algorithm_ptr->hypotheses.end() );
                lock.unlock();

                //L3::SE3 first_particle = hypotheses.front();
                L3::SE3 translation = algorithm_ptr->current_prediction;;
                glv::draw::translate( -1*translation.X(), -1*translation.Y(), -10.0 );

                int counter = 0;
                glv::Point3 vertices[ hypotheses.size() ];
                glv::Color  colors[ hypotheses.size() ];

                for( L3::Estimator::ParticleFilter<double>::PARTICLE_ITERATOR it = hypotheses.begin();
                        it != hypotheses.end();
                        it++ )
                {
                    vertices[counter++]( it->X(), it->Y(), 0.0 );
                }
           
                glv::draw::paint( glv::draw::Points, vertices, colors, counter );
            }

        };

        struct ParticleWeightVisualiser : BasicPlottable<double>
        {

            ParticleWeightVisualiser()
            {
                this->mPrim = glv::draw::Points;
            }

            void onMap( glv::GraphicsData& g, const glv::Data& d, const glv::Indexer& i)
            {
                L3::ReadLock lock(this->mutex);
            
                int counter = 0;
                while(i()){
                
                    double x = d.at<double>( 0, counter );
                    double y = d.at<double>(1, counter );
                    g.addVertex(x, y*100);
                    counter++;
                }

            }

            std::vector< double > weights;
            std::vector< double > hypotheses;

            void update()
            {
                std::vector< double >  _weights( weights.begin(), weights.end() );
                std::vector< double > _hypotheses( hypotheses.begin(), hypotheses.end() );
                
                L3::WriteLock lock(this->mutex);
              
                mData.resize( glv::Data::DOUBLE, 2, _weights.size() );

                glv::Indexer i(mData.size(1));

                int counter = 0;

                while( i() )
                {
                    mData.assign( _hypotheses[counter], 0, counter );
                    mData.assign( _weights[counter], 1, counter );
                    counter++;
                }
            }
        };

        struct ParticleWeightOverview : Updateable, glv::Table
        {
            ParticleWeightOverview( boost::shared_ptr< L3::Estimator::ParticleFilter<double> > algorithm, Updater* updater = NULL ) : glv::Table( "x," ), algorithm(algorithm)
            {
                {
                    // X
                    x_weight = boost::make_shared< ParticleWeightVisualiser >();

                    boost::shared_ptr< glv::Plot > plot( new glv::Plot( glv::Rect( 525, 80), *x_weight ) );
                    *this << *plot;
                    
                    plots.push_back( plot );
                    
                    plot->disable( glv::Controllable );
                    //plot->showNumbering(true);
                    //plot->numbering(false,1);
                    
                    plot->range( -5, 5, 0 );
                    
                    //plot->range( 0, 100, 0 );
                    //plot->range( 0, 2, 1 );
            

                }

                {
                    // Y
                    y_weight = boost::make_shared< ParticleWeightVisualiser >( );

                    boost::shared_ptr< glv::Plot > plot( new glv::Plot( glv::Rect( 525, 80), *y_weight ) );
                    *this << *plot;
                    
                    plots.push_back( plot );
                    
                    plot->disable( glv::Controllable );
                    plot->showNumbering(true);
                    plot->numbering(false,1);

                    plot->range( -5, 5, 0 );
                    //plot->range( 0, 100, 0 );
                    //plot->range( 0, 2, 0 );
            

                }

                {
                    // Z
                    theta_weight = boost::make_shared< ParticleWeightVisualiser >();

                    boost::shared_ptr< glv::Plot > plot( new glv::Plot( glv::Rect( 525, 80), *theta_weight ) );
                    *this << *plot;
                    
                    plots.push_back( plot );

                    plot->disable( glv::Controllable );
                    plot->showNumbering(true);
                    plot->numbering(false,1);
 
                    plot->range( -5, 5, 0 );
                    //plot->range( 0, 100, 0 );
                    //plot->range( 0, .5, 0 );
                              
                }

                this->fit();
                this->arrange();
            }


            std::vector< boost::shared_ptr< glv::Plot > > plots;
            boost::shared_ptr< ParticleWeightVisualiser > x_weight, y_weight, theta_weight;
            boost::weak_ptr< L3::Estimator::ParticleFilter<double> > algorithm;
       
            void update()
            {
                boost::shared_ptr< L3::Estimator::ParticleFilter<double> > algorithm_ptr = algorithm.lock();

                if( !algorithm_ptr )
                    return;

                L3::ReadLock master( algorithm_ptr->mutex );
                std::vector<double> weights( algorithm_ptr->weights.begin(), algorithm_ptr->weights.end() );
                std::vector<L3::SE3> hypotheses( algorithm_ptr->hypotheses.begin(), algorithm_ptr->hypotheses.end() );
                L3::SE3 current_prediction = algorithm_ptr->current_prediction;
                master.unlock();
           
                x_weight->weights = weights;
                x_weight->hypotheses.resize( weights.size() ); 
                
                y_weight->weights = weights;
                y_weight->hypotheses.resize( weights.size() ); 
               
                theta_weight->weights = weights;
                theta_weight->hypotheses.resize( weights.size() ); 

                int counter = 0;
                for( std::vector<L3::SE3>::iterator it = hypotheses.begin();
                        it != hypotheses.end();
                        it++) 
                {
                    x_weight->hypotheses[counter]=it->X()-current_prediction.X();
                    y_weight->hypotheses[counter]=it->Y()-current_prediction.Y();
                    theta_weight->hypotheses[counter]=it->Q()-current_prediction.Q();
               
                    counter++;
                }
                x_weight->update();
                y_weight->update();
                theta_weight->update();

            }
        
        };

        /*
         *  Particle Filter
         */
        ParticleFilterVisualiser::ParticleFilterVisualiser( boost::shared_ptr< L3::Estimator::ParticleFilter<double> > algorithm, Updater* updater  )  
        {
            // Weight visualiser
            boost::shared_ptr< glv::View > weight_visualiser = boost::make_shared< ParticleWeightOverview >( algorithm, updater );
            *this << *weight_visualiser;

            if( updater )
                updater->operator<<( (boost::dynamic_pointer_cast< Updateable >(weight_visualiser)).get() );

            views.push_back( weight_visualiser );

            // Overhead visualiser
            //boost::shared_ptr< ParticleVisualiser > visualiser = boost::make_shared< ParticleVisualiser >( algorithm );
            //views.push_back( boost::dynamic_pointer_cast< glv::View >( visualiser ) );
            //*this << *visualiser;
 
            this->fit();
            this->arrange();
        }

    }
}
