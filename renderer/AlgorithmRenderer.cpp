#include "AlgorithmRenderer.h"
#include "RenderingUtils.h"

namespace L3
{
    namespace Visualisers
    {

        struct TraversalVisualiser : Updateable
        {
            /*
             *  Minimisation::Traversals
             */
            TraversalVisualiser( boost::shared_ptr< L3::Estimator::Minimisation<double> > algorithm  ) : algorithm(algorithm)
            {
            }

            boost::weak_ptr< L3::Estimator::Minimisation<double> > algorithm;
                
            std::vector< L3::SE3 > evaluations;

            virtual void onDraw3D( glv::GLV& g )
            {
                if( evaluations.empty() )
                    return;

                ColorInterpolator interpolator;

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

                CoordinateSystem( evaluations.front(), 1 ).onDraw3D(g);
                CoordinateSystem( evaluations.back(), 1 ).onDraw3D(g);
            }            

            virtual void update()
            {
                boost::shared_ptr< L3::Estimator::Minimisation<double> > algorithm_ptr = algorithm.lock();

                if( !algorithm_ptr)
                    return;
                
                evaluations.assign( algorithm_ptr->evaluations.begin(), algorithm_ptr->evaluations.end() );
            }
        };


        struct TraversalVisualiserView : TraversalVisualiser, glv::View3D
        {
            /*
             *  Minimisation::Traversals
             */
            TraversalVisualiserView( boost::shared_ptr< L3::Estimator::Minimisation<double> > algorithm  ) 
                : TraversalVisualiser( algorithm ), glv::View3D( glv::Rect( 250,250 ))
            {
                *this  << label;
                label.pos( glv::Place::BL, 0, 0 ).anchor( glv::Place::TL ); 
                label.setValue( "Cost-function evaluations");
            }

            glv::Label label;

            void onDraw3D( glv::GLV& g )
            {
                boost::shared_ptr< L3::Estimator::Minimisation<double> > algorithm_ptr = algorithm.lock();

                if( !algorithm_ptr)
                    return;

                ColorInterpolator interpolator;

                std::vector< L3::SE3 > evaluations( algorithm_ptr->evaluations.begin(), algorithm_ptr->evaluations.end() );

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

                CoordinateSystem( evaluations.front(), 1 ).onDraw3D(g);
                CoordinateSystem( evaluations.back(), 1 ).onDraw3D(g);
            }            

        };


        struct TraversalVisualiserLeaf : TraversalVisualiser, Leaf
        {
            TraversalVisualiserLeaf( boost::shared_ptr< L3::Estimator::Minimisation<double> > algorithm  ) 
                : TraversalVisualiser( algorithm )
            {
            }
       
            void onDraw3D( glv::GLV& g )
            {
                TraversalVisualiser::onDraw3D(g);
            }
      
            void update()
            {
                TraversalVisualiser::update();
            }

        };

        /*
         *  Minimisation
         */
        MinimisationVisualiser::MinimisationVisualiser( boost::shared_ptr< L3::Estimator::Minimisation<double> > algorithm, Updater* updater,  boost::shared_ptr< Composite> composite  )  
            : algorithm(algorithm), composite(composite)
        {
            if ( !updater )
                throw std::exception();

            this->updater = updater;
            
            // Number of iterations of the algorithm
            boost::shared_ptr< StatisticsPlottable<int> > plottable( new StatisticsPlottable<int>() ); 
            plottable->setVariable( algorithm->algorithm_iterations );

            plottables.push_back( plottable );
            updater->operator<<( plottable.get() );
            updateables.push_back( plottable.get() );

            boost::shared_ptr< glv::Plot > plot( new glv::Plot( glv::Rect( 525, 80), *plottable ) );

            plot->disable( glv::Controllable );
            plot->range(0,100,0);
            plot->range(0,50*10,1);

            *plot << iterations_label;
            iterations_label.pos( glv::Place::BL, 0, 0 ).anchor( glv::Place::TL ); 
            iterations_label.setValue( "Algorithm iterations");

            (*this) << *plot;
            views.push_back( plot );

            // Estimation traversals
            boost::shared_ptr< glv::View > ptr = boost::dynamic_pointer_cast<glv::View>( boost::make_shared< TraversalVisualiserView >( algorithm ) );
            views.push_back( ptr );

            (*this) << *ptr;

            // Estimation traversals : Leaf
            boost::shared_ptr< Composite > composite_ptr = this->composite.lock();

            if( composite_ptr )
            {
                traversal_renderer = boost::make_shared< TraversalVisualiserLeaf >( algorithm );

                composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(traversal_renderer.get() ))); 
                leafs.push_back( dynamic_cast<L3::Visualisers::Leaf*>(traversal_renderer.get() ) );

                updater->operator<<( dynamic_cast< Updateable* >(traversal_renderer.get() ) );
                updateables.push_back(dynamic_cast< Updateable* >(traversal_renderer.get() ) ); 

            }
            else
                std::cerr << "No composite pointer passed!" << std::endl;

            this->fit();
            this->arrange();
        }

        MinimisationVisualiser::~MinimisationVisualiser()
        {
            
            for( std::list< Updateable* >::iterator updateable_it =  updateables.begin();
                    updateable_it != updateables.end();
                    updateable_it++ )
            {
                updater->remove( *updateable_it );
            }

            boost::shared_ptr< Composite > composite_ptr = this->composite.lock();

            if( composite_ptr )
            {
                for( std::list< Leaf* >::iterator leaf_it =  leafs.begin();
                        leaf_it != leafs.end();
                        leaf_it++ )
                {
                    std::list < Leaf* >::iterator it 
                        = std::find( composite_ptr->components.begin(), 
                                composite_ptr->components.end(), 
                                *leaf_it
                                );

                    if( it != composite_ptr->components.end() )
                        composite_ptr->components.erase( it );

                }
            }
        }
       

        /*
         *  IterativeDescent
         */
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
                    std::cerr<< "Unknown type!"<< std::endl;
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

            std::vector<L3::SE3> estimates( ptr->pose_estimates->estimates.begin(), ptr->pose_estimates->estimates.end() );
            std::vector<double> costs( ptr->pose_estimates->costs.begin(), ptr->pose_estimates->costs.end() );

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
            writer.unlock();

        }

        void DiscreteRotationVisualiser::onMap( glv::GraphicsData& g, const glv::Data& d, const glv::Indexer& i)
        {
            L3::ReadLock reader( this->mutex );
            int counter = 0;
            while(i()){
                double x = d.at<double>( 0, counter );
                double y = d.at<double>( 1, counter );
                g.addVertex(x, y);

                counter++;
            }
            reader.unlock();
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

            boost::shared_ptr< glv::View > minimisation_visualiser = boost::dynamic_pointer_cast< glv::View >( boost::make_shared< TraversalVisualiserView >( algorithm->minimisation ) );
            views.push_back( minimisation_visualiser );

            this->operator<<( *minimisation_visualiser );
        }

        struct ParticleFilterRendererLeaf : Leaf, Updateable, Lockable
        {
            ParticleFilterRendererLeaf( boost::shared_ptr< L3::Estimator::ParticleFilter<double> > filter ) : filter(filter)
            {
            }

            boost::weak_ptr< L3::Estimator::ParticleFilter<double> > filter;

            std::vector< L3::SE3 > hypotheses;

            void onDraw3D( glv::GLV& g )
            {
                L3::ReadLock lock( this->mutex );
                std::vector< L3::SE3> _hypotheses( hypotheses.begin(), hypotheses.end() );
                lock.unlock();

                /*
                 *  This is not called on update
                 */
                for ( L3::Estimator::ParticleFilter<double>::PARTICLE_ITERATOR it = _hypotheses.begin();  
                        it != _hypotheses.end();
                        it++ )
                    CoordinateSystem( *it ).onDraw3D(g);
            }


            void update()
            {
                boost::shared_ptr< L3::Estimator::ParticleFilter<double> > filter_ptr = filter.lock();

                if( !filter_ptr )
                    return;

                L3::WriteLock lock( this->mutex );
                hypotheses.assign( filter_ptr->hypotheses.begin(), filter_ptr->hypotheses.end() );
                lock.unlock();
            }
        };



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
                    CoordinateSystem( *it, 1 ).onDraw3D( g );

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
                    g.addVertex(x, y*100*2);
                    counter++;
                }

            }

            std::vector< double > weights;
            std::vector< double > hypotheses;

            void update()
            {
                std::vector< double >  _weights( weights.begin(), weights.end() );
                std::vector< double > _hypotheses( hypotheses.begin(), hypotheses.end() );

                //L3::SE3  current

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

        struct ParticleWeightOverview : glv::Table, Updateable, Lockable
        {
            ParticleWeightOverview( boost::shared_ptr< L3::Estimator::ParticleFilter<double> > algorithm, Updater* updater = NULL ) : glv::Table( "x," ), algorithm(algorithm)
            {
                {
                    // X
                    x_weight = boost::make_shared< ParticleWeightVisualiser >();

                    boost::shared_ptr< glv::Plot > plot( new glv::Plot( glv::Rect( 525, 2*80), *x_weight ) );
                    *this << *plot;

                    plots.push_back( plot );

                    plot->disable( glv::Controllable );
                    plot->showNumbering(true);
                    plot->range( -2, 2, 0 );
                    plot->range( 0, 1, 1 );
                }

                {
                    // Y
                    y_weight = boost::make_shared< ParticleWeightVisualiser >( );

                    boost::shared_ptr< glv::Plot > plot( new glv::Plot( glv::Rect( 525, 80), *y_weight ) );
                    *this << *plot;

                    plots.push_back( plot );

                    plot->disable( glv::Controllable );
                    plot->showNumbering(true);

                    plot->range( -.5, .5, 0 );
                }

                {
                    // Z
                    theta_weight = boost::make_shared< ParticleWeightVisualiser >();

                    boost::shared_ptr< glv::Plot > plot( new glv::Plot( glv::Rect( 525, 80), *theta_weight ) );
                    *this << *plot;

                    plots.push_back( plot );

                    plot->disable( glv::Controllable );
                    plot->showNumbering(true);

                    plot->range( -5, 5, 0 );
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

                L3::ReadLock master( this->mutex );
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

                if ( weights.empty() )
                    return;

                int counter = 0;
                for( std::vector<L3::SE3>::iterator it = hypotheses.begin();
                        it != hypotheses.end();
                        it++) 
                {
                    x_weight->hypotheses[counter]= it->X()-current_prediction.X();
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
        ParticleFilterVisualiser::ParticleFilterVisualiser( boost::shared_ptr< L3::Estimator::ParticleFilter<double> > algorithm, Updater* updater, boost::shared_ptr< Composite> composite )  
            : composite(composite)
        {
            if( !updater )
                throw std::exception();

            this->updater = updater;

            /*
             *  Stand-alone components
             */
            /*
             *Weight visualiser     
             */
            //TODO
            //  How can we make this look better? Everything is looking very homogeneous
            
            //weight_visualiser = boost::make_shared< ParticleWeightOverview >( algorithm, updater );
            //*this << *weight_visualiser;
            //updater->operator<<( (boost::dynamic_pointer_cast< Updateable >(weight_visualiser)).get() );
            //updateables.push_back(dynamic_cast< Updateable* >(weight_visualiser.get() ) ); 
            //views.push_back( weight_visualiser );

            /*
             *Overhead visualiser
             */
            boost::shared_ptr< ParticleVisualiser > particle_visualiser = boost::make_shared< ParticleVisualiser >( algorithm );
            views.push_back( boost::dynamic_pointer_cast< glv::View >( particle_visualiser ) );
            *this << *particle_visualiser;

            views.push_back( particle_visualiser );

            /*
             *Controls
             */
            //1. Num particles 
            boost::shared_ptr< glv::Slider > num_particles = boost::make_shared< glv::Slider > ();
            num_particles->interval( 50, 1000 );
            num_particles->attachVariable( algorithm->num_particles );
            *this  << *num_particles;
            variables.push_back( num_particles );

            boost::shared_ptr< glv::Label > num_particles_label = boost::make_shared< glv::Label >( "#Particles" );
            num_particles_label->pos( glv::Place::CL, 5, 0 ).anchor( glv::Place::CR ); 
            *num_particles << *num_particles_label ;
            this->labels.push_back( num_particles_label ); 

            //2. Linear process noise
            boost::shared_ptr< glv::Slider > linear_uncertainty = boost::make_shared< glv::Slider > ();
            linear_uncertainty->interval( 0, 3 );
            linear_uncertainty->attachVariable( algorithm->linear_uncertainty );
            *this  << *linear_uncertainty;
            variables.push_back( linear_uncertainty );

            boost::shared_ptr< glv::Label > linear_uncertainty_label = boost::make_shared< glv::Label >( "Lin. Vel. uncertainty (m2/s2)" );
            linear_uncertainty_label->pos( glv::Place::CL, 5, 0 ).anchor( glv::Place::CR ); 
            *linear_uncertainty << *linear_uncertainty_label ;
            this->labels.push_back( linear_uncertainty_label ); 

            //3. Rotational process noise
            boost::shared_ptr< glv::Slider > rotational_uncertainty = boost::make_shared< glv::Slider > ();
            rotational_uncertainty->interval( 0, 1 );
            rotational_uncertainty->attachVariable( algorithm->rotational_uncertainty );
            *this  << *rotational_uncertainty;
            variables.push_back( rotational_uncertainty );

            boost::shared_ptr< glv::Label > rotational_uncertainty_label = boost::make_shared< glv::Label >( "Rot. Vel. uncertainty (rad2/s2)" );
            rotational_uncertainty_label->pos( glv::Place::CL, 5, 0 ).anchor( glv::Place::CR ); 
            *rotational_uncertainty << *rotational_uncertainty_label ;
            this->labels.push_back( rotational_uncertainty_label ); 


            /*
             *3D components
             */
            boost::shared_ptr< Composite > composite_ptr = this->composite.lock();

            if( composite_ptr )
            {
                //composite->components.remove( dynamic_cast<L3::Visualisers::Leaf*>( particle_filter_renderer.get() ) );
                particle_filter_renderer = boost::make_shared< ParticleFilterRendererLeaf >( boost::dynamic_pointer_cast< L3::Estimator::ParticleFilter<double> >( algorithm ) );

                composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(particle_filter_renderer.get() ))); 
                leafs.push_back( dynamic_cast<L3::Visualisers::Leaf*>(particle_filter_renderer.get() ) );

                updater->operator<<( dynamic_cast< Updateable* >(particle_filter_renderer.get() ) );
                updateables.push_back(dynamic_cast< Updateable* >(particle_filter_renderer.get() ) ); 

            }
            else
                std::cerr << "No composite pointer passed!" << std::endl;

            /*
             *Finalize
             */

            this->arrange();
            this->fit();
        }

        ParticleFilterVisualiser::~ParticleFilterVisualiser()
        {
            // Remove children from the updates list  
            for( std::list< Updateable* >::iterator updateable_it =  updateables.begin();
                    updateable_it != updateables.end();
                    updateable_it++ )
            {
                updater->remove( *updateable_it );
            }

            boost::shared_ptr< Composite > composite_ptr = this->composite.lock();

            if( composite_ptr )
            {
                for( std::list< Leaf* >::iterator leaf_it =  leafs.begin();
                        leaf_it != leafs.end();
                        leaf_it++ )
                {
                    std::list < Leaf* >::iterator it 
                        = std::find( composite_ptr->components.begin(), 
                                composite_ptr->components.end(), 
                                *leaf_it
                                );

                    if( it != composite_ptr->components.end() )
                        composite_ptr->components.erase( it );

                }
            }
        }
    }
}
