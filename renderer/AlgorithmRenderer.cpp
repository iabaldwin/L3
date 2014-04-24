#include "AlgorithmRenderer.h"
#include "RenderingUtils.h"

namespace L3
{
    namespace Visualisers
    {
        struct TraversalVisualiser : Updateable, Lockable
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
                L3::ReadLock lock( this->mutex );

                ColorInterpolator interpolator;

                std::vector<glv::Point3> vertices(evaluations.size());
                std::vector<glv::Color> colors(evaluations.size());

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
                glv::draw::paint(glv::draw::Points, &vertices[0], &colors[0], counter);
                glv::draw::pointSize(1);
                glv::draw::lineWidth(.1);
                glv::draw::paint(glv::draw::LineStrip, &vertices[0], &colors[0], counter);

                CoordinateSystem( evaluations.front(), 1 ).onDraw3D(g);
                CoordinateSystem( evaluations.back(), 1 ).onDraw3D(g);
                lock.unlock(); 
            }            

            virtual void update()
            {
                L3::WriteLock lock( this->mutex );
                boost::shared_ptr< L3::Estimator::Minimisation<double> > algorithm_ptr = algorithm.lock();

                if( !algorithm_ptr)
                    return;
                
                evaluations.assign( algorithm_ptr->evaluations.begin(), algorithm_ptr->evaluations.end() );
                lock.unlock(); 
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
                label.setValue( "Cost-function evaluations");
                label.pos( glv::Place::TL, 0, 0 ).anchor( glv::Place::TL ); 
                *this  << label;
            }

            glv::Label label;

            void onDraw3D( glv::GLV& g )
            {
                far(150);

                if( !evaluations.empty() )
                    glv::draw::translate( -1*evaluations.back().X(), -1*evaluations.back().Y(), -5 );

                TraversalVisualiser::onDraw3D(g);
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

            boost::shared_ptr< glv::Label > iterations_label = boost::make_shared< glv::Label >();
            iterations_label->setValue( "Algorithm iterations");
            iterations_label->pos( glv::Place::BL, 0, 0 ).anchor( glv::Place::TL ); 
            *plot << *iterations_label;
            labels.push_back( iterations_label ) ;

            (*this) << *plot;
            views.push_back( plot );

            // Estimation traversals
            boost::shared_ptr< glv::View > traversal_view = boost::dynamic_pointer_cast<glv::View>( boost::make_shared< TraversalVisualiserView >( algorithm ) );
            updater->operator<< (dynamic_cast< Updateable* >(traversal_view.get() ) );
            updateables.push_back(dynamic_cast< Updateable* >(traversal_view.get() ) ); 
            views.push_back( traversal_view );

            (*this) << *traversal_view;

            // Controls
            //1. Tolerance
            boost::shared_ptr< ResettableSlider<double> > tolerance 
                = boost::make_shared< ResettableSlider<double> > ( "Tolerance", .001, .1 );
            tolerance->attach( algorithm->tolerance );
            *this  << *tolerance;
            views.push_back( tolerance );



            //2. Iterations
            boost::shared_ptr< ResettableSlider<int> > iterations 
                = boost::make_shared< ResettableSlider<int> > ( "Iterations", 1, 120 );
            iterations->attach( algorithm->max_iterations );
            *this  << *iterations;
            views.push_back( iterations );

            // Pyramid selector
            //boost::shared_ptr< glv::Buttons > pyramid_select = boost::make_shared< glv::Buttons >( glv::Rect(120, 20), 3, 1 );
            //pyramid_select->attachVariable( algorithm->pyramid_index, 0 );
            //*this << *pyramid_select;
            //widgets.push_back( pyramid_select );

            /*
             *3D components
             */

            // Estimation traversals : Leaf
            boost::shared_ptr< Composite > composite_ptr = this->composite.lock();

            if( composite_ptr )
            {
                traversal_leaf = boost::make_shared< TraversalVisualiserLeaf >( algorithm );

                composite->operator<<( *(dynamic_cast<L3::Visualisers::Leaf*>(traversal_leaf.get() ))); 
                leafs.push_back( dynamic_cast<L3::Visualisers::Leaf*>(traversal_leaf.get() ) );

                updater->operator<<( dynamic_cast< Updateable* >(traversal_leaf.get() ) );
                updateables.push_back(dynamic_cast< Updateable* >(traversal_leaf.get() ) ); 

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

            std::vector<glv::Point3> vertices(estimates.size());
            std::vector<glv::Color> colors(estimates.size());

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
            glv::draw::paint( glv::draw::Points, &vertices[0], &colors[0], ptr->pose_estimates->estimates.size() );
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
                L3::SE3 translation = *(algorithm_ptr->current_prediction);
                glv::draw::translate( -1*translation.X(), -1*translation.Y(), -10.0 );

                int counter = 0;
                std::vector<glv::Point3> vertices(hypotheses.size());
                std::vector<glv::Color> colors(hypotheses.size());

                for( L3::Estimator::ParticleFilter<double>::PARTICLE_ITERATOR it = hypotheses.begin();
                        it != hypotheses.end();
                        it++ )
                    CoordinateSystem( *it, 1 ).onDraw3D( g );

                glv::draw::paint( glv::draw::Points, &vertices[0], &colors[0], counter );
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
                L3::SE3 current_prediction = *(algorithm_ptr->current_prediction);
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
            boost::shared_ptr< ResettableSlider<int> > num_particles 
                = boost::make_shared< ResettableSlider<int> > ( "# Particles", 50, 1000 );
            num_particles->attach( algorithm->num_particles );
            *this  << *num_particles;
            views.push_back( num_particles );


            //2. Linear process noise
            boost::shared_ptr< ResettableSlider<double> > linear_uncertainty 
                = boost::make_shared< ResettableSlider<double> > ( "Lin. Vel. uncertainty (m2/s2)", 0, 3 );
            linear_uncertainty->attach( algorithm->linear_uncertainty );
            *this  << *linear_uncertainty;
            views.push_back( linear_uncertainty );

            //3. Rotational process noise
            boost::shared_ptr< ResettableSlider<double> > rotational_uncertainty 
                = boost::make_shared< ResettableSlider<double> > ( "Rot. Vel. uncertainty (rad2/s2)" , 0, .5 );
            rotational_uncertainty->attach( algorithm->rotational_uncertainty );
            *this  << *rotational_uncertainty;
            views.push_back( rotational_uncertainty );

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


        /*
         *  UKF visualiser
         */
       
        struct UKFPoseEstimate : Leaf
        {

            UKFPoseEstimate( const L3::SE3& pose, double size = 10.0 ) : pose(pose), size(size)
            {

            }
           
            double size;
            const L3::SE3& pose;

            void onDraw3D( glv::GLV& g )
            {
                CoordinateSystem( const_cast< L3::SE3& >(pose), size ).onDraw3D( g );
            }
        };

        struct UKFSigmaPoints 
        {
            UKFSigmaPoints(  boost::shared_ptr< L3::Estimator::UKF<double> > algorithm ) 
                : algorithm(algorithm)
            {
            }

            boost::weak_ptr< L3::Estimator::UKF<double> > algorithm ;

            void onDraw3D( glv::GLV& g )
            {
                boost::shared_ptr< L3::Estimator::UKF<double> > algorithm_ptr = algorithm.lock();

                if ( !algorithm_ptr )
                    return;

                std::vector< double > sigma_points( algorithm_ptr->sigma_points.begin(),
                        algorithm_ptr->sigma_points.end() );


                double* ptr = &sigma_points[0];

                int num_sigma_points = (2*3)+1;

                std::vector<glv::Point3> vertices(num_sigma_points);
                std::vector<glv::Color> colors(num_sigma_points);

                for( int i=0; i<num_sigma_points; i++ )
                {
                    double x1 = *ptr++;
                    double y1 = *ptr++;
                    double z1 = *ptr++;

                    vertices[i](x1, y1, z1 );
               
                }
               
                glv::draw::pointSize( 10 );
                glv::draw::paint(glv::draw::Points, &vertices[0], &colors[0], num_sigma_points);
            }

        };

        struct UKFSigmaPointsLeaf : UKFSigmaPoints, Leaf
        {
            UKFSigmaPointsLeaf(  boost::shared_ptr< L3::Estimator::UKF<double> > algorithm ) 
                : UKFSigmaPoints( algorithm )
            {
            }
            
            void onDraw3D( glv::GLV& g )
            {
                UKFSigmaPoints::onDraw3D(g);
            }
        };


        struct UKFSigmaPointsView : UKFSigmaPoints, glv::View3D
        {
            UKFSigmaPointsView(  boost::shared_ptr< L3::Estimator::UKF<double> > algorithm, const L3::SE3& pose  ) 
                : UKFSigmaPoints( algorithm ),
                glv::View3D( glv::Rect (250, 250 ) ),
                pose(pose) 
            {
            }
            
            const L3::SE3& pose;
        
            void onDraw3D( glv::GLV& g )
            {
                glv::draw::translate( -1*pose.X(), -1*pose.Y(), -10 );
                
                UKFSigmaPoints::onDraw3D(g);

            }

        };
        
        UKFVisualiser::UKFVisualiser( boost::shared_ptr< L3::Estimator::UKF<double> > algorithm, Updater* updater, boost::shared_ptr< Composite> composite )
            : composite(composite)
        {
            /*
             *  Associate updater
             */
            if( !updater )
                throw std::exception();

            this->updater = updater;

            // Re-configure arrangement
            this->arrangement( "x -, x x, " );
            this->paddingX( 0 );

            /*
             *Current estimate
             */
            boost::shared_ptr< Leaf > pose_renderer = 
                boost::make_shared< UKFPoseEstimate >( *(algorithm->current_prediction ) );

            composite->operator<<( *pose_renderer );
            this->leafs.push_back( pose_renderer );
      

            /*
             *  Integrated
             */
            boost::shared_ptr< Leaf > check_pose = 
                boost::make_shared< UKFPoseEstimate >( algorithm->prediction_model->check_pose );
            composite->operator<<( *check_pose );
            this->leafs.push_back( check_pose ); 

            /*
             *  Sigma points
             */
            //boost::shared_ptr< Leaf > sigma_points = 
                //boost::make_shared< UKFSigmaPointsLeaf >( algorithm );

            //composite->operator<<( *sigma_points );
            //this->leafs.push_back( sigma_points );

            
            /*
             *  Iterations
             */
            boost::shared_ptr< StatisticsPlottable<int> > plottable( new StatisticsPlottable<int>() ); 
            plottable->setVariable( algorithm->minimiser->algorithm_iterations );

            plottables.push_back( plottable );
            updater->operator<<( plottable.get() );
            updateables.push_back( plottable.get() );

            boost::shared_ptr< glv::Plot > plot( new glv::Plot( glv::Rect( 525, 80), *plottable ) );
            plot->disable( glv::Controllable );
            plot->range(0,100,0);
            plot->range(0,50*10,1);


            boost::shared_ptr< glv::Label > iterations_label = boost::make_shared< glv::Label >();
            iterations_label->setValue( "Algorithm iterations");
            iterations_label->pos( glv::Place::BL, 0, 0 ).anchor( glv::Place::TL ); 
            *plot << *iterations_label;

            (*this) << *plot;
            views.push_back( plot );


            /*
             *  Traversals
             */
            boost::shared_ptr< TraversalVisualiserView > minimisation_visualiser = 
                boost::make_shared< TraversalVisualiserView >( algorithm->minimiser );
            views.push_back( minimisation_visualiser );

            updater->operator<<( minimisation_visualiser.get() );
            updateables.push_back( minimisation_visualiser.get() );

            this->operator<<( *minimisation_visualiser );


            /*
             *  Weight stand-alone view
             */
            boost::shared_ptr< glv::View > sigma_points_viewer = 
                boost::make_shared< UKFSigmaPointsView >( algorithm, *(algorithm->current_prediction ) );

            views.push_back( sigma_points_viewer );

            this->operator<<( *sigma_points_viewer );

            // Frequency controller
            boost::shared_ptr< ResettableSlider<float> > frequency_controller = 
                boost::make_shared< ResettableSlider<float> >( "Frequency", .1, 40 ) ;
            frequency_controller->attach( algorithm->fundamental_frequency );
            *this  << *frequency_controller;
            views.push_back( frequency_controller );

            this->arrange();
            this->fit();
        }
    
        UKFVisualiser::~UKFVisualiser()
        {
            for( std::list< Updateable* >::iterator updateable_it =  updateables.begin();
                    updateable_it != updateables.end();
                    updateable_it++ )
            {
                updater->remove( *updateable_it );
            }

            boost::shared_ptr < Composite > composite_ptr = composite.lock();

            for( std::list< boost::shared_ptr< Leaf > >::iterator leaf_it =  leafs.begin();
                    leaf_it != leafs.end();
                    leaf_it++ )
            {
                std::list < Leaf* >::iterator it 
                    = std::find( composite_ptr->components.begin(), 
                            composite_ptr->components.end(), 
                            leaf_it->get()
                            );

                if( it != composite_ptr->components.end() )
                    composite_ptr->components.erase( it );

            }
        }
    }
}
