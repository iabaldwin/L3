#include "Widgets.h"
#include "Tracking.h"

namespace L3
{
namespace Visualisers
{

    template <typename T>
    struct Resetter : glv::View
    {
        Resetter( T& val ) 
            : glv::View( glv::Rect(20,20) ),
                val(val)
        {
            initial_val = val;
        }

        T& val;
        T initial_val;

        bool reset;

        void onDraw( glv::GLV& g )
        {
            if ( reset )
            {
                val = initial_val;
                reset = !reset;
            }
        }
    };

    void DensityController::attachHistogram( boost::shared_ptr< L3::Histogram<double> > histogram )
    {

        if( boost::shared_ptr< L3::HistogramUniformDistance <double> > hist_ptr  = 
                boost::dynamic_pointer_cast< L3::HistogramUniformDistance<double> >( histogram )) 
        {

            boost::shared_ptr< glv::NumberDialers  > experience_densities = 
                boost::make_shared< glv::NumberDialers > (2,1, .1, 10.0, 1, 1);

            boost::shared_ptr< glv::Label > label = boost::make_shared < glv::Label >();
            label->setValue( "Density (bins/m)");
            label->pos( glv::Place::CL, 0, 0 ).anchor( glv::Place::CR ); 
            *experience_densities << *label;

            labels.push_back( label);

            experience_densities->attachVariable( hist_ptr->bins_per_metre );
            experience_densities->fit();

            widgets.push_back( experience_densities );
            *this << *experience_densities;

        }

        this->fit();

    }

    FundamentalControls::FundamentalControls() : glv::Table( "x x,", 5, 5  )
    {


        // Alpha control
        boost::shared_ptr< glv::Widget > alpha_control = boost::make_shared< glv::Slider >();
        *this << *alpha_control;

        boost::shared_ptr< glv::Label > alpha_label = boost::make_shared< glv::Label >("Alpha" );
        alpha_label->pos( glv::Place::CL, 0, 0 ).anchor( glv::Place::CR );
        *alpha_control << *alpha_label;

        widgets.push_back( alpha_control );
        this->labels.push_front( alpha_label );

        // Beta control
        boost::shared_ptr< glv::Widget > beta_control = boost::make_shared< glv::Slider >();
        *this << *beta_control;

        boost::shared_ptr< glv::Label > beta_label = boost::make_shared< glv::Label >("Beta" );
        beta_label->pos( glv::Place::CL, 0, 0 ).anchor( glv::Place::CR );
        *beta_control << *beta_label;

        widgets.push_back( beta_control );
        this->labels.push_front( beta_label );

        // Velocity bias
        boost::shared_ptr< glv::Widget > velocity_bias = boost::make_shared< glv::Slider >();
        *this << *velocity_bias;

        velocity_bias->interval( .1,2 );
        
        boost::shared_ptr< glv::Label > velocity_bias_label = boost::make_shared< glv::Label >("Velocity bias" );
        velocity_bias_label->pos( glv::Place::CL, 0, 0 ).anchor( glv::Place::CR );
        *velocity_bias << *velocity_bias_label;

        widgets.push_back( velocity_bias );
        this->labels.push_front( velocity_bias_label );

        // Velocity bias reset
        boost::shared_ptr< glv::Button > velocity_bias_reset = boost::make_shared< glv::Button >( glv::Rect(20), false );
        *this << *velocity_bias_reset;

        widgets.push_back( velocity_bias_reset );

        this->fit();
        this->arrange();

    }

    void FundamentalControls::associateVelocitySource( boost::shared_ptr< FilteredScanMatchingVelocityProvider> ptr)
    {
        widgets[0]->attachVariable( ptr->_linear_velocity_filter->alpha );
        widgets[1]->attachVariable( ptr->_linear_velocity_filter->beta );
        widgets[2]->attachVariable( ptr->scaling_bias );
 
        boost::shared_ptr< Resetter<float> > r = boost::make_shared< Resetter< float > >( boost::ref( ptr->scaling_bias ) ); 

        *widgets[2] << *r ;
        views.push_back( r );
        widgets[3]->attachVariable( r->reset );
    }

}
}
