#include "Widgets.h"
#include "Tracking.h"

namespace L3
{
namespace Visualisers
{
    template <typename T>
        struct ResettableSlider : glv::View
    {
        ResettableSlider( const std::string& label_text, T lower=1, T upper=10 )  
            : glv::View( glv::Rect(150,20) ),
            reset( false )
        {

            table = boost::make_shared< glv::Table >( "x x x,", 5, 0 );

            slider = boost::make_shared< glv::Slider >();
    
            slider->interval( upper, lower );

            label = boost::make_shared< glv::Label >( label_text );
            label->pos( glv::Place::CL, 0, 0 ).anchor( glv::Place::CR );
            
            //reset_toggle = boost::make_shared< glv::Button >( glv::Rect(20), false );
            reset_toggle = boost::make_shared< glv::Button >( glv::Rect(20), true );

            *table <<  *slider;
            *table <<  *reset_toggle;
            *table <<  *label;
  
            table->arrange();
            table->fit();

            *this << *table;
       
            this->fit();
       
            this->disable( glv::DrawBorder );
        }

        T initial_val;

        bool reset;

        boost::shared_ptr< glv::Table > table;
        boost::shared_ptr< glv::Label > label;
        boost::shared_ptr< glv::Widget > slider;
        boost::shared_ptr< glv::Button > reset_toggle;

        void onDraw( glv::GLV& g )
        {
            if ( reset_toggle->getValue() )
                slider->setValue( initial_val ); 
        }

        void attach( T& t )
        {
            this->initial_val = t;
            slider->attachVariable( t ); 
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

    FundamentalControls::FundamentalControls() : glv::Table( "x,", 0, 0  )
    {
        // Border visibility
        this->enable( glv::DrawBorder );

        // Master holder
        boost::shared_ptr< glv::View > view_sizer = boost::make_shared< glv::View >( glv::Rect( 555, 120 ) );
        
        // Label
        boost::shared_ptr< glv::Label > label = boost::make_shared< glv::Label >();
        label->setValue( "Filter controls" );
        label->pos( glv::Place::BL, 0, 0 ).anchor( glv::Place::BL ); 
        (*view_sizer) << *label;

        labels.push_back( label );

        *this << *view_sizer;
        views.push_back(view_sizer);

        // Sub-table
        boost::shared_ptr< glv::Table > sub_table = boost::make_shared< glv::Table >( "x," );
        *view_sizer  << *sub_table;

        // Alpha control
        boost::shared_ptr< glv::View > alpha_control = boost::make_shared< ResettableSlider<float> >( "Alpha", .01, 1.0 );
        *sub_table<< *alpha_control;
        views.push_back( alpha_control );

        // Beta control
        boost::shared_ptr< glv::View > beta_control = boost::make_shared< ResettableSlider<float> >( "Beta ", .001, 0.1 );
        *sub_table<< *beta_control;
        views.push_back( beta_control );

        // Velocity bias
        boost::shared_ptr< glv::View > velocity_bias = boost::make_shared< ResettableSlider<float> >( "Bias ", .1, 2  );
        *sub_table<< *velocity_bias;
        views.push_back( velocity_bias );

        sub_table->fit();
        sub_table->arrange();

        view_sizer->fit();

        this->fit();
        this->arrange();
        
        views.push_back( sub_table );
    }

    void FundamentalControls::associateVelocitySource( boost::shared_ptr< FilteredScanMatchingVelocityProvider> ptr)
    {
        boost::dynamic_pointer_cast< ResettableSlider<float> >(views[1])->attach( ptr->_linear_velocity_filter->alpha );    // Alpha
        boost::dynamic_pointer_cast< ResettableSlider<float> >(views[2])->attach( ptr->_linear_velocity_filter->beta );     // Beta
        boost::dynamic_pointer_cast< ResettableSlider<float> >(views[3])->attach( ptr->scaling_bias );                      // Bias
    }

}
}
