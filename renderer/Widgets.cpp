#include "Widgets.h"
#include "Tracking.h"

namespace L3
{
namespace Visualisers
{

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

    FundamentalControls::FundamentalControls() : glv::Table( "x," )
    {
        // Alpha control
        boost::shared_ptr< glv::Widget > alpha_control = boost::make_shared< glv::Slider >();
        *this << *alpha_control;

        boost::shared_ptr< glv::Label > alpha_label = boost::make_shared< glv::Label >("Alpha" );
        alpha_label->pos( glv::Place::BR, 0, 0 ).anchor( glv::Place::BR );
        *alpha_control << *alpha_label;

        widgets.push_back( alpha_control );
        this->labels.push_front( alpha_label );

        boost::shared_ptr< glv::Widget > beta_control = boost::make_shared< glv::Slider >();
        *this << *beta_control;

        boost::shared_ptr< glv::Label > beta_label = boost::make_shared< glv::Label >("Beta" );
        beta_label->pos( glv::Place::BR, 0, 0 ).anchor( glv::Place::BR );
        *beta_control << *beta_label;

        widgets.push_back( beta_control );
        this->labels.push_front( beta_label );



        this->fit();
        this->arrange();

    }

    void FundamentalControls::addFilter( boost::shared_ptr< FilteredScanMatchingVelocityProvider> ptr)
    {
        widgets[0]->attachVariable( ptr->_linear_velocity_filter->alpha );
        widgets[1]->attachVariable( ptr->_linear_velocity_filter->beta );
    }
}
}
