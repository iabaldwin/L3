#include <iostream>
#include <fstream>

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include <boost/date_time/posix_time/posix_time_types.hpp>

#include "L3.h"
#include "Visualisers.h"

struct VisualiserRunner : L3::Visualisers::Leaf, L3::TemporalRunner
{

    VisualiserRunner( double start_time, L3::ConstantTimeIterator<L3::LHLV>* iterator, L3::PoseWindower* windower ) 
        : time(start_time), iterator(iterator), windower(windower)
    {
    }

    double                              time;
    L3::Predictor                       predictor;
    L3::PoseWindower*                   windower;
    L3::ConstantTimeIterator<L3::LHLV>* iterator; 

    void onDraw3D( glv::GLV& g )
    {
        // Update 
        this->update( time += .5 );

        L3::SE3 current = windower->operator()();
        L3::SE3 predicted = L3::SE3::ZERO();

        predictor.predict( predicted, current, iterator->window.begin(), iterator->window.end() );

        std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > >::iterator it = predictor.chain.begin();
        while( it != predictor.chain.end() )
        {
            L3::Visualisers::CoordinateSystem( it->second ).onDraw3D( g ); 
            it++;
        }
    }

};

int main (int argc, char ** argv)
{
    /*
     *  L3
     */
    L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-27-11-17-51Woodstock-All/" );
    if( !( dataset.validate() && dataset.load() ) )
        throw std::exception();

    L3::Configuration::Mission mission( dataset );

    // Constant time iterator over poses
    L3::ConstantTimeIterator< L3::SE3 >  pose_iterator( dataset.pose_reader );

    // Constant time iterator over poses
    L3::ConstantTimeIterator< L3::LHLV >  LHLV_iterator( dataset.LHLV_reader );

    // Constant time iterator over LIDAR
    L3::ConstantTimeIterator< L3::LMS151 > LIDAR_iterator( dataset.LIDAR_readers[ mission.declined ] );

    // Responsible for building poses
    L3::ConstantTimeWindower<L3::SE3>  pose_windower( &pose_iterator );

    // Generating swathe
    L3::SwatheBuilder swathe_builder( &pose_windower, &LIDAR_iterator );

    /*
     *Visualisation
     */
    glv::GLV top;
    glv::Window win(1400, 800, "Visualisation::PointCloud");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    // Point cloud renderer
    L3::Visualisers::Composite              composite;
    L3::Visualisers::BasicPanController     controller;
    L3::Visualisers::Grid                   grid;
    L3::Visualisers::SwatheRenderer         swathe_renderer( &swathe_builder ); 
    L3::Visualisers::PoseWindowerRenderer   pose_renderer( &pose_windower ); 

    composite.addController( dynamic_cast<L3::Visualisers::Controller*>( &controller ) ).stretch(1,1);

    // Add watchers
    composite << swathe_renderer << pose_renderer <<  grid;

    // Add runner
    VisualiserRunner runner( dataset.start_time, &LHLV_iterator, &pose_windower ) ;

    runner << &swathe_builder << &pose_windower << &LHLV_iterator;

    top << (composite << runner);

    win.setGLV(top);

    try
    {
        glv::Application::run();
    }
    catch( L3::end_of_stream& e )
    {
        std::cout << "Done" << std::endl;
    }

}

