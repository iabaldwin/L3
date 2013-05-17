
#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include <boost/scoped_ptr.hpp>

#include "L3.h"
#include "Visualisers.h"
#include "Components.h"
#include "Controls.h"
#include "Imagery.h"
#include "QueryInterface.h"

int main (int argc, char ** argv)
{

    //glv::GLV top;
    L3::Visualisers::L3GLV top;

    glv::Window win(1400, 800, "Soaring");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    L3::Visualisers::Composite              composite( glv::Rect( 1000, 600) );
    L3::Visualisers::BasicPanController     controller(composite.position);
    L3::Visualisers::Grid                   grid;
            
    L3::Visualisers::MouseQuerySelect query(  &composite );
    L3::Visualisers::WASDManager selection_manager( &query );

    L3::Configuration::Begbroke begbroke;
    begbroke.loadDatum();


    //Add trajectories

    boost::scoped_ptr <L3::IO::BinaryReader< L3::SE3 > > pose_reader( ( new L3::IO::BinaryReader<L3::SE3>() ) ) ;
    boost::scoped_ptr< L3::Dataset > dataset( new L3::Dataset( "/Users/ian/code/datasets/2012-02-06-13-15-35mistsnow/") );

    if (!pose_reader->open( dataset->path() + "/OxTS.ins" ) )
        exit(-1);

    // Read all the poses
    pose_reader->read();

    // Create pose sequence
    boost::shared_ptr< std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > > > poses( new std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > > () );

    // And extract them
    pose_reader->extract( *poses );

    for( std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > >::iterator it = poses->begin();
            it != poses->end();
            it++ )
        it->second->Z( 0 );

    // Here, we add a new component::leaf with the poses
    boost::shared_ptr< L3::Visualisers::PoseSequenceRenderer > sequence( new L3::Visualisers::PoseSequenceRenderer( poses ) );
    boost::shared_ptr< L3::Visualisers::LocaleRenderer > locale_renderer = L3::Visualisers::LocaleRendererFactory::build( begbroke );


    // Add Boxes
    L3::Visualisers::SelectableLeaf* renderer = new L3::Visualisers::SelectableLeaf( 20, 20, 20 );
    L3::Visualisers::SelectableLeaf* renderer2 = new L3::Visualisers::SelectableLeaf( 5, 10, 15 );

    top << ( composite << grid << *renderer << *renderer2 << *locale_renderer << *sequence );

    composite.addController( dynamic_cast<L3::Visualisers::Controller*>( &controller ) );

    win.setGLV(top);
    glv::Application::run();
}


