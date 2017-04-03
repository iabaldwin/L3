#include <iostream>

#include <glv.h>
#include <glv_binding.h>
#include <glv_util.h>

#include "L3.h"
#include "Visualisers.h"
#include "Components.h"

struct test_view : glv::View3D
{
    test_view( std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > > poses)
        : glv::View3D( glv::Rect( 100, 100 ) ), 
        poses(poses)
    {
        far(1000);
        this->stretch(1,1);
        counter = 0;
    }

    std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > > poses;
        
    L3::Visualisers::Grid grid;

    int counter;

    void onDraw3D(glv::GLV& g)
    {
       
        L3::SE3 origin;
        L3::Visualisers::CoordinateSystem( origin ).onDraw3D(g );
        
        L3::SE3 tform = *(poses[counter+=10].second);

        /*
         *<Rotation>
         */
        L3::Visualisers::transformCameraToPose( tform );
        /*
         *</Rotation>
         */

        grid.onDraw3D( g );

    
        for( std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > >::iterator it = poses.begin();
                it != poses.end();
                it++ )
        {
            L3::Visualisers::CoordinateSystem( *(it->second) ).onDraw3D(g);
        }

        if ( counter > poses.size() )
            counter = 0;
 
    }
};


int main (int argc, char ** argv)
{

    // Read a pose sequence
    boost::scoped_ptr <L3::IO::BinaryReader< L3::SE3 > > pose_reader( ( new L3::IO::BinaryReader<L3::SE3>() ) ) ;
    boost::scoped_ptr< L3::Dataset > dataset( new L3::Dataset( "/Users/ian/code/datasets/2012-04-16-20-05-30NightWoodstock1/" ) );
      
    if (!pose_reader->open( dataset->path() + "/OxTS.ins" ) )
        return (-1);

    // Read all the poses
    pose_reader->read();

    // Create pose sequence
    boost::shared_ptr< std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > > > poses( new std::vector< std::pair< double, boost::shared_ptr<L3::SE3> > > () );

    // And extract them
    pose_reader->extract( *poses );

    glv::GLV top;
    glv::Window win(1400, 800, "Soaring");

    // Colors
    top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);
    
    test_view view( *poses );
    
    top << ( view );

    win.setGLV(top);
    glv::Application::run();


}


