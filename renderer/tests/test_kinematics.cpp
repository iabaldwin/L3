#include <iostream>
#include <fstream>

#include <glv.h>
#include <glv_binding.h>
#include <glv_util.h>

#include "L3.h"
#include "Visualisers.h"

//template <typename T>
//struct ScanGenerator 
//{
    //void operator()( std::vector<T>& data )
    //{
        //data.resize( 541 );

        //for( typename std::vector<T>::iterator it = data.begin(); it != data.end(); it ++ )
            //*it = random() % 50;
    //}
//};

//struct Kinematics : L3::Visualisers::Leaf
//{

    //Kinematics()
    //{
        //ScanGenerator<float> generator;
       
        //scan.reset( new L3::LMS151() );
        //generator( scan->ranges );
 
        //pose.reset( new L3::SE3( 20, 20, 0, 0, 0, 0 ) );

        //cloud = new L3::PointCloud<double>();

        //L3::SE3 calibration = L3::SE3::ZERO();

        //projector = new L3::Projector<double>( &calibration, cloud );
  
        //vertices = new glv::Point3[541];
        //colors = new glv::Color[541];

        ////controllable = new L3::Visualisers::Controllable( pose );
   
        ////c = new L3::Visualisers::CoordinateSystem( *pose );
    //}
    
    //SWATHE swathe;

    //boost::shared_ptr< L3::SE3 >        pose;
    //boost::shared_ptr< L3::LMS151 >     scan;

    //L3::Projector<double>*              projector;
    //L3::PointCloud<double>*             cloud;


    //L3::Visualisers::Controllable*      controllable;
    //glv::Point3*                        vertices;
    //glv::Color*                         colors;
        
    //L3::Visualisers::CoordinateSystem*  c;

    //void onDraw3D( glv::GLV& g )
    //{
        //swathe.clear();
        //swathe.push_back( std::make_pair( pose, scan ) );

        //projector->project( swathe );

        //L3::PointCloud<double>::ITERATOR it = cloud->begin();
       
        //int counter = 0;
        //while( it != cloud->end() )
        //{
            //vertices[counter++]( it->x, it->y, it->z );
            //it++;
        //}
   
        //glv::draw::paint( glv::draw::Points, vertices, colors, counter );
   
        //c->onDraw3D( g );
   
        //controllable->update();
    //}

//};

int main (int argc, char ** argv)
{
    /*
     *L3
     */
    //L3::Dataset dataset( "/Users/ian/code/datasets/2012-02-27-11-17-51Woodstock-All/" );
    //if( !( dataset.validate() && dataset.load() ) )
        //throw std::exception();
   
    //L3::Configuration::Mission mission( dataset );

    //// Constant time iterator over poses
    //L3::ConstantTimeIterator< L3::SE3 >  pose_iterator( dataset.pose_reader );
    
    //// Constant time iterator over LIDAR - declined
    //L3::ConstantTimeIterator< L3::LMS151 > declined_lidar( dataset.LIDAR_readers[ mission.declined ] );
    
    //// Constant time iterator over LIDAR - horizontal
    //L3::ConstantTimeIterator< L3::LMS151 > horizontal_lidar( dataset.LIDAR_readers[ mission.horizontal ] );

    //double time = dataset.start_time;

    //// Windowed pose producer
    //L3::ConstantTimeWindower<L3::SE3> pose_windower( &pose_iterator );
    
    //// Swathe builder
    //L3::SwatheBuilder swathe_builder( &pose_windower, &declined_lidar );
    
    //// Build runner
    //L3::TemporalRunner t;

    /*
     *Visualisation
     */
    //glv::GLV top;
    //glv::Window win(1400, 800, "Visualisation::ScanRenderer");

    //// Colors
    //top.colors().set(glv::Color(glv::HSV(0.6,0.2,0.6), 0.9), 0.4);

    //// Point cloud renderer
    //L3::Visualisers::Composite      composite;
    //L3::Visualisers::Controller*    controller = new L3::Visualisers::FPSController( composite.position );
    //L3::Visualisers::Grid           grid;

    //Kinematics k;

    //// Compose 
    //top << (composite << k << grid) ;

    //win.setGLV(top);

    //try
    //{
        //glv::Application::run();
    //}
    //catch( ... )
    //{
    //}
}
