#ifndef L3_COMPONENTS_H
#define L3_COMPONENTS_H

#include <GLV/glv.h>
#include <GLV/glv_binding.h>
#include <GLV/glv_util.h>

#include <iostream>
#include "Datatypes.h"
#include "Definitions.h"
#include "Iterator.h"
#include "Pointcloud.h"

namespace L3
{
namespace Visualisers
{

struct Component : glv::View3D{

    Component()
    {
        /*
         *If you don't have an active
         *    view, then the onDraw
         *    methods are not called. 
         *    However, this stretches
         *    to the full size of the
         *    element.
         */
        stretch(1,1); 
    }
	
	virtual void onDraw3D(glv::GLV& g)
    {
    }
	virtual void onDraw2D(glv::GLV& g)
    {
    }


};

typedef std::vector<L3::Pose*>::iterator POSE_CHAIN_ITERATOR;

/*
 *Cloud Renderer
 */
template <typename T>
struct CloudRenderer : Component
{
    CloudRenderer( L3::PointCloud<T>* CLOUD ) : cloud(CLOUD)
    {
        colors = new glv::Color[cloud->size()];
        vertices = new glv::Point3[cloud->size()];
   
        // Build the cloud
        typedef typename L3::PointCloud<T>::PointCloudIterator it; 
        //typename std::vector<T>::iterator it = cloud->data.begin();
        //std::vector<T>::iterator it 

        //while( it != cloud->data.end() )
        //{
            //vertices[counter]( (*it)[0], (*it)[1], (*it)[2]); 
            //it++;
        //}
    
    };

    ~CloudRenderer()
    {
        delete [] colors;
        delete [] vertices;
    }

    void onDraw3D( glv::GLV& g )
    {
        //What is the type?
        std::cout << cloud->size() << std::endl; 
    }
    
    glv::Color* colors;
    glv::Point3* vertices;
    L3::PointCloud<T>* cloud;

};

/*
 *Iterator renderer
 */
struct IteratorRenderer : Component
{

    IteratorRenderer( L3::Iterator* ITERATOR )  : iterator(ITERATOR )
    {
    }

    void onDraw3D( glv::GLV& g )
    {
        // Update the iterator
        iterator->update( 1.0 );

        // Get the swathe
        SWATHE* swathe = iterator->getSwathe();

        // Reserve
        glv::Color* colors = new glv::Color[iterator->numScans()];
        glv::Point3* vertices = new glv::Point3[iterator->numScans()];;

        SWATHE::iterator it = swathe->begin();

        int counter = 0;
        while( it != swathe->end() )
        {
            vertices[counter]( (*it).first->x, (*it).first->y, 0 );
            it++; counter++;
        }
        
        glv::draw::translateZ( -190 );
        glv::draw::paint( glv::draw::Points, vertices, colors, counter );


        far( 200 );
        delete [] colors;
        delete [] vertices;
    }


    L3::Iterator* iterator;
};


/*
 *Pose chain renderer
 */

struct PoseChainRenderer : Component
{

    glv::Color* colors;
    glv::Point3* vertices;

    PoseChainRenderer( std::vector<L3::Pose*>& POSES ) : poses(POSES)
    {
        colors = new glv::Color[poses.size()];
        vertices = new glv::Point3[poses.size()];

        int counter = 0;
        for( POSE_CHAIN_ITERATOR it=poses.begin(); it < poses.end(); it++ )
        {
            vertices[counter]( (*it)->x, (*it)->y, 0 );
            colors[counter] = glv::HSV(0.6, .1, 0.45+0.55);

            counter++;
        }

        // Far clip
        far( 200 );
    }

    ~PoseChainRenderer()
    {
        delete [] colors;
        delete [] vertices;
    }

    std::vector<L3::Pose*>& poses;
   
    void onDraw3D(glv::GLV& g)
    {
        static int k = 0;
        glv::draw::rotateZ( k++ );
        glv::draw::translateZ( -190 );
        glv::draw::paint( glv::draw::Points, vertices, colors, poses.size() );
    }

    void onDraw2D(glv::GLV& g)
    {
    }

};

}
}
#endif
