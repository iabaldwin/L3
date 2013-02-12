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
        //stretch(1,1); 
    }


	virtual void onDraw3D(glv::GLV& g)
    {
    }
	virtual void onDraw2D(glv::GLV& g)
    {
    }


};

struct Leaf 
{

    virtual void onDraw3D( glv::GLV& g )
    {}

    virtual void onDraw2D( glv::GLV& g )
    {}

};


/*
 *Cloud Renderer
 */
template <typename T>
struct CloudRenderer : Leaf
{
    CloudRenderer( L3::PointCloud<T>* CLOUD ) : cloud(CLOUD)
    {
        colors = new glv::Color[cloud->size()];
        vertices = new glv::Point3[cloud->size()];
   
        // Build the cloud
        typename std::vector< Point<T> >::iterator it;
        it = cloud->data.begin();

        int counter = 0;
        while( it != cloud->data.end() )
        {
            vertices[counter]( (*it).x, (*it).y, (*it).z); 
            colors[counter] = glv::HSV(0.6, .1, (*it).z*0.45+0.55);
            it++; counter++;
        }
    
    };

    ~CloudRenderer()
    {
        delete [] colors;
        delete [] vertices;
    }

    void onDraw3D( glv::GLV& g )
    {
        glv::draw::paint( glv::draw::Points, vertices, colors, cloud->size());
    }
    
    glv::Color* colors;
    glv::Point3* vertices;
    L3::PointCloud<T>* cloud;

};

/*
 *Iterator renderer
 */
struct IteratorRenderer : Leaf
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
            it++; 
            counter++;
        }
        
        glv::draw::paint( glv::draw::Points, vertices, colors, counter );


        //far( 200 );
        delete [] colors;
        delete [] vertices;
    }

    L3::Iterator* iterator;

};


/*
 *Pose chain renderer
 */
typedef std::vector<L3::Pose*>::iterator POSE_CHAIN_ITERATOR;
struct PoseChainRenderer : Leaf
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

    }

    ~PoseChainRenderer()
    {
        delete [] colors;
        delete [] vertices;
    }

    std::vector<L3::Pose*>& poses;
   
    void onDraw3D(glv::GLV& g)
    {
        glv::draw::paint( glv::draw::Points, vertices, colors, poses.size() );
    }

    void onDraw2D(glv::GLV& g)
    {
    }

};

struct Composite : glv::View3D{

    Composite()
    {
        stretch(1,1); 
        far( 300 );
    }

    virtual void onDraw3D(glv::GLV& g)
    {
        std::list<Leaf*>::iterator it = components.begin();
        glv::draw::translateZ( -250 );

        while( it != components.end() )
        {
            (*it)->onDraw3D( g );
            it++;
        }
    
    }

	virtual void onDraw2D( glv::GLV& g)
    {
    }

    Composite& operator<<( Leaf& leaf )
    {
        components.push_back( &leaf );
        return *this;
    }

    std::list<Leaf*> components; 

};


}
}
#endif
