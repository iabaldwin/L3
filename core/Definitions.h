#ifndef L3_DEFINITIONS_H
#define L3_DEFINITIONS_H

#include <vector>
#include "Datatypes.h"

/*
 *  <vector>
 *  For parallelization, we require random access
 */

typedef std::vector< std::pair< L3::Pose*, L3::LMS151* > >  SWATHE;
//typedef std::vector<L3::Pose*>                              POSE_SEQUENCE;
//typedef std::vector<L3::LMS151*>                            LIDAR_SEQUENCE;

//template <typename T>
//struct SEQUENCE
//{
    //std::vector< T* > data;
    //typedef typename std::vector<T*>::iterator SEQUENCE_ITERATOR;
//};

//struct POSE_SEQUENCE : SEQUENCE< L3::Pose >
//{
    //POSE_SEQUENCE( std::vector<L3::Pose*> poses )
    //{
        //data.assign( poses.begin(), poses.end() );
    //}
//};

//struct LIDAR_SEQUENCE: SEQUENCE< L3::LIDAR >
//{
//};


#endif
