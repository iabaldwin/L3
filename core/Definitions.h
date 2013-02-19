#ifndef L3_DEFINITIONS_H
#define L3_DEFINITIONS_H

#include <vector>
#include <deque>
#include "Datatypes.h"

/*
 *  <vector>
 *  For parallelization, we require random access
 */

// Container types
typedef std::vector< std::pair< double, L3::Pose*> >        POSE_SEQUENCE;
typedef std::vector< std::pair< double, L3::LIDAR*> >       LIDAR_SEQUENCE;
typedef std::vector< std::pair< L3::Pose*, L3::LIDAR* > >   SWATHE;


// Iterators
typedef std::vector< std::pair< double, L3::Pose*> >::iterator      POSE_SEQUENCE_ITERATOR;
typedef std::deque< std::pair< double, std::string > >::iterator    WINDOW_ITERATOR;
 
#endif
