#ifndef L3_DEFINITIONS_H
#define L3_DEFINITIONS_H

#include "Datatypes.h"

//typedef std::list< std::pair< L3::Pose*, L3::LMS151* > >    SWATHE;
typedef std::vector< std::pair< L3::Pose*, L3::LMS151* > >    SWATHE;
typedef std::vector<L3::Pose*>                              POSE_SEQUENCE;
typedef std::vector<L3::LMS151*>                            LIDAR_SEQUENCE;

#endif
