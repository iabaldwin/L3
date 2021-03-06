#pragma once

#include <vector>
#include <deque>
#include <boost/shared_ptr.hpp>

#include "Datatypes.h"

// Container types
typedef std::vector< std::pair< double, boost::shared_ptr< L3::Pose > > >                                       POSE_SEQUENCE;
typedef std::vector< std::pair< double, boost::shared_ptr< L3::LIDAR > > >                                      LIDAR_SEQUENCE;
typedef std::vector< std::pair< boost::shared_ptr<L3::Pose>, boost::shared_ptr<L3::LIDAR> > >                   SWATHE;

// Iterators
typedef std::vector< std::pair< double, boost::shared_ptr< L3::Pose > > >::iterator                             POSE_SEQUENCE_ITERATOR;
typedef std::vector< std::pair< double, boost::shared_ptr< L3::LIDAR > > >::iterator                            LIDAR_SEQUENCE_ITERATOR;

typedef std::vector< std::pair< boost::shared_ptr<L3::Pose>, boost::shared_ptr<L3::LIDAR> > >::iterator         SWATHE_ITERATOR;
typedef std::vector< std::pair< boost::shared_ptr<L3::Pose>, boost::shared_ptr<L3::LIDAR> > >::const_iterator   SWATHE_ITERATOR_CONST;
