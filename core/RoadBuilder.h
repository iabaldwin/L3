#ifndef L3_ROAD_BUILDER_H
#define L3_ROAD_BUILDER_H

#include "Datatypes.h"
#include "PointCloud.h"
#include "Dataset.h"
#include "Reader.h"
#include "Projector.h"
#include "Configuration.h"
#include "Integrator.h"


namespace L3
{

    struct RoadBuilder
    {
        RoadBuilder( L3::Dataset& dataset, double start_time, double end_time, double experience_section_threshold=10.0, double scan_spacing_threshold=0.2  );    
        
        boost::shared_ptr<L3::IO::BinaryReader< L3::SE3 > >                 pose_reader;
        boost::shared_ptr<L3::IO::SequentialBinaryReader< L3::LMS151 > >    LIDAR_reader;
    };


}

#endif
