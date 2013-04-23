#ifndef L3_H
#define L3_H

#include "Datatypes.h"          // Poses, LIDAR etc
#include "Definitions.h"        // Useful typedefs
#include "Core.h"               // Base types, exceptions
#include "Dataset.h"            // Dataset reader
#include "Configuration.h"      // Libconfig configuration parameters
#include "Runner.h"             // Dataset runner
#include "Iterator.h"           // Constant-time, constant-distance, etc
#include "PointCloud.h"         // Points, point-cloud
#include "Projector.h"          // Convert raw LIDAR data to Point cloud
#include "SwatheBuilder.h"      // Associate LIDAR sans with poses
#include "Reader.h"             // Basic pose/LIDAR reader
#include "Writer.h"             // Basic pose/LIDAR writer
#include "Timing.h"             // Timers, etc.
#include "Utils.h"              // Path management, locales, etc.
#include "ChainBuilder.h"       // Poses from rotational data
#include "Experience.h"         // Experience generation
#include "LibraryAdapters.h"    // PCL, ...
#include "PoseProvider.h"       // Pose sources
#include "Histogram.h"          // Custom histogram type
#include "Estimator.h"          // Core estimator
#include "Smoother.h"           // Histogram smoothing
#include "Integrator.h"         // Chain integration
#include "Predictor.h"          // Pose predcition
#include "Simulator.h"          // Artificial data provider
#include "Misc.h"               // Everything else...

#endif
