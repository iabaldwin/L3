cc_library(
    name = "L3",
    srcs = [
      "Core.cpp",
      "VelocityProvider.cpp",
      "Experience.cpp",
      "PointCloud.cpp",
      "ScanMatching.cpp",
      "Reader.cpp",
      "Reflectance.cpp",
      "Runner.cpp",
      "Tracking.cpp",
      "Dataset.cpp",
      "Utils.cpp",
      "Misc.cpp",
      "SpatialData.cpp",
      "Windower.cpp",
      "Performance.cpp",
      "Projector.cpp",
      "Datatypes.cpp",
      "Histogram.cpp",
      "SwatheBuilder.cpp",
      "Writer.cpp",
      "Predictor.cpp",
      "Iterator.cpp",
      "Configuration.cpp",
      "Estimator.cpp",
    ] + [
      "Filter.hpp",
      "Integrator.hpp",
      "PoseWindower.hpp",
      "Estimator.hpp",
      "Smoother.hpp",
    ],
    hdrs = [
      "Estimator.h",
      "AlgorithmFactory.h",
      "Smoother.h",
      "CostFactory.h",
      "Performance.h",
      "Core.h",
      "Filter.h",
      "Experience.h",
      "Writer.h",
      "PointCloud.h",
      "PoseProvider.h",
      "Configuration.h",
      "WindowerFactory.h",
      "Predictor.h",
      "Projector.h",
      "Utils.h",
      "Reflectance.h",
      "SwatheBuilder.h",
      "Definitions.h",
      "AbstractFactory.h",
      "Reader.h",
      "Dataset.h",
      "ScanMatching.h",
      "Windower.h",
      "Histogram.h",
      "Tracking.h",
      "L3.h",
      "Timing.h",
      "Runner.h",
      "Datatypes.h",
      "VelocityProvider.h",
      "ChainBuilder.h",
      "Iterator.h",
      "Misc.h",
      "Integrator.h",
      "SpatialData.h",
      "PoseWindower.h",
    ],
    includes = ["."],
    deps = [
        "//thirdparty/MIToolbox:MI",
        "//thirdparty/bayesclasses-bayes:bayes",
        "@com_github_eigen_eigen//:eigen",
        "@pcl",
        "@boost//:thread",
        "@boost//:system",
        "@boost//:filesystem",
        "@com_github_google_glog//:glog",
        "@z4",
        "@gsl",
        "@config",
    ],
    visibility = ["//visibility:public"],
    copts = [
        "-fopenmp",
        "-std=c++14",
    ],
    linkopts = [
        "-lgomp",
    ],
)
