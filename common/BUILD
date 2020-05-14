cc_library(
    name = "estimation",
    srcs = [
      "estimation/Pose.cpp",
    ],
    hdrs = [
      "estimation/Pose.h",
    ],
    deps = [
        "@com_github_eigen_eigen//:eigen",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "imagery",
    srcs = [
      "imagery/Imagery.cpp",
    ],
    hdrs = [
      "imagery/Imagery.h",
      "imagery/stb_image.h",
    ],
    includes = ["."],
    visibility = ["//visibility:public"],
    deps = [
        "@boost//:smart_ptr",
        "@boost//:shared_ptr",
        "@boost//:system",
        "@boost//:filesystem",
    ],
)
