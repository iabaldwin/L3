cc_binary(
    name = "app",
    srcs = [
        "app.cpp",
    ],

    deps = [
        "//core:L3",
        "//thirdparty/GLV",
        "//renderer",
        "@poco",
        "@boost//:thread",
        "@boost//:filesystem",
        "@boost//:regex",
        "@lua",
        "@config",
        "@gsl",
        "@tbb",
        "@com_github_google_glog//:glog",
        ],
    linkopts = [
        "-lglut"
    ],
)

cc_binary(
    name = "headless",
    srcs = [
        "headless.cpp",
    ],

    deps = [
        "//core:L3",
        "@poco",
        "@boost//:thread",
        "@boost//:filesystem",
        "@boost//:regex",
        "@config",
        "@gsl",
        "@tbb",
        "@com_github_google_glog//:glog",
        ],
)
