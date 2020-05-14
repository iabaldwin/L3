load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

http_archive(
    name = "com_github_eigen_eigen",
    build_file = "eigen.BUILD",
    sha256 = "dd254beb0bafc695d0f62ae1a222ff85b52dbaa3a16f76e781dce22d0d20a4a6",
    strip_prefix = "eigen-eigen-5a0156e40feb",
    urls = ["http://bitbucket.org/eigen/eigen/get/3.3.4.tar.bz2",],
)

new_local_repository(
    name = "pcl",
    path = "/usr",
    build_file_content = """
cc_library(
  name = "pcl",
  hdrs = glob([
    "include/pcl-1.8/pcl/**/*",
  ]),
  srcs = glob([
    "lib/x86_64-linux-gnu/libpcl_*",
  ]),
  includes = [
  "include/pcl-1.8/",
  ],
  visibility = ["//visibility:public"],
  )
"""
)

git_repository(
    name = "com_github_nelhage_rules_boost",
    commit = "5c39b9edd63374fbe4e541c0aca15fddc60752aa",
    remote = "https://github.com/nelhage/rules_boost.git",
)

load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")
boost_deps()

new_local_repository(
    name = "poco",
    path = "/usr",
    build_file_content = """
cc_library(
  name = "poco",
  hdrs = glob([
    "include/Poco/**/*",
  ]),
  srcs = glob([
    "lib/libPoco*",
  ]),
  includes = [
  "include/",
  ],
  visibility = ["//visibility:public"],
  )
"""
)

new_local_repository(
    name = "gsl",
    path = "/usr",
    build_file_content = """
cc_library(
  name = "gsl",
  hdrs = glob([
    "include/gsl/**/*",
  ]),
  srcs = glob([
    "lib/x86_64-linux-gnu/libgsl*",
  ]),
  includes = [
  "include/",
  ],
  visibility = ["//visibility:public"],
  )
"""
)

new_local_repository(
    name = "config",
    path = "/usr",
    build_file_content = """
cc_library(
  name = "config",
  hdrs = [
    "include/libconfig.h++",
  ],
  srcs = [
    "lib/x86_64-linux-gnu/libconfig++.a"
  ],
  includes = [
  "include/",
  ],
  visibility = ["//visibility:public"],
  )
"""
)

new_local_repository(
    name = "z4",
    path = "/usr/lib",
    build_file_content = """
cc_library(
  name = "z4",
  srcs = [
    "x86_64-linux-gnu/liblz4.so"
  ],
  visibility = ["//visibility:public"],
  )
"""
)

new_local_repository(
    name = "lua",
    path = "/usr",
    build_file_content = """
cc_library(
  name = "lua",
  hdrs = glob([
    "include/lua5.1/**/*"
  ]),
  includes = [
    "include/lua5.1",
  ],
  srcs = glob([
    "lib/x86_64-linux-gnu/liblua5.1.so"
  ]),
  visibility = ["//visibility:public"],
  )
"""
)

new_local_repository(
    name = "tbb",
    path = "/usr/lib",
    build_file_content = """
cc_library(
  name = "tbb",
  srcs = glob([
    "x86_64-linux-gnu/libtbb*",
  ]),
  visibility = ["//visibility:public"],
  )
"""
)

http_archive(
    name = "com_github_gflags_gflags",
    sha256 = "6e16c8bc91b1310a44f3965e616383dbda48f83e8c1eaa2370a215057b00cabe",
    strip_prefix = "gflags-77592648e3f3be87d6c7123eb81cbad75f9aef5a",
    urls = [
        "https://mirror.bazel.build/github.com/gflags/gflags/archive/77592648e3f3be87d6c7123eb81cbad75f9aef5a.tar.gz",
        "https://github.com/gflags/gflags/archive/77592648e3f3be87d6c7123eb81cbad75f9aef5a.tar.gz",
    ],
)

http_archive(
    name = "com_github_google_glog",
    sha256 = "7083af285bed3995b5dc2c982f7de39bced9f0e6fd78d631f3285490922a0c3d",
    strip_prefix = "glog-3106945d8d3322e5cbd5658d482c9ffed2d892c0",
    urls = [
        "https://github.com/drigz/glog/archive/3106945d8d3322e5cbd5658d482c9ffed2d892c0.tar.gz",
    ],
)
