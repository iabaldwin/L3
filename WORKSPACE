load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

http_archive(
    name = "com_github_eigen_eigen",
    build_file = "eigen.BUILD",
    sha256 = "8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72",
    strip_prefix = "eigen-3.4.0",
    urls = [
        "https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.bz2",
        "https://storage.googleapis.com/mirror.tensorflow.org/gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.bz2",
    ],
)

new_local_repository(
    name = "pcl",
    path = "/usr",
    build_file_content = """
cc_library(
  name = "pcl",
  hdrs = glob([
    "include/pcl-1.14/pcl/**/*",
  ]),
  srcs = glob([
    "lib/*/libpcl_*",
  ]),
  includes = [
  "include/pcl-1.14/",
  ],
  visibility = ["//visibility:public"],
  )
"""
)

new_local_repository(
    name = "boost",
    path = "/usr",
    build_file_content = """
cc_library(
  name = "headers",
  hdrs = glob(["include/boost/**/*"]),
  includes = ["include/"],
  visibility = ["//visibility:public"],
)
cc_library(
  name = "thread",
  srcs = glob(["lib/*/libboost_thread.so"]),
  deps = [":headers"],
  visibility = ["//visibility:public"],
)
cc_library(
  name = "system",
  srcs = glob(["lib/*/libboost_system.so"]),
  deps = [":headers"],
  visibility = ["//visibility:public"],
)
cc_library(
  name = "filesystem",
  srcs = glob(["lib/*/libboost_filesystem.so"]),
  deps = [":headers"],
  visibility = ["//visibility:public"],
)
cc_library(
  name = "regex",
  srcs = glob(["lib/*/libboost_regex.so"]),
  deps = [":headers"],
  visibility = ["//visibility:public"],
)
cc_library(
  name = "smart_ptr",
  deps = [":headers"],
  visibility = ["//visibility:public"],
)
cc_library(
  name = "shared_ptr",
  deps = [":headers"],
  visibility = ["//visibility:public"],
)
cc_library(
  name = "numeric_ublas",
  deps = [":headers"],
  visibility = ["//visibility:public"],
)
cc_library(
  name = "chrono",
  srcs = glob(["lib/*/libboost_chrono.so"]),
  deps = [":headers"],
  visibility = ["//visibility:public"],
)
"""
)

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
    "lib/*/libPoco*",
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
    "lib/*/libgsl*",
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
  srcs = glob([
    "lib/*/libconfig++.a",
  ]),
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
  srcs = glob([
    "*/liblz4.so",
  ]),
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
    "lib/*/liblua5.1.so"
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
    "*/libtbb*",
  ]),
  visibility = ["//visibility:public"],
  )
"""
)

http_archive(
    name = "com_github_gflags_gflags",
    sha256 = "34af2f15cf7367513b352bdcd2493ab14ce43692d2dcd9dfc499492966c64dcf",
    strip_prefix = "gflags-2.2.2",
    urls = [
        "https://github.com/gflags/gflags/archive/v2.2.2.tar.gz",
        "https://mirror.bazel.build/github.com/gflags/gflags/archive/v2.2.2.tar.gz",
    ],
)

http_archive(
    name = "com_github_google_glog",
    sha256 = "122fb6b712f82b7a6a33b0e3a78eb4e309c3e1b05f5c3587eb091a39e7e86900",
    strip_prefix = "glog-0.7.1",
    urls = [
        "https://github.com/google/glog/archive/v0.7.1.tar.gz",
    ],
)
