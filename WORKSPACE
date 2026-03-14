load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

http_archive(
    name = "com_github_eigen_eigen",
    build_file = "eigen.BUILD",
    sha256 = "b170583f59d6778be4bfeae88583c77ed610df5b803ce5cb4aa850d0e8017c2f",
    strip_prefix = "eigen-3.3.4",
    urls = ["https://gitlab.com/libeigen/eigen/-/archive/3.3.4/eigen-3.3.4.tar.bz2"],
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
    "lib/x86_64-linux-gnu/libpcl_*",
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
  srcs = ["lib/x86_64-linux-gnu/libboost_thread.so"],
  deps = [":headers"],
  visibility = ["//visibility:public"],
)
cc_library(
  name = "system",
  srcs = ["lib/x86_64-linux-gnu/libboost_system.so"],
  deps = [":headers"],
  visibility = ["//visibility:public"],
)
cc_library(
  name = "filesystem",
  srcs = ["lib/x86_64-linux-gnu/libboost_filesystem.so"],
  deps = [":headers"],
  visibility = ["//visibility:public"],
)
cc_library(
  name = "regex",
  srcs = ["lib/x86_64-linux-gnu/libboost_regex.so"],
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
  srcs = ["lib/x86_64-linux-gnu/libboost_chrono.so"],
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
    "lib/x86_64-linux-gnu/libPoco*",
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
