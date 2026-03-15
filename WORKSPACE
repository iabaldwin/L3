load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")
load("//tools:local_deps.bzl", "boost_repository", "lua_repository", "pcl_repository", "simple_lib_repository", "tbb_repository", "z4_repository")

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

pcl_repository(name = "pcl")

boost_repository(name = "boost")

simple_lib_repository(
    name = "poco",
    lib_name = "poco",
    hdrs_glob = ["root/include/Poco/**/*"],
    srcs_glob = ["root/{lib_dir}/libPoco*.{ext}*"],
    extra_includes = ["root/include/"],
)

simple_lib_repository(
    name = "gsl",
    lib_name = "gsl",
    hdrs_glob = ["root/include/gsl/**/*"],
    srcs_glob = ["root/{lib_dir}/libgsl*.{ext}*"],
    extra_includes = ["root/include/"],
)

simple_lib_repository(
    name = "config",
    lib_name = "config",
    hdrs_glob = ["root/include/libconfig.h++"],
    srcs_glob = ["root/{lib_dir}/libconfig++.a", "root/{lib_dir}/libconfig++.{ext}*"],
    extra_includes = ["root/include/"],
)

z4_repository(name = "z4")

lua_repository(name = "lua")

tbb_repository(name = "tbb")

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
