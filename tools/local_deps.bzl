"""Repository rules for auto-detecting system library paths across Linux and macOS."""

def _detect_prefix(repository_ctx):
    """Detect the system library prefix based on OS."""
    os_name = repository_ctx.os.name.lower()
    if "mac" in os_name or "darwin" in os_name:
        # Try Apple Silicon Homebrew first, then Intel Homebrew
        result = repository_ctx.execute(["brew", "--prefix"])
        if result.return_code == 0:
            return result.stdout.strip()
        return "/usr/local"
    return "/usr"

def _detect_lib_dir(repository_ctx, prefix):
    """Detect the library subdirectory."""
    os_name = repository_ctx.os.name.lower()
    if "mac" in os_name or "darwin" in os_name:
        return "lib"
    # Linux: find the multiarch triplet
    result = repository_ctx.execute(["dpkg-architecture", "-qDEB_HOST_MULTIARCH"])
    if result.return_code == 0:
        return "lib/" + result.stdout.strip()
    # Fallback: try common paths
    return "lib"

def _is_macos(repository_ctx):
    os_name = repository_ctx.os.name.lower()
    return "mac" in os_name or "darwin" in os_name

def _lib_ext(repository_ctx):
    if _is_macos(repository_ctx):
        return "dylib"
    return "so"

def _pcl_impl(repository_ctx):
    prefix = _detect_prefix(repository_ctx)
    lib_dir = _detect_lib_dir(repository_ctx, prefix)
    ext = _lib_ext(repository_ctx)
    macos = _is_macos(repository_ctx)

    # Detect PCL version directory
    pcl_include = "include/pcl-1.14/"
    if macos:
        result = repository_ctx.execute(["bash", "-c", "ls -d {}/include/pcl-*/".format(prefix)])
        if result.return_code == 0:
            # Extract "include/pcl-X.Y/" from the first match
            path = result.stdout.strip().split("\n")[0]
            pcl_include = path.replace(prefix + "/", "")

    repository_ctx.symlink(prefix, "root")
    repository_ctx.file("BUILD", """
cc_library(
  name = "pcl",
  hdrs = glob(["root/{pcl_include}pcl/**/*"]),
  srcs = glob(["root/{lib_dir}/libpcl_*.{ext}"]),
  includes = ["root/{pcl_include}"],
  visibility = ["//visibility:public"],
)
""".format(pcl_include = pcl_include, lib_dir = lib_dir, ext = ext))

pcl_repository = repository_rule(
    implementation = _pcl_impl,
)

def _boost_impl(repository_ctx):
    prefix = _detect_prefix(repository_ctx)
    lib_dir = _detect_lib_dir(repository_ctx, prefix)
    ext = _lib_ext(repository_ctx)
    macos = _is_macos(repository_ctx)

    repository_ctx.symlink(prefix, "root")

    # On macOS Homebrew, boost libs may have version suffixes
    lib_pattern = "root/{lib_dir}/libboost_{{name}}*.{ext}*".format(lib_dir = lib_dir, ext = ext)
    if macos:
        lib_pattern = "root/{lib_dir}/libboost_{{name}}*.{ext}".format(lib_dir = lib_dir, ext = ext)

    repository_ctx.file("BUILD", """
cc_library(
  name = "headers",
  hdrs = glob(["root/include/boost/**/*"]),
  includes = ["root/include/"],
  visibility = ["//visibility:public"],
)
cc_library(
  name = "thread",
  srcs = glob(["{thread}"]),
  deps = [":headers"],
  visibility = ["//visibility:public"],
)
cc_library(
  name = "system",
  srcs = glob(["{system}"]),
  deps = [":headers"],
  visibility = ["//visibility:public"],
)
cc_library(
  name = "filesystem",
  srcs = glob(["{filesystem}"]),
  deps = [":headers"],
  visibility = ["//visibility:public"],
)
cc_library(
  name = "regex",
  srcs = glob(["{regex}"]),
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
  srcs = glob(["{chrono}"]),
  deps = [":headers"],
  visibility = ["//visibility:public"],
)
""".format(
        thread = lib_pattern.format(name = "thread"),
        system = lib_pattern.format(name = "system"),
        filesystem = lib_pattern.format(name = "filesystem"),
        regex = lib_pattern.format(name = "regex"),
        chrono = lib_pattern.format(name = "chrono"),
    ))

boost_repository = repository_rule(
    implementation = _boost_impl,
)

def _simple_lib_impl(repository_ctx):
    """Generic repository rule for simple system libraries."""
    prefix = _detect_prefix(repository_ctx)
    lib_dir = _detect_lib_dir(repository_ctx, prefix)
    ext = _lib_ext(repository_ctx)
    name = repository_ctx.attr.lib_name
    hdrs_glob = repository_ctx.attr.hdrs_glob
    srcs_glob = repository_ctx.attr.srcs_glob
    includes = repository_ctx.attr.extra_includes

    repository_ctx.symlink(prefix, "root")

    # Resolve placeholders in patterns
    resolved_hdrs = [h.replace("{lib_dir}", lib_dir).replace("{ext}", ext) for h in hdrs_glob]
    resolved_srcs = [s.replace("{lib_dir}", lib_dir).replace("{ext}", ext) for s in srcs_glob]
    resolved_includes = [i.replace("{lib_dir}", lib_dir) for i in includes]

    hdrs_str = ", ".join(['"{}"'.format(h) for h in resolved_hdrs])
    srcs_str = ", ".join(['"{}"'.format(s) for s in resolved_srcs])
    includes_str = ", ".join(['"{}"'.format(i) for i in resolved_includes])

    repository_ctx.file("BUILD", """
cc_library(
  name = "{name}",
  hdrs = glob([{hdrs}]),
  srcs = glob([{srcs}]),
  includes = [{includes}],
  visibility = ["//visibility:public"],
)
""".format(name = name, hdrs = hdrs_str, srcs = srcs_str, includes = includes_str))

simple_lib_repository = repository_rule(
    implementation = _simple_lib_impl,
    attrs = {
        "lib_name": attr.string(mandatory = True),
        "hdrs_glob": attr.string_list(default = []),
        "srcs_glob": attr.string_list(default = []),
        "extra_includes": attr.string_list(default = []),
    },
)

def _lua_impl(repository_ctx):
    prefix = _detect_prefix(repository_ctx)
    lib_dir = _detect_lib_dir(repository_ctx, prefix)
    ext = _lib_ext(repository_ctx)
    macos = _is_macos(repository_ctx)

    repository_ctx.symlink(prefix, "root")

    if macos:
        # Homebrew lua@5.1 installs to its own prefix
        result = repository_ctx.execute(["brew", "--prefix", "lua@5.1"])
        if result.return_code == 0:
            lua_prefix = result.stdout.strip()
            repository_ctx.symlink(lua_prefix, "lua_root")
            repository_ctx.file("BUILD", """
cc_library(
  name = "lua",
  hdrs = glob(["lua_root/include/**/*"]),
  srcs = glob(["lua_root/lib/*.dylib"]),
  includes = ["lua_root/include"],
  visibility = ["//visibility:public"],
)
""")
            return

    # Linux fallback
    repository_ctx.file("BUILD", """
cc_library(
  name = "lua",
  hdrs = glob(["root/include/lua5.1/**/*"]),
  srcs = glob(["root/{lib_dir}/liblua5.1.{ext}"]),
  includes = ["root/include/lua5.1"],
  visibility = ["//visibility:public"],
)
""".format(lib_dir = lib_dir, ext = ext))

lua_repository = repository_rule(
    implementation = _lua_impl,
)

def _tbb_impl(repository_ctx):
    prefix = _detect_prefix(repository_ctx)
    lib_dir = _detect_lib_dir(repository_ctx, prefix)
    ext = _lib_ext(repository_ctx)

    repository_ctx.symlink(prefix, "root")
    repository_ctx.file("BUILD", """
cc_library(
  name = "tbb",
  srcs = glob(["root/{lib_dir}/libtbb*.{ext}*"]),
  visibility = ["//visibility:public"],
)
""".format(lib_dir = lib_dir, ext = ext))

tbb_repository = repository_rule(
    implementation = _tbb_impl,
)

def _z4_impl(repository_ctx):
    prefix = _detect_prefix(repository_ctx)
    lib_dir = _detect_lib_dir(repository_ctx, prefix)
    ext = _lib_ext(repository_ctx)

    repository_ctx.symlink(prefix, "root")
    repository_ctx.file("BUILD", """
cc_library(
  name = "z4",
  srcs = glob(["root/{lib_dir}/liblz4*.{ext}*"]),
  visibility = ["//visibility:public"],
)
""".format(lib_dir = lib_dir, ext = ext))

z4_repository = repository_rule(
    implementation = _z4_impl,
)
