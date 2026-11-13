# websocketpp (BSD-3-Clause), header-only: viz's vendored copy (viz third_party/websocketpp/UPSTREAM.md).
load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "websocketpp",
    hdrs = glob(["websocketpp/**/*.hpp"]),
    defines = ["ASIO_STANDALONE"],
    includes = ["."],
    linkopts = ["-lpthread"],
    visibility = ["//visibility:public"],
)
