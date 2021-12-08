load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
  name = "async_buffer",
  srcs = [
    "async_buffer.cc",
  ],
  hdrs = [
    "async_buffer.h",
  ],
  deps = [
    "@pcl",
    "@eigen",    
  ]
)

cc_binary(
    name = "ndt_mapping",
    srcs = [
        "ndt_mapping.cc",
    ],
    copts = ["-DMODULE_NAME=\\\"ndt_mapping\\\""],    
    deps = [
      ":async_buffer",
      "//cyber/common:log",
      "//modules/localization/msf/common/io:localization_msf_common_io",
      "//external:gflags",
      "@pcl",
      "@eigen",
    ],
)
