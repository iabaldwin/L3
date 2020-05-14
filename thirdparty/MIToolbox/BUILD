licenses(["permissive"])

cc_library(
    name = "MI",
    srcs = [
      "src/WeightedMutualInformation.c",
      "src/RenyiEntropy.c",
      "src/WeightedEntropy.c",
      "src/CalculateProbability.c",
      "src/Entropy.c",
      "src/ArrayOperations.c",
      "src/RenyiMutualInformation.c",
      "src/MutualInformation.c",
    ],
    hdrs = [
      "include/MIToolbox/RenyiEntropy.h",
      "include/MIToolbox/MIToolbox.h",
      "include/MIToolbox/MutualInformation.h",
      "include/MIToolbox/WeightedMutualInformation.h",
      "include/MIToolbox/CalculateProbability.h",
      "include/MIToolbox/Entropy.h",
      "include/MIToolbox/WeightedEntropy.h",
      "include/MIToolbox/RenyiMutualInformation.h",
      "include/MIToolbox/ArrayOperations.h",
    ],
    includes = [
        "include/"
    ],
    visibility = ["//visibility:public"],
    defines = ["COMPILE_C"],
)
