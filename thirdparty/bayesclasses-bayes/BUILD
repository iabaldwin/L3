licenses(["permissive"])

cc_library(
    name = "bayes",
    srcs = [
      "BayesFilter/matSupSub.hpp",
      "BayesFilter/filters/average1.hpp",
      "BayesFilter/filters/indirect.hpp",
      "BayesFilter/uBLASmatrix.hpp",
      "BayesFilter/unsFlt.hpp",
      "BayesFilter/UDFlt.hpp",
      "BayesFilter/matSup.hpp",
      "BayesFilter/infRtFlt.hpp",
      "BayesFilter/CIFlt.hpp",
      "BayesFilter/allFilters.hpp",
      "BayesFilter/bayesFlt.hpp",
      "BayesFilter/SIRFlt.hpp",
      "BayesFilter/itrFlt.hpp",
      "BayesFilter/schemeFlt.hpp",
      "BayesFilter/bayesException.hpp",
      "BayesFilter/models.hpp",
      "BayesFilter/covFlt.hpp",
      "BayesFilter/uLAPACK.hpp",
      "BayesFilter/infFlt.hpp",
    ] + [
      "BayesFilter/infRtFlt.cpp",
      "BayesFilter/CIFlt.cpp",
      "BayesFilter/matSup.cpp",
      "BayesFilter/bayesFltAlg.cpp",
      "BayesFilter/infFlt.cpp",
      "BayesFilter/unsFlt.cpp",
      "BayesFilter/UdU.cpp",
      "BayesFilter/covFlt.cpp",
      "BayesFilter/SIRFlt.cpp",
      "BayesFilter/itrFlt.cpp",
      "BayesFilter/UDFlt.cpp",
      "BayesFilter/bayesFlt.cpp",
    ],
    deps = [
      "@boost//:system",
      "@boost//:numeric_ublas",
    ],
    includes = ["."],
    visibility = ["//visibility:public"],
    linkopts = [
        "-llapack",
    ],
)
