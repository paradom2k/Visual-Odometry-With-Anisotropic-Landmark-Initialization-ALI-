# KLT tracking and MSCKF + SLAM estimation core adopted from OpenVINS, built without ROS.
#
# Source: the ROS-free ov_core + ov_init + ov_msckf tree used by ALI-VINS.
# Dependency map:
#   ov_msckf -> ov_core, ov_init
#   ov_init  -> ov_core, Ceres (dynamic initializer only)
#   ov_core  -> Eigen, OpenCV, Boost(filesystem, date_time, math)
#
# The experiments use the SLAM branch, which is controlled by max_slam > 0 (there is no use_slam
# key in this OpenVINS version).  ALI is the OV_WEIGHT=l2cov landmark-initialization covariance.

set(ALI_OPENVINS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/openvins" CACHE PATH
    "the ROS-free OpenVINS tree")

set(ALI_CERES_ROOT "E:/jo_gvil/jo_gvil 1.0/github_prj/GraphRTK-INS-main/src/Third-party/ceres" CACHE PATH
    "Ceres install root (contains include/ceres and lib/ceres.lib); needs glog beside it")
set(ALI_BOOST_ROOT "E:/jo_gvil/jo_gvil 1.0/github_prj/20261011/boost_1_92_0/boost_1_92_0" CACHE PATH
    "Boost root (contains boost/ headers and stage/lib)")
set(ALI_BOOST_TOOLSET "vc143" CACHE STRING "Boost library name toolset tag")
set(ALI_BOOST_VERSION_TAG "1_92" CACHE STRING "Boost library name version tag")

file(GLOB_RECURSE ALI_OV_CORE_SOURCES "${ALI_OPENVINS_ROOT}/ov_core/src/*.cpp")
file(GLOB_RECURSE ALI_OV_INIT_SOURCES "${ALI_OPENVINS_ROOT}/ov_init/src/*.cpp")
file(GLOB_RECURSE ALI_OV_MSCKF_SOURCES "${ALI_OPENVINS_ROOT}/ov_msckf/src/*.cpp")

# CpiV2 is an alternative inertial preintegration model that this branch never selects: the dynamic
# initializer uses CpiV1 only.  Leaving it in costs a very large /O2 translation unit for no symbol.
list(FILTER ALI_OV_CORE_SOURCES EXCLUDE REGEX "CpiV2\\.cpp$")
list(LENGTH ALI_OV_CORE_SOURCES ALI_OV_CORE_COUNT)
list(LENGTH ALI_OV_INIT_SOURCES ALI_OV_INIT_COUNT)
list(LENGTH ALI_OV_MSCKF_SOURCES ALI_OV_MSCKF_COUNT)
message(STATUS "ALI-VINS: OpenVINS sources -- ov_core ${ALI_OV_CORE_COUNT}, ov_init ${ALI_OV_INIT_COUNT}, "
               "ov_msckf ${ALI_OV_MSCKF_COUNT}")

add_library(ali_vins_openvins STATIC
  ${ALI_OV_CORE_SOURCES}
  ${ALI_OV_INIT_SOURCES}
  ${ALI_OV_MSCKF_SOURCES}
)

target_include_directories(ali_vins_openvins PUBLIC
  "${ALI_OPENVINS_ROOT}/ov_core/src"
  "${ALI_OPENVINS_ROOT}/ov_init/src"
  "${ALI_OPENVINS_ROOT}/ov_msckf/src"
  "${ALI_EIGEN_INCLUDE}"
  "${ALI_BOOST_ROOT}"
  "${ALI_CERES_ROOT}/include"
)
target_include_directories(ali_vins_openvins SYSTEM PUBLIC "${ALI_OPENCV_INCLUDE}")

target_compile_definitions(ali_vins_openvins PUBLIC
  ROS_AVAILABLE=0
  ENABLE_ARUCO_TAGS=0
)

set(ALI_BOOST_TAG "${ALI_BOOST_TOOLSET}-mt-x64-${ALI_BOOST_VERSION_TAG}")
set(ALI_BOOST_FILESYSTEM "${ALI_BOOST_ROOT}/stage/lib/libboost_filesystem-${ALI_BOOST_TAG}.lib")
set(ALI_BOOST_DATE_TIME "${ALI_BOOST_ROOT}/stage/lib/libboost_date_time-${ALI_BOOST_TAG}.lib")
set(ALI_BOOST_THREAD "${ALI_BOOST_ROOT}/stage/lib/libboost_thread-${ALI_BOOST_TAG}.lib")
foreach(pair
    "ALI_BOOST_FILESYSTEM;${ALI_BOOST_FILESYSTEM}"
    "ALI_BOOST_DATE_TIME;${ALI_BOOST_DATE_TIME}"
    "ALI_BOOST_THREAD;${ALI_BOOST_THREAD}"
    "ALI_CERES_LIBRARY;${ALI_CERES_ROOT}/lib/ceres.lib")
  list(GET pair 0 name)
  list(GET pair 1 path)
  if(NOT EXISTS "${path}")
    message(FATAL_ERROR "ALI-VINS: ${name} not found at ${path}")
  endif()
endforeach()

target_link_libraries(ali_vins_openvins PUBLIC
  ${ALI_OPENCV_LIBS}
  "${ALI_CERES_ROOT}/lib/ceres.lib"
  "${ALI_CERES_ROOT}/lib/glog.lib"
  "${ALI_BOOST_FILESYSTEM}"
  "${ALI_BOOST_DATE_TIME}"
  "${ALI_BOOST_THREAD}"
)

if(MSVC)
  target_compile_options(ali_vins_openvins PRIVATE /EHsc /utf-8 /bigobj /MP)
  target_compile_definitions(ali_vins_openvins PRIVATE
    _CRT_SECURE_NO_WARNINGS
    _USE_MATH_DEFINES
    NOMINMAX
    WIN32_LEAN_AND_MEAN
  )
endif()
