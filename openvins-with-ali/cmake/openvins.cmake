# OpenVINS as this project's visual-inertial kernel, built without ROS.
#
# Source: github_prj/open_vins-master.  Copied to thirdparty/openvins with the ROS nodes, the stand-alone entry
# points and the tests removed -- see the file lists below.  The list is not guessed: OpenVINS supports a no-ROS
# build itself (ENABLE_ROS=OFF / ROS_AVAILABLE=0) and its cmake/ROS1.cmake enumerates exactly which sources the
# libraries contain in that case, which is what this mirrors.
#
# Only one file in the copied tree still mentions ROS, ov_core/src/utils/opencv_yaml_parse.h, and its ROS code is
# inside `#if ROS_AVAILABLE == 1`, so defining ROS_AVAILABLE=0 removes it without editing anything.
#
# Dependency map, established by reading the sources rather than by assuming:
#
#   ov_msckf  -> ov_core, ov_init                       (VioManager.cpp includes init/InertialInitializer.h)
#   ov_init   -> ov_core, Ceres                         (InertialInitializer -> DynamicInitializer -> ceres/)
#   ov_core   -> Eigen, OpenCV, Boost(filesystem, date_time, thread, math)
#
# ov_core and ov_msckf contain *no* reference to Ceres at all; it is ov_init that needs it.  All of these are
# already present in the workspace, so this build needs no downloads:
#
#   Eigen   GraphRTK-INS-main/src/Third-party/Eigen
#   OpenCV  the same install the rest of this project links
#   Ceres   GraphRTK-INS-main/src/Third-party/ceres -- prebuilt 1.14.0, with glog, gflags, SuiteSparse, OpenBLAS.
#           Ceres 1.14 is enough: ov_init guards on `CERES_VERSION_MAJOR == 2 && CERES_VERSION_MINOR >= 2` and falls
#           back to ceres::LocalParameterization, which 1.14 provides (ceres/manifold.h does not exist there).
#   Boost   boost_1_92_0.  Its stage/lib already had date_time/thread/etc. but not filesystem, which OpenVINS does
#           use (VioManager.cpp, VioManagerOptions.h, opencv_yaml_parse.h), so filesystem is built from that same
#           local tree -- see the ALI_BOOST_FILESYSTEM variable below.

set(ALI_OPENVINS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/openvins" CACHE PATH
    "the ROS-free OpenVINS tree copied into this project")

set(ALI_CERES_ROOT "E:/jo_gvil/jo_gvil 1.0/github_prj/GraphRTK-INS-main/src/Third-party/ceres" CACHE PATH
    "Ceres install root (contains include/ceres and lib/ceres.lib); needs glog beside it")
set(ALI_BOOST_ROOT "E:/jo_gvil/jo_gvil 1.0/github_prj/20261011/boost_1_92_0/boost_1_92_0" CACHE PATH
    "Boost root (contains boost/ headers and stage/lib)")
set(ALI_BOOST_TOOLSET "vc143" CACHE STRING "Boost library name toolset tag")
set(ALI_BOOST_VERSION_TAG "1_92" CACHE STRING "Boost library name version tag")

# ---------------------------------------------------------------- sources
# Everything under each module's src/ except what was removed when the tree was copied.  Globbing here is
# deliberate: the three modules are upstream code and their file set is not ours to maintain, so a file added or
# removed upstream should not need a matching edit here.
file(GLOB_RECURSE ALI_OV_CORE_SOURCES "${ALI_OPENVINS_ROOT}/ov_core/src/*.cpp")
file(GLOB_RECURSE ALI_OV_INIT_SOURCES "${ALI_OPENVINS_ROOT}/ov_init/src/*.cpp")
file(GLOB_RECURSE ALI_OV_MSCKF_SOURCES "${ALI_OPENVINS_ROOT}/ov_msckf/src/*.cpp")
list(LENGTH ALI_OV_CORE_SOURCES ALI_OV_CORE_COUNT)
list(LENGTH ALI_OV_INIT_SOURCES ALI_OV_INIT_COUNT)
list(LENGTH ALI_OV_MSCKF_SOURCES ALI_OV_MSCKF_COUNT)
message(STATUS "openvins_ali: OpenVINS sources -- ov_core ${ALI_OV_CORE_COUNT}, ov_init ${ALI_OV_INIT_COUNT}, "
               "ov_msckf ${ALI_OV_MSCKF_COUNT}")

# One library rather than three: the modules are compiled together upstream when ROS is absent (their own no-ROS
# path appends ov_core's sources into each dependent library), and keeping them as one target avoids three
# targets that would each have to be told the same include paths and definitions.
add_library(openvins_ali_core STATIC
  ${ALI_OV_CORE_SOURCES}
  ${ALI_OV_INIT_SOURCES}
  ${ALI_OV_MSCKF_SOURCES}
)

target_include_directories(openvins_ali_core PUBLIC
  "${ALI_OPENVINS_ROOT}/ov_core/src"
  "${ALI_OPENVINS_ROOT}/ov_init/src"
  "${ALI_OPENVINS_ROOT}/ov_msckf/src"
  "${ALI_EIGEN_INCLUDE}"
  "${ALI_BOOST_ROOT}"
  "${ALI_CERES_ROOT}/include"
)
target_include_directories(openvins_ali_core SYSTEM PUBLIC "${ALI_OPENCV_INCLUDE}")

# ROS_AVAILABLE=0 is what OpenVINS itself uses for the no-ROS build; ENABLE_ARUCO_TAGS=0 keeps TrackAruco from
# including <opencv2/aruco.hpp>, which is in opencv_contrib and not in the install this project links.
target_compile_definitions(openvins_ali_core PUBLIC
  ROS_AVAILABLE=0
  ENABLE_ARUCO_TAGS=0
)

# Boost.Filesystem has to exist before this configure step.  They are separate variables rather than a glob so a
# missing one fails the configure with a sentence instead of a wall of unresolved externals at link time.
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
    message(FATAL_ERROR
      "openvins_ali: ${name} not found at ${path}\n"
      "  Boost.Filesystem is not built by the workspace's Boost build; build it from the same source tree with:\n"
      "    b2 --with-filesystem variant=release link=static runtime-link=shared threading=multi address-model=64 stage\n"
      "  (with TMP/TEMP pointed at a writable directory -- b2 fails with \"failed to write command file!\" otherwise)")
  endif()
endforeach()

# glog is a hard dependency of a Ceres shared build: ceres.lib's link interface names glog.lib.
target_link_libraries(openvins_ali_core PUBLIC
  ${ALI_OPENCV_LIBS}
  "${ALI_CERES_ROOT}/lib/ceres.lib"
  "${ALI_CERES_ROOT}/lib/glog.lib"
  "${ALI_BOOST_FILESYSTEM}"
  "${ALI_BOOST_DATE_TIME}"
  "${ALI_BOOST_THREAD}"
)

if(MSVC)
  # Deliberately no /FI force-include here.  The kernel came from a tree that force-included a compatibility header
  # supplying M_PI and similar, and doing the same broke the build in a way worth recording:
  #
  # that header includes <windows.h>, and wingdi.h defines ERROR as a macro.  ov_core/src/utils/print.h declares
  # `enum PrintLevel { ALL = 0, DEBUG = 1, INFO = 2, WARNING = 3, ERROR = 4, SILENT = 5 };`, so with windows.h in
  # front of it the enumerator becomes `0 = 4` and the file does not parse.  OpenVINS never includes windows.h
  # itself, so the force-include was the only thing putting it there.
  #
  # What was actually needed is M_PI (Propagator.cpp, DynamicInitializer.cpp, quat_ops.h), and MSVC's <cmath>
  # defines that under _USE_MATH_DEFINES.  NOMINMAX and WIN32_LEAN_AND_MEAN are set too, because Ceres or glog may
  # pull windows.h in transitively and std::min/std::max are used throughout.
  target_compile_options(openvins_ali_core PRIVATE /EHsc /utf-8 /bigobj /MP)
  target_compile_definitions(openvins_ali_core PRIVATE
    _CRT_SECURE_NO_WARNINGS
    _USE_MATH_DEFINES
    NOMINMAX
    WIN32_LEAN_AND_MEAN
  )
endif()
