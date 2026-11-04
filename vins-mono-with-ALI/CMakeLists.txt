# VINS-Mono + ALI -- ROS-free MSVC port.
#
# The estimator core is VINS-Mono (GPL-3.0).  The port replaces the ROS node with src/vins_run.cpp and
# adds ALI whitening to ProjectionFactor.  Build with build_msvc.bat, which uses the same MSVC/CMake as
# ALI-VINS.

cmake_minimum_required(VERSION 3.20)
project(VINS-Mono-ALI VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")
if(MSVC)
  foreach(config Debug Release RelWithDebInfo MinSizeRel)
    string(TOUPPER "${config}" upper)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${upper} "${CMAKE_CURRENT_SOURCE_DIR}/bin")
  endforeach()
endif()

set(OpenCV_ROOT "" CACHE PATH "OpenCV install root")
set(Eigen3_ROOT "" CACHE PATH "Eigen root containing Eigen/Core")
set(VINS_CERES_ROOT "E:/jo_gvil/jo_gvil 1.0/github_prj/GraphRTK-INS-main/src/Third-party/ceres" CACHE PATH "Ceres install")
set(VINS_BOOST_ROOT "E:/jo_gvil/jo_gvil 1.0/github_prj/20261011/boost_1_92_0/boost_1_92_0" CACHE PATH "Boost root")

if(Eigen3_ROOT AND EXISTS "${Eigen3_ROOT}/Eigen/Core")
  set(VINS_EIGEN_INCLUDE "${Eigen3_ROOT}")
else()
  find_package(Eigen3 REQUIRED)
  set(VINS_EIGEN_INCLUDE "${EIGEN3_INCLUDE_DIR}")
endif()

if(OpenCV_ROOT)
  find_path(VINS_OPENCV_INCLUDE "opencv2/core.hpp" HINTS "${OpenCV_ROOT}/include" NO_DEFAULT_PATH)
  file(GLOB VINS_OPENCV_LIBS "${OpenCV_ROOT}/lib/opencv_*.lib" "${OpenCV_ROOT}/x64/vc*/lib/opencv_*.lib")
else()
  find_package(OpenCV REQUIRED COMPONENTS core imgproc video calib3d imgcodecs highgui features2d)
  set(VINS_OPENCV_INCLUDE "${OpenCV_INCLUDE_DIRS}")
  set(VINS_OPENCV_LIBS "${OpenCV_LIBS}")
endif()

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/vins.cmake")

add_executable(vins_run
  src/vins_run.cpp
  src/ali/ali.cpp
  ${VINS_CORE_SOURCES}
)
target_link_libraries(vins_run PRIVATE vins_mono_core)

add_executable(vins_app src/app_main.cpp)
if(WIN32)
  target_link_libraries(vins_app PRIVATE user32 gdi32 comctl32 shell32 comdlg32)
endif()

if(MSVC)
  foreach(tgt vins_run vins_app vins_mono_core)
    target_compile_options(${tgt} PRIVATE /EHsc /utf-8 /bigobj)
    target_compile_definitions(${tgt} PRIVATE
      _CRT_SECURE_NO_WARNINGS _USE_MATH_DEFINES NOMINMAX WIN32_LEAN_AND_MEAN GLOG_NO_ABBREVIATED_SEVERITIES)
  endforeach()
endif()

set(VINS_STAGED_DLLS "")
if(WIN32 AND OpenCV_ROOT)
  file(GLOB _opencv_dlls "${OpenCV_ROOT}/bin/opencv_*.dll")
  list(APPEND VINS_STAGED_DLLS ${_opencv_dlls})
endif()
if(WIN32 AND VINS_CERES_ROOT)
  file(GLOB _ceres_dlls "${VINS_CERES_ROOT}/bin/*.dll")
  list(APPEND VINS_STAGED_DLLS ${_ceres_dlls})
endif()
if(VINS_STAGED_DLLS)
  list(REMOVE_DUPLICATES VINS_STAGED_DLLS)
  add_custom_target(vins_stage_dlls ALL
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${VINS_STAGED_DLLS} "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
    COMMENT "staging runtime DLLs into bin/")
endif()

message(STATUS "vins-mono-with-ALI: vins_run + vins_app -> ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
