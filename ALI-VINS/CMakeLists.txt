# ALI-VINS -- an independent implementation that adopts the OpenVINS KLT tracking and
# MSCKF + SLAM design, with GFTT detection and ALI landmark initialization.
#
# Layout:
#   bin/      ali_vins_app.exe, ali_vins_run.exe, runtime DLLs
#   config/   *.conf run configurations and their OpenVINS YAMLs
#   sol/      trajectories and run logs
#
# Build (MSVC + Ninja):
#   cmake -G Ninja -S . -B build -DCMAKE_BUILD_TYPE=Release ^
#         -DOpenCV_ROOT=<opencv install> ^
#         -DEigen3_ROOT=<dir containing Eigen/Core> ^
#         -DALI_CERES_ROOT=<ceres install> ^
#         -DALI_BOOST_ROOT=<boost root>
#   cmake --build build -j 4
cmake_minimum_required(VERSION 3.20)
project(ALI-VINS VERSION 0.1.0 LANGUAGES CXX)

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

set(OpenCV_ROOT "" CACHE PATH "OpenCV install root (contains include/opencv2 and the import libraries)")
set(Eigen3_ROOT "" CACHE PATH "Eigen root (contains Eigen/Core)")

if(Eigen3_ROOT AND EXISTS "${Eigen3_ROOT}/Eigen/Core")
  set(ALI_EIGEN_INCLUDE "${Eigen3_ROOT}")
  message(STATUS "ALI-VINS: Eigen from Eigen3_ROOT = ${Eigen3_ROOT}")
else()
  find_package(Eigen3 QUIET)
  if(Eigen3_FOUND)
    set(ALI_EIGEN_INCLUDE "${EIGEN3_INCLUDE_DIR}")
    message(STATUS "ALI-VINS: Eigen from find_package = ${EIGEN3_INCLUDE_DIR}")
  else()
    message(FATAL_ERROR "Eigen not found: set -DEigen3_ROOT=<dir containing Eigen/Core>")
  endif()
endif()

if(OpenCV_ROOT)
  find_path(ALI_OPENCV_INCLUDE "opencv2/core.hpp" HINTS "${OpenCV_ROOT}/include" NO_DEFAULT_PATH)
  if(NOT ALI_OPENCV_INCLUDE)
    message(FATAL_ERROR "no OpenCV headers under ${OpenCV_ROOT}/include")
  endif()
  file(GLOB ALI_OPENCV_LIBS
       "${OpenCV_ROOT}/lib/opencv_*.lib" "${OpenCV_ROOT}/lib/libopencv_*.so"
       "${OpenCV_ROOT}/lib/libopencv_*.a" "${OpenCV_ROOT}/x64/vc*/lib/opencv_*.lib")
  if(NOT ALI_OPENCV_LIBS)
    message(FATAL_ERROR "no OpenCV import libraries under ${OpenCV_ROOT}")
  endif()
  list(LENGTH ALI_OPENCV_LIBS ALI_OPENCV_LIB_COUNT)
  message(STATUS "ALI-VINS: OpenCV headers ${ALI_OPENCV_INCLUDE}, ${ALI_OPENCV_LIB_COUNT} library file(s)")
else()
  find_package(OpenCV REQUIRED COMPONENTS core imgproc video calib3d imgcodecs features2d)
  set(ALI_OPENCV_INCLUDE "${OpenCV_INCLUDE_DIRS}")
  set(ALI_OPENCV_LIBS "${OpenCV_LIBS}")
  message(STATUS "ALI-VINS: OpenCV ${OpenCV_VERSION} from find_package")
endif()

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/openvins.cmake")

# ------------------------------------------------------------------ the estimator driver
add_executable(ali_vins_run src/ali_vins_run.cpp)
target_link_libraries(ali_vins_run PRIVATE ali_vins_openvins)
if(MSVC)
  target_compile_options(ali_vins_run PRIVATE /EHsc /utf-8 /bigobj)
  target_compile_definitions(ali_vins_run PRIVATE
    _CRT_SECURE_NO_WARNINGS
    _USE_MATH_DEFINES
    NOMINMAX
    WIN32_LEAN_AND_MEAN
  )
endif()

# ------------------------------------------------------------------ the Win32 application
# It links only Win32 libraries: the estimator runs as a child process, so a crash is reported
# in the window instead of taking the window down with it.
add_executable(ali_vins_app src/app_main.cpp)
if(MSVC)
  target_compile_options(ali_vins_app PRIVATE /EHsc /utf-8)
  target_compile_definitions(ali_vins_app PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()
if(WIN32)
  target_link_libraries(ali_vins_app PRIVATE user32 gdi32 comctl32 shell32 comdlg32)
endif()

# ------------------------------------------------------------------ runtime DLLs beside the executables
set(ALI_STAGED_DLLS "")
if(WIN32 AND OpenCV_ROOT)
  file(GLOB ALI_OPENCV_DLLS "${OpenCV_ROOT}/bin/opencv_*.dll")
  list(APPEND ALI_STAGED_DLLS ${ALI_OPENCV_DLLS})
endif()
if(WIN32 AND ALI_CERES_ROOT)
  file(GLOB ALI_CERES_DLLS "${ALI_CERES_ROOT}/bin/*.dll")
  list(APPEND ALI_STAGED_DLLS ${ALI_CERES_DLLS})
endif()
if(ALI_STAGED_DLLS)
  list(REMOVE_DUPLICATES ALI_STAGED_DLLS)
  add_custom_target(ali_vins_stage_dlls ALL
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${ALI_STAGED_DLLS} "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
    COMMENT "ALI-VINS: staging runtime DLLs into bin/")
endif()

message(STATUS "ali_vins ${PROJECT_VERSION}: app + KLT/MSCKF/SLAM driver, output to ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
