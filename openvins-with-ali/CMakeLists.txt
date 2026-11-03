# openvins_ali -- OpenVINS with Anisotropic Landmark Initialization (ALI).
#
# A self-contained project.  It needs OpenCV, Eigen, Boost and Ceres from outside, and vendors the estimation kernel
# it uses (thirdparty/openvins).  A default build produces exactly two executables, both into bin/:
#
#   bin/openvins_ali_app.exe           the application: pick a configuration, run it, watch the log, see the plot
#   bin/openvins_ali_run.exe  the kernel driver the application starts
#
# together with the runtime DLLs they need, so bin/ runs on its own.  build/ is scratch space.
#
#   cmake -G Ninja -S . -B build -DCMAKE_BUILD_TYPE=Release ^
#         -DOpenCV_ROOT=<opencv install> ^
#         -DEigen3_ROOT=<dir containing Eigen/Core> ^
#         -DALI_CERES_ROOT=<ceres install> ^
#         -DALI_BOOST_ROOT=<boost root>
#
# build_msvc.bat does exactly this with the paths of the machine it was developed on.
cmake_minimum_required(VERSION 3.20)
project(openvins_ali VERSION 0.2.0 LANGUAGES CXX)

# C++14, not 17: the vendored OpenVINS kernel is written to it, and the tree it came from requires it.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# ---------------------------------------------------------------- layout
# Executables go to bin/ rather than into the CMake binary directory, so that build/ can be deleted at any time
# without losing anything and the three directories a user is meant to see are the three they get:
#
#   bin/   the application, the kernel driver, and the DLLs they need
#   conf/  the configuration the application lists
#   sol/   what a run writes (created on the first run)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")
if(MSVC)
  # Multi-configuration generators append the configuration name; pinning the per-config variables keeps the output
  # path the same either way.
  foreach(config Debug Release RelWithDebInfo MinSizeRel)
    string(TOUPPER "${config}" upper)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${upper} "${CMAKE_CURRENT_SOURCE_DIR}/bin")
  endforeach()
endif()

# ---------------------------------------------------------------- dependencies
set(OpenCV_ROOT "" CACHE PATH "OpenCV install root (contains include/opencv2 and the import libraries)")
set(Eigen3_ROOT "" CACHE PATH "Eigen root (contains Eigen/Core)")

# Eigen: prefer the explicit root, then a normal find_package.
if(Eigen3_ROOT AND EXISTS "${Eigen3_ROOT}/Eigen/Core")
  set(ALI_EIGEN_INCLUDE "${Eigen3_ROOT}")
  message(STATUS "openvins_ali: Eigen from Eigen3_ROOT = ${Eigen3_ROOT}")
else()
  find_package(Eigen3 QUIET)
  if(Eigen3_FOUND)
    set(ALI_EIGEN_INCLUDE "${EIGEN3_INCLUDE_DIR}")
    message(STATUS "openvins_ali: Eigen from find_package = ${EIGEN3_INCLUDE_DIR}")
  else()
    message(FATAL_ERROR "Eigen not found: set -DEigen3_ROOT=<dir containing Eigen/Core>")
  endif()
endif()

# OpenCV.  Deliberately not find_package() first: a Windows OpenCV pack records the compiler it was built with in
# OpenCVConfig.cmake and refuses a different one ("Found OpenCV Windows Pack but it has no binaries compatible with
# your configuration"), which is exactly what happens when the pack is registered for vc16 and the build uses a
# newer MSVC.  Since all this project needs is the headers and the import libraries, it resolves them directly when
# OpenCV_ROOT is given and only falls back to find_package otherwise.
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
  message(STATUS "openvins_ali: OpenCV headers ${ALI_OPENCV_INCLUDE}, ${ALI_OPENCV_LIB_COUNT} library file(s)")
else()
  find_package(OpenCV REQUIRED COMPONENTS core imgproc video calib3d imgcodecs)
  set(ALI_OPENCV_INCLUDE "${OpenCV_INCLUDE_DIRS}")
  set(ALI_OPENCV_LIBS "${OpenCV_LIBS}")
  message(STATUS "openvins_ali: OpenCV ${OpenCV_VERSION} from find_package")
endif()

# ---------------------------------------------------------------- the kernel
# OpenVINS's ov_core + ov_init + ov_msckf with ROS removed.  cmake/openvins.cmake documents the source list and where
# every one of its dependencies comes from.
include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/openvins.cmake")

# ---------------------------------------------------------------- the kernel driver
# Feeds a manifest plus its IMU csv into VioManager and writes the trajectory.  Kept as its own executable rather
# than folded into the application so that a run can be reproduced from a command line, and so that a crash is
# reported in the window's log instead of taking the window with it.
add_executable(openvins_ali_run src/openvins_run.cpp)
target_link_libraries(openvins_ali_run PRIVATE openvins_ali_core)
if(MSVC)
  target_compile_options(openvins_ali_run PRIVATE /EHsc /utf-8 /bigobj)
  target_compile_definitions(openvins_ali_run PRIVATE
    _CRT_SECURE_NO_WARNINGS
    _USE_MATH_DEFINES
    NOMINMAX
    WIN32_LEAN_AND_MEAN
  )
endif()

# ---------------------------------------------------------------- the application
# Links neither OpenCV nor the kernel: it only needs the Win32 libraries, because it starts the driver above as a
# child process.  That is what keeps the window alive when the estimator crashes, and it keeps the application
# buildable even if a solver dependency is missing.
add_executable(openvins_ali_app src/app_main.cpp)
if(MSVC)
  target_compile_options(openvins_ali_app PRIVATE /EHsc /utf-8)
  target_compile_definitions(openvins_ali_app PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()
if(WIN32)
  # comdlg32 is the file-open dialog the plot's "Load .txt" uses; the rest are the window itself.
  target_link_libraries(openvins_ali_app PRIVATE user32 gdi32 comctl32 shell32 comdlg32)
endif()

# ---------------------------------------------------------------- DLLs beside the executables
# bin/ has to be runnable on its own, so the runtime DLLs are copied next to the executables rather than relying on
# their bin directories being on PATH.  Both sets are needed: OpenCV by the driver's image decoding, and the
# Ceres/glog/gflags/OpenBLAS/LAPACK set by the kernel, which links a shared Ceres.
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
  add_custom_target(openvins_ali_stage_dlls ALL
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${ALI_STAGED_DLLS} "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
    COMMENT "openvins_ali: staging runtime DLLs into bin/")
endif()

message(STATUS "openvins_ali ${PROJECT_VERSION}: application + OpenVINS kernel, output to ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
