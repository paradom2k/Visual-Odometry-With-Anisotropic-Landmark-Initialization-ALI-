# ali-orbslam3 -- ORB-SLAM3 with Anisotropic Landmark Initialization (ALI).
#
# WHAT THIS IS
#   Upstream ORB-SLAM3 1.0 under `thirdparty/orbslam3/`, plus a Windows/MSVC port and the ALI changes in
#   `patches/orbslam3/overlay/`.  CMake copies the pristine tree into the build directory and overlays the
#   changed files there, so the checked-in upstream tree stays byte-identical and the provenance of every
#   changed file is a file in `patches/`.
#
#   Two executables:
#     bin/ali_run.exe   headless: one conf file in, one trajectory out
#     bin/ali_ui.exe    the same run with a window: pick a conf, press Run, watch the trajectory and features
#
# DEPENDENCIES -- local installations, no network, no package manager
#   OpenCV  a self-contained installation (include/, lib/*.lib, bin/*.dll)
#   Eigen   an include tree containing Eigen/Core
#   GLFW    only for ali_ui; -DALI_BUILD_UI=OFF skips it
#   Boost   not required: the handful of Boost.Serialization and Boost.Algorithm headers ORB-SLAM3 uses are
#           replaced by the stand-ins in src/compat/orbslam3/boost/.  If a real Boost tree is named it is
#           used for headers instead.
#
# BUILD
#   cmake -G Ninja -S . -B build -DCMAKE_BUILD_TYPE=Release ^
#         -DCMAKE_CXX_COMPILER=<cl.exe> ^
#         -DALI_OPENCV_ROOT=<opencv install> ^
#         -DALI_EIGEN_ROOT=<dir containing Eigen/Core> ^
#         -DALI_EIGEN_PARENT=<dir containing eigen3/> ^
#         -DALI_GLFW_ROOT=<glfw root>
#   cmake --build build -j 4
#
#   build_msvc.bat does exactly this with the paths of the machine it was developed on.
cmake_minimum_required(VERSION 3.20)
project(ali_orbslam3 VERSION 1.0.0 LANGUAGES C CXX)

# The build copies the vendored ORB-SLAM3 tree into the build directory before overlaying the patches.
# An in-source configure would make that directory the vendored source itself; refuse it.
if(CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR)
  message(FATAL_ERROR "ali-orbslam3 requires an out-of-source build: cmake -S . -B build")
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")
if(MSVC)
  foreach(config Debug Release RelWithDebInfo MinSizeRel)
    string(TOUPPER "${config}" upper)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${upper} "${CMAKE_CURRENT_SOURCE_DIR}/bin")
  endforeach()
endif()

# ---------------------------------------------------------------- layout
set(ALI_ORB_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/orbslam3")
set(ALI_ORB_OVERLAY "${CMAKE_CURRENT_SOURCE_DIR}/patches/orbslam3/overlay")
set(ALI_ORB "${CMAKE_BINARY_DIR}/thirdparty/orbslam3")
set(ALI_COMPAT "${CMAKE_CURRENT_SOURCE_DIR}/src/compat/orbslam3")
set(ALI_SRC "${CMAKE_CURRENT_SOURCE_DIR}/src/ali")

file(GLOB_RECURSE ALI_ORB_OVERLAY_INPUTS "${ALI_ORB_OVERLAY}/*")
file(GLOB_RECURSE ALI_ORB_UPSTREAM_INPUTS "${ALI_ORB_SOURCE}/*")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  ${ALI_ORB_OVERLAY_INPUTS} ${ALI_ORB_UPSTREAM_INPUTS})
file(REMOVE_RECURSE "${ALI_ORB}")
# copy_directory rather than file(COPY): file(COPY) was measured to leave some destination files at their
# upstream content when source and destination are written in the same configure pass, which made the build
# use the unpatched header while the overlay beside it looked correct.
execute_process(COMMAND "${CMAKE_COMMAND}" -E copy_directory "${ALI_ORB_SOURCE}" "${ALI_ORB}"
                COMMAND_ERROR_IS_FATAL ANY)
if(EXISTS "${ALI_ORB_OVERLAY}")
  execute_process(COMMAND "${CMAKE_COMMAND}" -E copy_directory "${ALI_ORB_OVERLAY}" "${ALI_ORB}"
                  COMMAND_ERROR_IS_FATAL ANY)
endif()

# ---------------------------------------------------------------- dependency roots
set(ALI_OPENCV_ROOT "E:/jo_gvil/jo_gvil 1.0/build/opencv-install" CACHE PATH "OpenCV installation root")
set(ALI_EIGEN_ROOT "E:/jo_gvil/jo_gvil 1.0/github_prj/GraphRTK-INS-main/src/Third-party/Eigen"
    CACHE PATH "Eigen include root (so that <Eigen/Core> resolves)")
set(ALI_EIGEN_PARENT "E:/jo_gvil/jo_gvil 1.0/github_prj/GraphRTK-INS-main/src/Third-party/ceres/include"
    CACHE PATH "the directory containing eigen3/ (g2o includes eigen3/Eigen/...)")
set(ALI_BOOST_ROOT "" CACHE PATH "optional real Boost headers; empty uses the stand-ins in src/compat")
set(ALI_GLFW_ROOT "E:/jo_gvil/jo_gvil 1.0/github_prj/GraphRTK-INS-main/src/Third-party/GLFW"
    CACHE PATH "GLFW root (include/ and lib/)")
option(ALI_BUILD_UI "build ali_ui (needs GLFW)" ON)

# ---------------------------------------------------------------- OpenCV
find_path(ALI_OPENCV_INCLUDE "opencv2/core.hpp" HINTS "${ALI_OPENCV_ROOT}/include" NO_DEFAULT_PATH)
if(NOT ALI_OPENCV_INCLUDE)
  message(FATAL_ERROR "OpenCV headers not found under ${ALI_OPENCV_ROOT}")
endif()
file(GLOB ALI_OPENCV_LIBS "${ALI_OPENCV_ROOT}/lib/opencv_*.lib"
                          "${ALI_OPENCV_ROOT}/x64/vc*/lib/opencv_*.lib")
if(NOT ALI_OPENCV_LIBS)
  message(FATAL_ERROR "no OpenCV import libraries under ${ALI_OPENCV_ROOT}")
endif()
list(LENGTH ALI_OPENCV_LIBS ALI_OPENCV_COUNT)
message(STATUS "ali: OpenCV ${ALI_OPENCV_INCLUDE}, ${ALI_OPENCV_COUNT} import library(ies)")

# ---------------------------------------------------------------- the ORB-SLAM3 core
# Viewer.cc and MapDrawer.cc are ORB-SLAM3's Pangolin drawing code.  This project has no Pangolin; the
# methods the core calls are defined in src/compat/orbslam3/port/viewer_stubs.cpp instead.
file(GLOB_RECURSE ALI_ORB_CORE "${ALI_ORB}/src/*.cc" "${ALI_ORB}/src/*.cpp")
list(FILTER ALI_ORB_CORE EXCLUDE REGEX "/Viewer\\.(cc|cpp)$")
list(FILTER ALI_ORB_CORE EXCLUDE REGEX "/MapDrawer\\.(cc|cpp)$")

file(GLOB ALI_DBOW2 "${ALI_ORB}/Thirdparty/DBoW2/DBoW2/*.cpp" "${ALI_ORB}/Thirdparty/DBoW2/DUtils/*.cpp")
file(GLOB_RECURSE ALI_G2O "${ALI_ORB}/Thirdparty/g2o/g2o/*.cpp")
list(FILTER ALI_G2O EXCLUDE REGEX "_test\\.(cpp|cc)$")
# Sophus is header-only here.  Its py/cpp_gencode fragments are binding codegen, not translation units, and
# its test/ and examples/ trees each carry their own main().
file(GLOB_RECURSE ALI_SOPHUS "${ALI_ORB}/Thirdparty/Sophus/*.cpp")
list(FILTER ALI_SOPHUS EXCLUDE REGEX "/py/")
list(FILTER ALI_SOPHUS EXCLUDE REGEX "/(test|tests|examples)/")
list(FILTER ALI_SOPHUS EXCLUDE REGEX "test_.*\\.(cpp|cc)$")
list(FILTER ALI_SOPHUS EXCLUDE REGEX "HelloSO3\\.(cpp|cc)$")

set(ALI_SUPPORT
  "${CMAKE_CURRENT_SOURCE_DIR}/src/common/log.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/common/types.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/common/time_system.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/common/geodesy.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/data/bag_reader.cpp"
)

add_library(ali_orbslam3 STATIC
  ${ALI_ORB_CORE}
  "${ALI_COMPAT}/port/viewer_stubs.cpp"
  ${ALI_DBOW2}
  ${ALI_G2O}
  ${ALI_SOPHUS}
  ${ALI_SUPPORT}
)

target_include_directories(ali_orbslam3 PUBLIC
  # compat first on purpose: it holds the stand-ins for <pangolin/pangolin.h>, <unistd.h>, <openssl/md5.h>
  # and the Boost-serialization subset.  Found after the real thing, a stand-in is no stand-in at all.
  "${ALI_COMPAT}"
  "${ALI_ORB}"           # ported headers include "Thirdparty/DBoW2/..." relative to the port root
  "${ALI_ORB}/include"
  "${ALI_ORB}/include/CameraModels"
  "${ALI_ORB}/Thirdparty"
  "${ALI_ORB}/Thirdparty/Sophus"
  "${ALI_ORB}/Thirdparty/g2o"
  "${ALI_EIGEN_ROOT}"
  "${ALI_EIGEN_PARENT}"
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
if(ALI_BOOST_ROOT)
  target_include_directories(ali_orbslam3 PUBLIC "${ALI_BOOST_ROOT}")
endif()
target_include_directories(ali_orbslam3 SYSTEM PUBLIC ${ALI_OPENCV_INCLUDE})
target_link_libraries(ali_orbslam3 PUBLIC ${ALI_OPENCV_LIBS})

if(MSVC)
  target_compile_options(ali_orbslam3 PUBLIC /EHsc /utf-8 /bigobj)
  target_compile_options(ali_orbslam3 PRIVATE /FI "${ALI_COMPAT}/jo_compat.h")
  target_compile_definitions(ali_orbslam3 PUBLIC _CRT_SECURE_NO_WARNINGS BOOST_ALL_NO_LIB NOMINMAX)
  if(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_options(ali_orbslam3 PRIVATE /O2 /Ob2)
  endif()
endif()

# ---------------------------------------------------------------- the project layer
add_library(ali_core STATIC
  "${ALI_SRC}/ali_conf.cpp"
  "${ALI_SRC}/ali_files.cpp"
  "${ALI_SRC}/ali_sensors.cpp"
  "${ALI_SRC}/ali_run.cpp"
)
target_include_directories(ali_core PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
  "${ALI_SRC}"
  "${ALI_ORB}/include"
  "${ALI_ORB}"
)
target_link_libraries(ali_core PUBLIC ali_orbslam3)
if(MSVC)
  target_compile_options(ali_core PUBLIC /EHsc /utf-8 /bigobj)
  target_compile_definitions(ali_core PUBLIC _CRT_SECURE_NO_WARNINGS BOOST_ALL_NO_LIB NOMINMAX)
endif()

add_executable(ali_run "${ALI_SRC}/ali_main.cpp")
target_link_libraries(ali_run PRIVATE ali_core)

# Thread stack.  On Windows a std::thread created with dwStackSize 0 inherits the executable's PE
# SizeOfStackReserve, so this is the stack of the estimator worker and of ORB-SLAM3's mapping/closing
# threads as well.  ORB-SLAM3 runs nested g2o solves on those threads; the linker default of 1 MB was
# observed to fault there.  64 MB is address-space reserve, committed on demand, not 64 MB of memory.
function(ali_link_flags target)
  if(MSVC)
    target_link_options(${target} PRIVATE "/STACK:67108864")
  endif()
endfunction()
ali_link_flags(ali_run)

# ---------------------------------------------------------------- the UI
if(ALI_BUILD_UI)
  find_path(ALI_GLFW_INCLUDE "GLFW/glfw3.h" HINTS "${ALI_GLFW_ROOT}/include" NO_DEFAULT_PATH)
  find_library(ALI_GLFW_LIBRARY NAMES glfw3 glfw HINTS "${ALI_GLFW_ROOT}/lib" NO_DEFAULT_PATH)
  if(ALI_GLFW_INCLUDE AND ALI_GLFW_LIBRARY)
    add_executable(ali_ui "${ALI_SRC}/ali_ui.cpp")
    target_include_directories(ali_ui PRIVATE "${ALI_GLFW_INCLUDE}")
    target_link_libraries(ali_ui PRIVATE ali_core ${ALI_GLFW_LIBRARY} opengl32)
    ali_link_flags(ali_ui)
    set(ALI_HAVE_UI TRUE)
  else()
    message(WARNING "ali: GLFW not found under ${ALI_GLFW_ROOT}; ali_ui will not be built")
  endif()
endif()

# ---------------------------------------------------------------- runtime DLLs beside the executables
file(GLOB ALI_STAGED_DLLS "${ALI_OPENCV_ROOT}/bin/opencv_*.dll")
if(MSVC AND ALI_HAVE_UI)
  file(GLOB ALI_OMP_DLLS "${ALI_GLFW_ROOT}/../*omp*.dll" "${ALI_GLFW_ROOT}/bin/*omp*.dll")
  list(APPEND ALI_STAGED_DLLS ${ALI_OMP_DLLS})
endif()
if(ALI_STAGED_DLLS)
  list(REMOVE_DUPLICATES ALI_STAGED_DLLS)
  add_custom_target(ali_stage_dlls ALL
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different ${ALI_STAGED_DLLS} "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
    COMMENT "ali: staging runtime DLLs into bin/")
endif()

message(STATUS "ali-orbslam3 ${PROJECT_VERSION}: ORB-SLAM3 + ALI, output to ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
