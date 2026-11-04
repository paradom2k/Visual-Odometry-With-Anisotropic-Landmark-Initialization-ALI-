# VINS-Mono estimator sources, minus the ROS node and visualization wrappers.
set(VINS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/vins_estimator")
set(FT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/feature_tracker")
set(CAM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/camera_model")

set(VINS_CORE_SOURCES
  ${VINS_ROOT}/src/estimator.cpp
  ${VINS_ROOT}/src/feature_manager.cpp
  ${VINS_ROOT}/src/parameters.cpp
  ${VINS_ROOT}/src/factor/marginalization_factor.cpp
  ${VINS_ROOT}/src/factor/pose_local_parameterization.cpp
  ${VINS_ROOT}/src/factor/projection_factor.cpp
  ${VINS_ROOT}/src/factor/projection_td_factor.cpp
  ${VINS_ROOT}/src/initial/initial_aligment.cpp
  ${VINS_ROOT}/src/initial/initial_ex_rotation.cpp
  ${VINS_ROOT}/src/initial/initial_sfm.cpp
  ${VINS_ROOT}/src/initial/solve_5pts.cpp
  ${VINS_ROOT}/src/utility/utility.cpp
  ${FT_ROOT}/src/feature_tracker.cpp
  ${FT_ROOT}/src/parameters.cpp
  ${CAM_ROOT}/src/camera_models/Camera.cc
  ${CAM_ROOT}/src/camera_models/CameraFactory.cc
  ${CAM_ROOT}/src/camera_models/CataCamera.cc
  ${CAM_ROOT}/src/camera_models/EquidistantCamera.cc
  ${CAM_ROOT}/src/camera_models/PinholeCamera.cc
  ${CAM_ROOT}/src/camera_models/ScaramuzzaCamera.cc
  ${CAM_ROOT}/src/gpl/gpl.cc
)

add_library(vins_mono_core STATIC ${VINS_CORE_SOURCES})
target_include_directories(vins_mono_core PUBLIC
  "${CAM_ROOT}/include"
  "${FT_ROOT}/src"
  "${VINS_ROOT}/src"
  "${VINS_ROOT}/src/utility"
  "${VINS_ROOT}/src/factor"
  "${VINS_ROOT}/src/initial"
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/compat"
  "${VINS_EIGEN_INCLUDE}"
  "${VINS_BOOST_ROOT}"
  "${VINS_CERES_ROOT}/include"
)
target_include_directories(vins_mono_core SYSTEM PUBLIC "${VINS_OPENCV_INCLUDE}")

target_link_libraries(vins_mono_core PUBLIC
  ${VINS_OPENCV_LIBS}
  "${VINS_CERES_ROOT}/lib/ceres.lib"
  "${VINS_CERES_ROOT}/lib/glog.lib"
)
if(WIN32)
  target_link_libraries(vins_mono_core PUBLIC ws2_32)
endif()
