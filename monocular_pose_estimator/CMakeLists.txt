cmake_minimum_required(VERSION 2.4.6)
project(monocular_pose_estimator)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../monocular_pose_estimator_lib/)

# search for everything we need to build the package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  dynamic_reconfigure
  sensor_msgs
  geometry_msgs
  monocular_pose_estimator_lib
  cmake_modules
  camera_calibration_parsers
  nodelet
)

# since we need eigen search it as well
# find_package makes the ${..._INCLUDE_DIRS} ${..._LIBRARIES} variables we use later
find_package(Eigen REQUIRED)

# Add opencv
find_package(OpenCV REQUIRED)

# Generate the reconfigure options
generate_dynamic_reconfigure_options(cfg/MonocularPoseEstimator.cfg)

# export the dependencies of this package for who ever depends on us
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp cv_bridge image_transport dynamic_reconfigure sensor_msgs geometry_msgs monocular_pose_estimator_lib nodelet
  DEPENDS eigen OpenCV
)

# tell catkin where to find the headers for this project
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# make the node
add_executable(monocular_pose_estimator
  src/node.cpp src/monocular_pose_estimator.cpp
)
add_dependencies(monocular_pose_estimator ${catkin_EXPORTED_TARGETS}  ${PROJECT_NAME}_gencfg)

target_link_libraries(monocular_pose_estimator
   yaml-cpp
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

# make the node
add_executable(monocular_pose_estimator_cvcamera
  src/node.cpp src/monocular_pose_estimator_cvcamera.cpp
)
add_dependencies(monocular_pose_estimator_cvcamera ${catkin_EXPORTED_TARGETS}  ${PROJECT_NAME}_gencfg)

target_link_libraries(monocular_pose_estimator_cvcamera
   yaml-cpp
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

# make the nodelet
add_library(monocular_pose_estimator_nodelet
  src/nodelet.cpp src/monocular_pose_estimator.cpp
)

add_dependencies(monocular_pose_estimator_nodelet ${catkin_EXPORTED_TARGETS}  ${PROJECT_NAME}_gencfg)

target_link_libraries(monocular_pose_estimator_nodelet
   yaml-cpp
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)
 
install(TARGETS monocular_pose_estimator monocular_pose_estimator_nodelet monocular_pose_estimator_cvcamera
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch marker_positions
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES monocular_pose_estimator_nodelet.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)