cmake_minimum_required(VERSION 2.4.6)
project(monocular_pose_estimator_lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(catkin REQUIRED COMPONENTS cmake_modules roscpp)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)
set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})  ### TODO: TEMPORARY FOR 14.04


catkin_package(
  INCLUDE_DIRS include
  DEPENDS EIGEN3 opencv
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

file(GLOB SOURCEFILES "src/*.cpp")

option(MPE_ENABLE_DEBUG_IMAGES "Enable the display of debugging images (works only if you have a graphical interface)" OFF)

if(MPE_ENABLE_DEBUG_IMAGES)
	add_definitions(-DMPE_SHOW_DEBUG_IMAGE)
endif(MPE_ENABLE_DEBUG_IMAGES)

add_library(${PROJECT_NAME} SHARED ${SOURCEFILES})
add_dependencies(${PROJECT_NAME} SHARED ${catkin_EXPORTED_TARGETS} ) 
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION} 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}    
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
