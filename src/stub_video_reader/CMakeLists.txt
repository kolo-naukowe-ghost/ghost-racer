cmake_minimum_required(VERSION 2.8.3)
project(stub_video_reader)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  std_srvs
  message_generation
  cv_bridge
  image_transport
)

find_package(OpenCV 3 REQUIRED)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/stub_video_reader.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/VideoReader.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
