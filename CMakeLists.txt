cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)
add_compile_options(-std=c++11)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  genmsg
  message_generation
  tf
)

add_service_files(
  FILES
  serviceFile.srv
)
## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(#include 
${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(
    testTalk
    test/testTalk.launch
    test/main.cpp
    test/testTalk.cpp
  )

  target_link_libraries(
    testTalk
    ${catkin_LIBRARIES}
  )

  add_dependencies(
    testTalk
    talker
    listener
    ${catkin_EXPORTED_TARGETS}
  )
endif()
