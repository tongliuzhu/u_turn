cmake_minimum_required (VERSION 3.5)
project(hybrid_astar)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -pthread")

find_package(OpenCV REQUIRED)
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS})

file( GLOB_RECURSE ALL_SOURCES ${PROJECT_SOURCE_DIR}/src/*.cpp )
add_executable(${PROJECT_NAME} ${ALL_SOURCES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${catkin_LIBRARIES})
