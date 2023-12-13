cmake_minimum_required(VERSION 2.8.3)
project(costmap_2d)

find_package(
  Boost REQUIRED COMPONENTS system
        )
find_package(OpenCV REQUIRED)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
include_directories(
    include
)

include_directories(${Boost_INCLUDE_DIRS}
                     ${OpenCV_INCLUDE_DIRS})


add_library(costmap_2d_lib
  src/array_parser.cpp
  src/costmap_2d.cpp
  src/layer.cpp
  src/layered_costmap.cpp
  src/costmap_2d_ros.cpp
  src/costmap_math.cpp
  src/footprint.cpp
  src/costmap_layer.cpp
  src/navigation.cpp
)


add_library(layers
  plugins/inflation_layer.cpp
  plugins/obstacle_layer.cpp
  plugins/static_layer.cpp
)
find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})
target_link_libraries(layers costmap_2d_lib )

add_executable(costmap main.cpp)

target_link_libraries(
        costmap
        costmap_2d_lib
        layers
        ${Boost_LIBRARIES}
        pthread 
        boost_thread
        ${OpenCV_LIBS}
        yaml-cpp
)
