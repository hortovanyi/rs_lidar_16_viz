cmake_minimum_required(VERSION 3.5)
project(rs_lidar_16_viz)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(PCL 1.10 REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)

include_directories(include SYSTEM ${PCL_INCLUDE_DIRS})

add_executable(rs_lidar_16_viz
  src/rs_lidar_16_viz.cpp
)

target_link_libraries(rs_lidar_16_viz
                    ${PCL_LIBRARIES}
                    ${Boost_LIBRARIES}       
                      pcap
                      pthread
)

