cmake_minimum_required(VERSION 3.5)
project(ndt_matching)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_core REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL 1.3 REQUIRED)

add_library(ndt_lib src/ndt_lib.cpp)
target_include_directories(ndt_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(ndt_lib PRIVATE "NDT_MATCHING_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ndt_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ndt_node src/ndt_node.cpp)

ament_target_dependencies(ndt_node
  ndt_lib
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "rcutils")

target_include_directories(ndt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(ndt_node ndt_lib ${PCL_LIBRARIES})

install(TARGETS ndt_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
 
  include_directories(include)

  #add unit test source and dependencies
  ament_add_gtest(ndt_test test/ndt_test.cpp ENV PCD=map.pcd)
  target_link_libraries(ndt_test ndt_lib ${PCL_LIBRARIES})

  #add integration test source and dependencies
  ament_add_gtest(integration_test test/integration_test.cpp )
  ament_target_dependencies(integration_test
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "rcutils")
  target_link_libraries(integration_test ${PCL_LIBRARIES})
 
endif()

ament_export_include_directories(
  include
)
ament_export_interfaces(
  export_${PROJECT_NAME}
)
ament_export_libraries(
  ndt_lib
)

ament_package()
