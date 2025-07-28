cmake_minimum_required(VERSION 3.20.0)
project(mrs_vins_imu_filter)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[mrs_vins_imu_filter]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

set(DEPENDENCIES
  geometry_msgs
  mrs_lib
  nav_msgs
  rclcpp
  rclcpp_components
  std_msgs
  tf2
  tf2_ros
)

set(LIBRARIES 
  MrsVinsImuFilter_VinsImuFilter
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${mrs_lib_INCLUDE_DIRS}
)

## --------------------------------------------------------------
## |                           Compile                          |
## --------------------------------------------------------------

# Imu Filter

add_library(MrsVinsImuFilter_VinsImuFilter SHARED
  src/VinsImuFilter.cpp
)

ament_target_dependencies(MrsVinsImuFilter_VinsImuFilter
  ${DEPENDENCIES}
)

rclcpp_components_register_nodes(MrsVinsImuFilter_VinsImuFilter PLUGIN "vins_imu_filter::VinsImuFilter" EXECUTABLE MrsVinsImuFilter_VinsImuFilter)

target_compile_definitions(MrsVinsImuFilter_VinsImuFilter PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

## --------------------------------------------------------------
## |                           testing                          |
## --------------------------------------------------------------

# if(ENABLE_TESTS)

#   add_subdirectory(test)

# endif()

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

ament_export_libraries(
  ${LIBRARIES}
)

install(TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY scripts
  USE_SOURCE_PERMISSIONS
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
