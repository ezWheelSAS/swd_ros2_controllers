cmake_minimum_required(VERSION 3.5)
project(swd_ros2_controllers)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)

#------------------------------------------------------------------------------
# Project definition, variable and dependencies
#------------------------------------------------------------------------------

set(CMAKE_PROJECT_NAME ${PROJECT_NAME})
set(CMAKE_SYSTEM_NAME Linux)

set(CMAKE_PROJECT_BRIEF "ez-Wheel SWD ROS2 Controllers package")

set(PKG_CONFIG_PATH /opt/ezw/usr/lib/pkgconfig CACHE PATH "")
set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${PKG_CONFIG_PATH}")
message(STATUS "Pkgconfig extra path are: $ENV{PKG_CONFIG_PATH} ")

# Module dependencies used as pkgconfig or direct filepath/dirpath
set(LIST_MODULES_PKG_CONFIG_NAME "ezw-types ezwLog ezw-smc-core" CACHE STRING "Lists of pkgconfig modules dependencies" FORCE)

unset(__pkg_config_checked_EZW_TYPES CACHE)
unset(__pkg_config_checked_EZW_LOG CACHE)
unset(__pkg_config_checked_EZW_SMC_CORE CACHE)

set(EZW_TYPES_PKG_CONFIG_NAME "ezw-types" CACHE STRING "ezw-types pkg config name")
set(EZW_LOG_PKG_CONFIG_NAME "ezwLog" CACHE STRING "ezwLog pkg config name")
set(EZW_SMC_CORE_PKG_CONFIG_NAME "ezw-smc-core" CACHE STRING "Module EZW_SMC_CORE_CONFIG pkg config name")

#------------------------------------------------------------------------------
# pkgconfig
#------------------------------------------------------------------------------

find_package(PkgConfig REQUIRED)

#############
## Clang ##
#############

# Allow Clang-tidy
# set(CMAKE_CXX_CLANG_TIDY clang-tidy-12 --config-file=${CMAKE_SOURCE_DIR}/.clang-tidy --use-color --header-filter=${CMAKE_SOURCE_DIR}/${CMAKE_PROJECT_NAME}/include/)
set(DCMAKE_EXPORT_COMPILE_COMMANDS ON)


#------------------------------------------------------------------------------
# Project configuration
#------------------------------------------------------------------------------

include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/msg
  ${CMAKE_SOURCE_DIR}/msg/${PROJECT_NAME}
)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

# Use project pkg config
pkg_check_modules(EZW_TYPES REQUIRED ezw-types)
pkg_check_modules(EZW_LOG REQUIRED ezwLog)
pkg_check_modules(EZW_SMC_CORE REQUIRED ezw-smc-core)

# Add include dirs. The case not configured will be empty
message(STATUS "
Add inc directory:
EZW_TYPES_INCLUDE_DIRS : ${EZW_TYPES_INCLUDE_DIRS}
EZW_LOG_INCLUDE_DIRS : ${EZW_LOG_INCLUDE_DIRS}
EZW_SMC_CORE_INCLUDE_DIRS : ${EZW_SMC_CORE_INCLUDE_DIRS}
target_include_directories(target include ${Boost_INCLUDE_DIRS})
-------------------")

include_directories(
  ${EZW_TYPES_INCLUDE_DIRS}
  ${EZW_LOG_INCLUDE_DIRS}
  ${EZW_SMC_CORE_INCLUDE_DIRS}
)

# Add library dirs. The case not configured will be empty
message(STATUS "
Add library directory:
EZW_LOG_LIBRARY_DIRS : ${EZW_LOG_LIBRARY_DIRS}
EZW_SMC_CORE_LIBRARY_DIRS : ${EZW_SMC_CORE_LIBRARY_DIRS}
------------------------------")

link_directories(
  ${EZW_LOG_LIBRARY_DIRS}
  ${EZW_TOOLS_LIBRARY_DIRS}
  ${EZW_SMC_CORE_LIBRARY_DIRS}
)

# Add required definitions from pkgconfig
message(STATUS "
Add definitions:
EZW_LOG_CFLAGS : ${EZW_LOG_CFLAGS}
EZW_LOG_LDFLAGS : ${EZW_LOG_LDFLAGS}
EZW_SMC_CORE_CFLAGS : ${EZW_SMC_CORE_CFLAGS}
EZW_SMC_CORE_LDFLAGS : ${EZW_SMC_CORE_LDFLAGS}
------------------------------")

add_definitions(
  ${EZW_LOG_CFLAGS}
  ${EZW_LOG_LDFLAGS}
  ${EZW_SMC_CORE_CFLAGS}
  ${EZW_SMC_CORE_LDFLAGS}
)

file(GLOB_RECURSE SOURCES_DIFF_DRIVE_CRTL src/diff_drive_controller/*.cpp)

add_executable(swd_diff_drive_controller ${SOURCES_DIFF_DRIVE_CRTL})

ament_target_dependencies(swd_diff_drive_controller
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_msgs
)

target_link_libraries(
  swd_diff_drive_controller
  ${EZW_LOG_LIBRARIES}
  ${EZW_SMC_CORE_LIBRARIES}
)

install(TARGETS
  swd_diff_drive_controller
  DESTINATION lib/${PROJECT_NAME}
)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SafetyFunctions.msg"
)

ament_package()
