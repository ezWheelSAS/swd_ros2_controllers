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
find_package(rclcpp REQUIRED) # new name of roscpp
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED) # new name of message_generation


add_executable(swd_diff_drive_controller src/DiffDriveController.cpp)

target_include_directories(swd_diff_drive_controller PRIVATE include)

ament_target_dependencies(swd_diff_drive_controller
  geometry_msgs
  std_msgs
  rclcpp
)


#------------------------------------------------------------------------------
# Build definition (to do before calling 'project()')
#------------------------------------------------------------------------------

# Build type
if(NOT CMAKE_BUILD_TYPE)
  set(
    CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE
  )
endif(NOT CMAKE_BUILD_TYPE)

# CASE OF C PROJECT
set(CMAKE_C_STANDARD 99)
if(NOT CMAKE_C_FLAGS)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Wformat-security " CACHE STRING "C99 compilation flags" FORCE)
endif(NOT CMAKE_C_FLAGS)

# CASE OF CPP PROJECT
set(CMAKE_CXX_STANDARD 11)
if(NOT CMAKE_CXX_FLAGS)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wformat-security" CACHE STRING "C++11 compilation flags" FORCE)
endif(NOT CMAKE_CXX_FLAGS)


# The code needs C++14 to work!
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


#------------------------------------------------------------------------------
# Project definition, variable and dependencies
#------------------------------------------------------------------------------

set(CMAKE_PROJECT_NAME ${PROJECT_NAME})
set(CMAKE_SYSTEM_NAME Linux)

set(CMAKE_PROJECT_BRIEF "ez-Wheel SWD ROS2 Controllers package")

# if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
#     set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/image CACHE PATH "" FORCE)
# endif (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

set(PKG_CONFIG_PATH /opt/ezw/usr/lib/pkgconfig CACHE PATH "")
set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${PKG_CONFIG_PATH}")
message(STATUS "Pkgconfig extra path are: $ENV{PKG_CONFIG_PATH} ")

# Allow CTest
enable_testing()

# Allow subdirectory test and config
option(ENABLE_TESTS "Enable Tests" 1)

# find_package(Doxygen)
# option(ENABLE_DOCS "Build API documentation" ${DOXYGEN_FOUND})

# Module dependencies used as pkgconfig or direct filepath/dirpath
set(LIST_MODULES_PKG_CONFIG_NAME "ezw-types ezwLog ezw-smc-core" CACHE STRING "Lists of pkgconfig modules dependencies" FORCE)

unset(__pkg_config_checked_EZW_TYPES CACHE)
unset(__pkg_config_checked_EZW_LOG CACHE)
unset(__pkg_config_checked_EZW_SMC_CORE CACHE)

set(EZW_TYPES_PKG_CONFIG_NAME "ezw-types" CACHE STRING "ezw-types pkg config name")
set(EZW_LOG_PKG_CONFIG_NAME "ezwLog" CACHE STRING "ezwLog pkg config name")
set(EZW_SMC_CORE_PKG_CONFIG_NAME "ezw-smc-core" CACHE STRING "Module EZW_SMC_CORE_CONFIG pkg config name")


#------------------------------------------------------------------------------
# Project version
#------------------------------------------------------------------------------

# Get the latest abbreviated commit hash of the working branch
execute_process(
  COMMAND git log -1 --format=%h
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE GIT_COMMIT_HASH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

#------------------------------------------------------------------------------
# pkgconfig
#------------------------------------------------------------------------------

find_package(PkgConfig REQUIRED)




#------------------------------------------------------------------------------
# Project configuration
#------------------------------------------------------------------------------

include_directories(include)
include_directories(/opt/ezw/usr/include/dbus-1.0)

include_directories(${CMAKE_SOURCE_DIR}/include)

if(HAS_CONFIG)
  message(STATUS "Adding config from ${CMAKE_SOURCE_DIR}/config/")
  include_directories(${CMAKE_SOURCE_DIR}/config)
endif(HAS_CONFIG)

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


######

target_link_libraries(
  swd_diff_drive_controller
  ${EZW_LOG_LIBRARIES}
  ${EZW_SMC_CORE_LIBRARIES}

)


install(TARGETS swd_diff_drive_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)



ament_package()
