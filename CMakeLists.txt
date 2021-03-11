cmake_minimum_required(VERSION 3.5)
project(trivial_oculus_gui)

get_filename_component(FIPS_ROOT_DIR "../fips" ABSOLUTE)
include("${FIPS_ROOT_DIR}/cmake/fips.cmake")

fips_setup()

# == Code common to all builds =======================================

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

## C++ flags applied to all builds
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#find_package( Boost REQUIRED COMPONENTS system )

## Pull in imaging_sonar_msgs as a special exception mostly
## so we don't need to fips-ify it for this one use.
fips_include_directories(
  #${CMAKE_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
  ${FIPS_PROJECT_DIR}/../imaging_sonar_msgs/include
)

## Verbose output about ASIO handlers
#add_definitions( -DBOOST_ASIO_ENABLE_HANDLER_TRACKING )

#fips_add_subdirectory("lib/")

if( NOT FIPS_IMPORT )

  add_subdirectory("tools/")
endif()

#if( FIPS_UNITTESTS )
#  add_subdirectory( test/ )
#endif()

fips_finish()
