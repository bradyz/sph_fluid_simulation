cmake_minimum_required(VERSION 2.8)
project(fluid_simulation)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(LIBIGL REQUIRED)

if (NOT LIBIGL_FOUND)
   message(FATAL_ERROR "LibIGL not found.")
endif()

option(LIBIGL_WITH_NANOGUI     ON)
option(LIBIGL_WITH_VIEWER      ON)
option(LIBIGL_WITH_OPENGL      ON)
option(LIBIGL_WITH_OPENGL_GLFW ON)

add_subdirectory("${PROJECT_SOURCE_DIR}/lib/libigl/shared/cmake" "libigl")

# Prepare the build environment
include_directories(${LIBIGL_INCLUDE_DIRS})
add_definitions(${LIBIGL_DEFINITIONS})

# Fluid simulation.
FILE(GLOB SPH_SRC ${PROJECT_SOURCE_DIR}/src/*.cpp)
add_executable(sph ${SPH_SRC} ${LIBIGL_EXTRA_SOURCES})
target_link_libraries(sph ${LIBIGL_LIBRARIES} ${LIBIGL_EXTRA_LIBRARIES})
