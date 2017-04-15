cmake_minimum_required(VERSION 2.8)
project(GLSL)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11 -g" )
set(CMAKE_FIND_LIBRARY_SUFFIXES "${CMAKE_FIND_LIBRARY_SUFFIXES}")

link_directories(
    "/usr/local/lib"
)

include_directories(
    "/usr/local/include"
)

find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})

find_package(GLEW REQUIRED)
include_directories(${GLEW_INCLUDE_DIRS})
link_libraries(${GLEW_LIBRARIES})

find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)
include_directories(${GLFW_INCLUDE_DIRS})

find_package(JPEG REQUIRED)
include_directories(${JPEG_INCLUDE_DIR})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Fluid simulation.
FILE(GLOB SPH_SRC ${PROJECT_SOURCE_DIR}/src/*.cpp)
add_executable(sph ${SPH_SRC})

target_link_libraries(sph
                      ${OPENGL_gl_LIBRARY}
                      ${GLFW_LIBRARIES}
                      ${GLEW_LIBRARY}
                      ${JPEG_LIBRARIES}
                      ${LDFLAGS})
