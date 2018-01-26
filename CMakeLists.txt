cmake_minimum_required(VERSION 3.6 FATAL_ERROR)

project(es C CXX)

set(${CMAKE_PROJECT_NAME}_VERSION_MAJOR        0)
set(${CMAKE_PROJECT_NAME}_VERSION_MINOR        0)
set(${CMAKE_PROJECT_NAME}_VERSION_PATCH        0)
set(PROJECT_VERSION
  "${${CMAKE_PROJECT_NAME}_VERSION_MAJOR}.\
${${CMAKE_PROJECT_NAME}_VERSION_MINOR}.\
${${CMAKE_PROJECT_NAME}_VERSION_PATCH}")

set(PROJECT_PATH "${CMAKE_CURRENT_SOURCE_DIR}")
set(DOXYGEN_PATH "${PROJECT_PATH}/doxydocs")

option(BUILD_SHARED_LIBS "Build shared libraries." OFF)
option(BUILD_DOXYGEN_DOCS "Build docs"             OFF)

configure_file(${PROJECT_PATH}/core/include/es/core/version.h.in
  ${PROJECT_PATH}/core/include/es/core/version.h @ONLY IMMEDIATE)

find_package(Eigen3 REQUIRED)

if(BUILD_DOXYGEN_DOCS)
  find_package(Doxygen)

  if(NOT DOXYGEN_FOUND)
    message(FATAL_ERROR
      "Doxygen is needed to build the documentation.")
  endif()

  configure_file(${PROJECT_PATH}/Doxyfile.in
                 ${PROJECT_PATH}/Doxyfile @ONLY IMMEDIATE)

  add_custom_target(doxygen_docs ALL COMMAND ${DOXYGEN_EXECUTABLE}
                    ${PROJECT_PATH}/Doxyfile
                    SOURCES ${PROJECT_PATH}/Doxyfile)
  install(DIRECTORY ${PROJECT_PATH}/doxydocs DESTINATION share/doc)
endif()

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang") # using Clang
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic -std=c++14")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")  # using GCC
  set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -std=c++14")
endif()

add_subdirectory(core)
add_subdirectory(rayes)
add_subdirectory(coco)

