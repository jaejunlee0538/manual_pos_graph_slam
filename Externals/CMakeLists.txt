cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install)

add_definitions(-DG2O_DEBUG_FACTORY)
add_subdirectory(g2o)
add_subdirectory(libicp)
