cmake_minimum_required(VERSION 2.8.3)
project(libicp)

# check c++11 / c++0x(https://svn.boost.org/trac/boost/ticket/9240)
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
 set(CMAKE_CXX_FLAGS "-std=c++11  -fext-numeric-literals")
 #'-fext-numeric-literals' : https://svn.boost.org/trac/boost/ticket/9240
elseif(COMPILER_SUPPORTS_CXX0X)
 set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
 message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

# include directory
include_directories(
        include
        ${Boost_INCLUDE_DIRS}
)

# determine if OpenMP can/should be used
option(USE_OPENMP "Enable OpenMP?" ON) # set to OFF to disable
if(USE_OPENMP)
 FIND_PACKAGE(OpenMP)
 if(OPENMP_FOUND OR OpenMP_FOUND)
  message(STATUS "OpenMP flags = ${OpenMP_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
 else(OPENMP_FOUND OR OpenMP_FOUND)
  message(STATUS "OpenMP not supported")
 endif(OPENMP_FOUND OR OpenMP_FOUND)
endif(USE_OPENMP)

add_library(icplib STATIC src/icp.cpp src/icpPointToPlane.cpp src/icpPointToPoint.cpp src/kdtree.cpp src/matrix.cpp)

add_executable(demo src/demo.cpp)
target_link_libraries(demo icplib)
install(
	TARGETS demo
	DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
install(
	TARGETS icplib
	DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
install(DIRECTORY 
	include/libicp
	DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
