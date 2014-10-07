project(CPD)
cmake_minimum_required(VERSION 2.8)

set( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )
if( NOT EIGEN3_INCLUDE_DIR )
    message( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation." )
endif()
include_directories(EIGEN3_INCLUDE_DIR)

set(inc_base include/Base)
set(src_base src/Base)

set(incs_base 	${inc_base}/vector.h
				${inc_base}/vector2.h
				${inc_base}/vector3.h)
		 
set(srcs_base 	${src_base}/vector.cpp
				${src_base}/vector2.cpp
				${src_base}/vector3.cpp)
				
set(incs 		${incs_base})
set(srcs		${srcs_base})


#Organize the files
set(Headers 	"Header Files")
set(Sources		"Source Files")

set(headfiles_base		${Headers}\\Base)
set(srcfiles_base		${Sources}\\Base)

source_group(${headfiles_base} FILES ${incs_base})
source_group(${srcfiles_base} FILES ${srcs_base})

add_executable(CPD ${srcs} ${incs})