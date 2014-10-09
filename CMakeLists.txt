project(CPD)
cmake_minimum_required(VERSION 2.8)

set( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )
if( NOT EIGEN3_INCLUDE_DIR )
    message( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation." )
endif()
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

set(inc_base include/Base)
set(inc_core include/Core)
set(src_base src/Base)
set(src_core src/Core)

set(incs_base 	${inc_base}/vector.h
				${inc_base}/matrix.h
				${inc_base}/parameters.h
				${inc_base}/data.h)

set(incs_core	${inc_core}/em.h
				${inc_core}/registrator.h
				${inc_core}/cpd_base.h
				${inc_core}/cpd_rigid.h
				${inc_core}/cpd_affine.h
				${inc_core}/cpd_nonrigid.h)
		 
set(srcs_base 	${src_base}/vector.cpp
				${src_base}/data.cpp)

set(srcs_core	${src_core}/em.cpp
				${src_core}/registrator.cpp
				${src_core}/cpd_base.cpp
				${src_core}/cpd_rigid.cpp
				${src_core}/cpd_affine.cpp
				${src_core}/cpd_nonrigid.cpp)
				
set(incs 		${incs_base} ${incs_core})
set(srcs		${srcs_base} ${srcs_core})


#Organize the files
set(Headers 	"Header Files")
set(Sources		"Source Files")

set(headfiles_base		${Headers}\\Base)
set(headfiles_core		${Headers}\\Core)
set(srcfiles_base		${Sources}\\Base)
set(srcfiles_core		${Sources}\\Core)

source_group(${headfiles_base} FILES ${incs_base})
source_group(${headfiles_core} FILES ${incs_core})

source_group(${srcfiles_base} FILES ${srcs_base})
source_group(${srcfiles_core} FILES ${srcs_core})

add_executable(CPD ${srcs} ${incs})