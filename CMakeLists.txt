project(CPD)
cmake_minimum_required(VERSION 2.8)

set( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )
if( NOT EIGEN3_INCLUDE_DIR )
    message( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation." )
endif()
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/cpd)

set(src_base cpd/Base)
set(src_core cpd/Core)
		 
set(srcs_base 	${src_base}/vector.hpp
				${src_base}/matrix.hpp
				${src_base}/parameters.hpp
				${src_base}/data.hpp)

set(srcs_core	${src_core}/em.hpp
				${src_core}/registrator.hpp
				${src_core}/cpd_base.hpp
				${src_core}/cpd_rigid.hpp
				${src_core}/cpd_affine.hpp
				${src_core}/cpd_nonrigid.hpp)
				
set(srcs_test	cpd/test.cpp)

set(srcs		${srcs_base} ${srcs_core} ${srcs_test})


#Organize the files
set(Sources		"Source Files")

set(srcfiles_base		${Sources}\\Base)
set(srcfiles_core		${Sources}\\Core)
set(srcfiles_test		${Sources})

source_group(${srcfiles_base} FILES ${srcs_base})
source_group(${srcfiles_core} FILES ${srcs_core})
source_group(${srcfiles_test} FILES ${srcs_test})

add_executable(CPD ${srcs})