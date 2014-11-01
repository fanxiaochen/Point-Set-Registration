project(CPD)
cmake_minimum_required(VERSION 2.8)

set( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )
if( NOT EIGEN3_INCLUDE_DIR )
    message( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation." )
endif()

find_package(OpenSceneGraph REQUIRED osgViewer osgGA)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/cpd)

set(src_base cpd/base)
set(src_core cpd/core)
set(src_fast cpd/fast)
set(src_disp cpd/disp)
set(src_3rdparty cpd/3rdparty/figtree)
		 
set(srcs_base	${src_base}/matrix.hpp
				${src_base}/parameters.hpp
				${src_base}/data.hpp)

set(srcs_core	${src_core}/registrator.hpp
				${src_core}/cpd_base.hpp
				${src_core}/cpd_rigid.hpp
				${src_core}/cpd_affine.hpp
				${src_core}/cpd_nonrigid.hpp)

set(srcs_fast		${src_fast}/fgt_wrapper.hpp
					${src_fast}/low_rank.hpp)

set(srcs_disp		${src_disp}/render_thread.hpp)

set(srcs_3rdparty	${src_3rdparty}/figtree.h
					${src_3rdparty}/figtree_internal.h
					${src_3rdparty}/kCenterClustering.h
					${src_3rdparty}/figtree.cpp
					${src_3rdparty}/kCenterClustering.cpp)
				
set(srcs_test	cpd/test.cpp)

set(srcs		${srcs_base} ${srcs_core} ${srcs_fast} ${srcs_disp} ${srcs_3rdparty} ${srcs_test})


#Organize the files
set(Sources		"Source Files")

set(srcfiles_base		${Sources}\\base)
set(srcfiles_core		${Sources}\\core)
set(srcfiles_fast		${Sources}\\fast)
set(srcfiles_disp		${Sources}\\disp)

set(srcfiles_3rdparty	${Sources}\\3rdparty\\figtree)
set(srcfiles_test		${Sources})

source_group(${srcfiles_base} FILES ${srcs_base})
source_group(${srcfiles_core} FILES ${srcs_core})
source_group(${srcfiles_fast} FILES ${srcs_fast})
source_group(${srcfiles_disp} FILES ${srcs_disp})
source_group(${srcfiles_3rdparty} FILES ${srcs_3rdparty})
source_group(${srcfiles_test} FILES ${srcs_test})

add_executable(CPD ${srcs})
target_link_libraries(CPD ${OPENSCENEGRAPH_LIBRARIES})