##---------------------------------------------------------------------------------------------------------------------
##  MICO SLAM plugin
##---------------------------------------------------------------------------------------------------------------------
##  Copyright 2020 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
##---------------------------------------------------------------------------------------------------------------------
##  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
##  and associated documentation files (the "Software"), to deal in the Software without restriction,
##  including without limitation the rights to use, copy, modify, merge, publish, distribute,
##  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
##  furnished to do so, subject to the following conditions:
##
##  The above copyright notice and this permission notice shall be included in all copies or substantial
##  portions of the Software.
##
##  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
##  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
##  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
##  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
##  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
##---------------------------------------------------------------------------------------------------------------------

cmake_minimum_required (VERSION 3.12 FATAL_ERROR)
project(mico VERSION 1.0 LANGUAGES C CXX)
set(PLUGIN_NAME ${PROJECT_NAME}-slam)

##################################################
###### Create project and configure ALIAS ########
##################################################

file(GLOB_RECURSE MICO_SLAM_SOURCE_FILES "src/*.cpp" "src/*.c")
file(GLOB_RECURSE MICO_SLAM_HEADER_FILES "include/*.h" "include/*.inl")
 
SET(MICO_MODULES ${MICO_MODULES} slam PARENT_SCOPE)
add_library(${PLUGIN_NAME} SHARED ${MICO_SLAM_HEADER_FILES} ${MICO_SLAM_SOURCE_FILES})
target_compile_features(${PLUGIN_NAME} PUBLIC cxx_std_17)
target_include_directories(${PLUGIN_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PLUGIN_NAME} PUBLIC pthread)

add_library(${PROJECT_NAME}::${PLUGIN_NAME} ALIAS ${PLUGIN_NAME})

##################################################
######    Configure project               ########
##################################################

##################################################
######    Loading 3rd party libraries.    ########
##################################################
if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release CACHE STRING
       "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
       FORCE )
endif()

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules/" ${CMAKE_MODULE_PATH})

target_compile_options(${PLUGIN_NAME} PUBLIC "-Wall")

if(NOT ${BUNDLE_COMPILATION})
  find_package(flow REQUIRED)
  if(${flow_FOUND})
    target_link_libraries(${PLUGIN_NAME}  LINK_PUBLIC flow::flow)
    target_compile_definitions(${PLUGIN_NAME} PUBLIC HAS_FLOW)
  endif()
else()
  set(flow_FOUND TRUE)
  target_link_libraries(${PLUGIN_NAME}  LINK_PUBLIC flow)
  target_compile_definitions(${PLUGIN_NAME} PUBLIC HAS_FLOW)
endif()

set(MICO_DEPS "core")
foreach(DEP ${MICO_DEPS})
  if(NOT ${BUNDLE_COMPILATION})
    find_package(mico-${DEP} REQUIRED HINTS "/usr/local/lib/cmake/mico")
  endif()
  target_link_libraries(${PLUGIN_NAME} LINK_PRIVATE mico::mico-${DEP})
endforeach()


find_package(OpenCV REQUIRED)
if(${OpenCV_FOUND})
	set_target_properties(${OpenCV_LIBS} PROPERTIES MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE)

	target_include_directories(${PLUGIN_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
	target_link_libraries(${PLUGIN_NAME} LINK_PUBLIC ${OpenCV_LIBS})

	if(${OpenCV_VERSION_MAJOR} VERSION_EQUAL "4")
		target_compile_definitions(${PLUGIN_NAME} PUBLIC "HAS_OPENCV_4")
	elseif(${OpenCV_VERSION_MAJOR} VERSION_EQUAL "3")
		target_compile_definitions(${PLUGIN_NAME} PUBLIC "HAS_OPENCV_3")
	else()
		MESSAGE(FATAL_ERROR "Not possible to compile with OpenCV 2")
	endif()
endif()

find_package(PCL QUIET REQUIRED)
if(${PCL_FOUND})
	MESSAGE( STATUS "Adding properly PCL and its dependencies")
	target_include_directories(${PLUGIN_NAME} PUBLIC ${PCL_INCLUDE_DIRS})
	target_link_libraries(${PLUGIN_NAME} LINK_PUBLIC ${PCL_LIBRARIES})
	
	target_compile_definitions(${PLUGIN_NAME} PUBLIC "ENABLE_PCL" "DISABLE_PCAP" "DISABLE_PNG" "DISABLE_LIBUSB_1_0")

	if(${PCL_VERSION} VERSION_GREATER 1.8)
		target_compile_definitions(${PLUGIN_NAME} PUBLIC "HAS_PCL_1_8")
	else()
		target_compile_definitions(${PLUGIN_NAME} PUBLIC "HAS_PCL_1_7")
	endif()
else()
	MESSAGE( FATAL_ERROR "Couldnt Add PCL and/or it's dependencies")
endif()

# target_link_libraries(${PLUGIN_NAME} LINK_PUBLIC ${dlib_LIBRARIES})
find_package(DBoW2 REQUIRED)
find_package(DLoopDetector REQUIRED)
find_package(DLib REQUIRED)
if((${DBoW2_FOUND} EQUAL "1") AND (${DLib_FOUND} EQUAL "1") AND (${DLoopDetector_FOUND} EQUAL "1"))
	target_compile_definitions(${PLUGIN_NAME} PUBLIC "USE_DBOW2")
	target_include_directories(${PLUGIN_NAME} PUBLIC ${DBoW2_INCLUDE_DIR})
	target_link_libraries(${PLUGIN_NAME} LINK_PUBLIC ${DBoW2_LIBRARY})
	target_include_directories(${PLUGIN_NAME} PUBLIC ${DLoopDetector_INCLUDE_DIR})
	#target_link_libraries(${PLUGIN_NAME} LINK_PUBLIC ${DLoopDetector_LIBRARY})
	target_include_directories(${PLUGIN_NAME} PUBLIC /usr/local/include/DUtils)	## WHY is it not working with variable?...
	target_include_directories(${PLUGIN_NAME} PUBLIC /usr/local/include/DVision)
	target_include_directories(${PLUGIN_NAME} PUBLIC /usr/local/include/DUtilsCV) # ${DLib_INCLUDE_DIR})
	target_link_libraries(${PLUGIN_NAME} LINK_PUBLIC ${DLib_LIBRARY})
else()
	message(FATAL_ERROR "DBoW2 not found, disabling SLAM module")
endif()

set(G2O_ROOT /usr/local)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(g2o HINTS "/usr/local/lib/cmake/g2o")
if(${g2o_FOUND})
	target_compile_definitions(${PLUGIN_NAME} PUBLIC "USE_G2O")	
	target_include_directories(${PLUGIN_NAME}  PUBLIC ${CHOLMOD_INCLUDE_DIR})
	
	target_link_libraries(${PLUGIN_NAME} LINK_PUBLIC lapack)
	target_link_libraries(${PLUGIN_NAME}  LINK_PUBLIC g2o::core g2o::types_sba g2o::solver_cholmod)
else()
	message(FATAL_ERROR "G2O not found, disabling SLAM module")
endif()

##################################################
######    Library.                        ########
##################################################
#add_definitions(-fopenmp)

##################################################
######    Documentation.                  ########
##################################################
find_package(Doxygen)
if(DOXYGEN_FOUND)
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
	add_custom_target(	doc_mico-slam
						${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
						WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
						COMMENT "Generating API documentation with Doxygen" VERBATIM )
endif(DOXYGEN_FOUND)


##################################################
######    Installation.                   ########
##################################################
include(CMakePackageConfigHelpers)



install(TARGETS ${PLUGIN_NAME} EXPORT ${PLUGIN_NAME}-targets
	COMPONENT ${PLUGIN_NAME}
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION bin
	INCLUDES DESTINATION include
)

install(EXPORT ${PLUGIN_NAME}-targets
	FILE "${PLUGIN_NAME}-targets.cmake"
	NAMESPACE ${PROJECT_NAME}::
	DESTINATION lib/cmake/${PROJECT_NAME}
	COMPONENT ${PLUGIN_NAME}
)

configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/templates/${PLUGIN_NAME}-config.cmake.in"
	"${CMAKE_CURRENT_BINARY_DIR}/${PLUGIN_NAME}-config.cmake"
	@ONLY
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${PLUGIN_NAME}-config-version.cmake"
    VERSION ${version}
    COMPATIBILITY AnyNewerVersion
)

install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/${PLUGIN_NAME}-config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/${PLUGIN_NAME}-config-version.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME}
    COMPONENT ${PLUGIN_NAME}
)

# DESTINATION will be automatically prefixed by ${CMAKE_INSTALL_PREFIX}
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
    COMPONENT ${PLUGIN_NAME}
    DESTINATION ${CMAKE_INSTALL_PREFIX}
)

# uninstall target 
if(NOT TARGET uninstall) 
    configure_file( 
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/templates/cmake_uninstall.cmake.in" 
        "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" 
        @ONLY) 
 
    add_custom_target(uninstall COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake) 
endif() 

# FLOW INSTALL target 
add_custom_target(flow_install_${PLUGIN_NAME} ALL
                COMMAND ${CMAKE_COMMAND} -E copy 	${CMAKE_CURRENT_BINARY_DIR}/lib${PLUGIN_NAME}.so 
                ${CMAKE_BINARY_DIR}/plugins/lib${PLUGIN_NAME}.so
)
add_dependencies(flow_install_${PLUGIN_NAME} ${PLUGIN_NAME})


##################################################
####  CPACK deb generation configuration    ######
##################################################
if(${PREPARE_PACKAGE_PPA})
  set(UPLOAD_PPA "ppa:bardo91/mico")
  set(CPACK_GENERATOR "DEB")
  set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Pablo Ramon Soria <pabramsor@gmail.com>") #required
  set(CPACK_DEBIAN_PACKAGE_NAME ${PLUGIN_NAME})
  
  include(CPack)
  include(cmake/templates/prepare_package_ppa.cmake)
endif()