cmake_minimum_required(VERSION 3.1)

project(OptiMo CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

set(EXTERNAL_INSTALL_DIR ${CMAKE_BINARY_DIR}/external)
link_directories(${EXTERNAL_INSTALL_DIR}/lib)
include_directories(${EXTERNAL_INSTALL_DIR}/include)

# Append possible paths to Qt5 for macOS
if(APPLE AND EXISTS /usr/local/opt/qt5)
	list(APPEND CMAKE_PREFIX_PATH "/usr/local/opt/qt5")
endif()
if(APPLE AND EXISTS /usr/local/opt/qt)
	list(APPEND CMAKE_PREFIX_PATH "/usr/local/opt/qt")
endif()

######################################################################
# Build rules for the main project
######################################################################

# Dependencies
find_package(Eigen3 REQUIRED)
find_package(Qt5 COMPONENTS Widgets OpenGL Concurrent REQUIRED)
find_package(OpenGL REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Qt5Gui_INCLUDE_DIRS})
include_directories(${OPENGL_INCLUDE_DIRS})

set(THREEDIMUTIL_BUILD_DEMOS OFF CACHE INTERNAL "" FORCE)
add_subdirectory(${CMAKE_SOURCE_DIR}/external/three-dim-util)

set(PARALLELUTIL_BUILD_TESTS OFF CACHE INTERNAL "" FORCE)
add_subdirectory(${CMAKE_SOURCE_DIR}/external/parallel-util)

# Qt Widgets
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Main header files
include_directories(${CMAKE_SOURCE_DIR}/OptiMo)

# Main source files
file(GLOB files ${CMAKE_SOURCE_DIR}/OptiMo/*.cpp ${CMAKE_SOURCE_DIR}/OptiMo/*.h ${CMAKE_SOURCE_DIR}/OptiMo/*.ui)

# Executable definition for macOS
add_executable(OptiMo MACOSX_BUNDLE ${files})
set_target_properties(OptiMo PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${CMAKE_SOURCE_DIR}/resources/Info.plist.in)
target_link_libraries(OptiMo Qt5::Widgets Qt5::Concurrent Qt5::OpenGL ${OPENGL_LIBRARIES} rbdl nlopt json11 three-dim-util parallel-util)

######################################################################
# Build rules for external projects
######################################################################

include(ExternalProject)
ExternalProject_Add(
	rbdl
	SOURCE_DIR ${CMAKE_SOURCE_DIR}/external/rbdl-v2.5.0
	INSTALL_DIR ${EXTERNAL_INSTALL_DIR}
	CMAKE_ARGS 
	-DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR> 
	-DCMAKE_MACOSX_RPATH=ON 
	-DRBDL_BUILD_STATIC=ON 
	-DRBDL_BUILD_ADDON_BENCHMARK=OFF 
	-DRBDL_BUILD_ADDON_LUAMODEL=OFF 
	-DRBDL_BUILD_ADDON_URDFREADER=OFF
	-DRBDL_BUILD_PYTHON_WRAPPER=OFF
	-DRBDL_BUILD_TESTS=OFF
	-DRBDL_ENABLE_LOGGING=OFF
	-DRBDL_STORE_VERSION=OFF
	-DRBDL_USE_SIMPLE_MATH=OFF
)
ExternalProject_Add(
	nlopt
	GIT_REPOSITORY ${CMAKE_SOURCE_DIR}/external/nlopt
	GIT_TAG eafdc8ce9403a4e1532239c1bebbdb86064eda38
	INSTALL_DIR ${EXTERNAL_INSTALL_DIR}
	CMAKE_ARGS 
	-DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
	-DBUILD_SHARED_LIBS=OFF
	-DNLOPT_GUILE=OFF
	-DNLOPT_LINK_PYTHON=OFF
	-DNLOPT_MATLAB=OFF
	-DNLOPT_OCTAVE=OFF
	-DNLOPT_PYTHON=OFF
	-DNLOPT_SWIG=OFF
)
ExternalProject_Add(
	json11
	GIT_REPOSITORY ${CMAKE_SOURCE_DIR}/external/json11
	GIT_TAG ec4e45219af1d7cde3d58b49ed762376fccf1ace
	INSTALL_DIR ${EXTERNAL_INSTALL_DIR}
	CMAKE_ARGS 
	-DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
	-DJSON11_BUILD_TESTS=OFF
	-DJSON11_ENABLE_DR1467_CANARY=OFF
)
ExternalProject_Add(
	tinycolormap
	GIT_REPOSITORY ${CMAKE_SOURCE_DIR}/external/tinycolormap
	GIT_TAG v0.3.1
	INSTALL_DIR ${EXTERNAL_INSTALL_DIR}
	CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
)
ExternalProject_Add(
	nlopt-util
	GIT_REPOSITORY ${CMAKE_SOURCE_DIR}/external/nlopt-util
	GIT_TAG v0.1.0
	INSTALL_DIR ${EXTERNAL_INSTALL_DIR}
	CMAKE_ARGS -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
)
add_dependencies(OptiMo rbdl nlopt json11 tinycolormap nlopt-util)
