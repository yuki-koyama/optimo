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
find_package(Qt5 COMPONENTS Concurrent Gui Widgets REQUIRED)

set(JSON11_BUILD_TESTS          OFF CACHE INTERNAL "" FORCE)
set(JSON11_ENABLE_DR1467_CANARY OFF CACHE INTERNAL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/external/json11)

set(THREEDIMUTIL_BUILD_DEMOS OFF CACHE INTERNAL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/external/three-dim-util)

set(PARALLELUTIL_BUILD_TESTS OFF CACHE INTERNAL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/external/parallel-util)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/external/nlopt-util)

set(TINYCOLORMAP_BUILD_TOOLS OFF CACHE INTERNAL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/external/tinycolormap)

# Qt Widgets
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Main header files
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/OptiMo)

# Main source files
file(GLOB files ${CMAKE_CURRENT_SOURCE_DIR}/OptiMo/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/OptiMo/*.h ${CMAKE_CURRENT_SOURCE_DIR}/OptiMo/*.ui)

# Executable definition for macOS
add_executable(OptiMo MACOSX_BUNDLE ${files})
set_target_properties(OptiMo PROPERTIES MACOSX_BUNDLE_INFO_PLIST ${CMAKE_CURRENT_SOURCE_DIR}/resources/Info.plist.in)
target_link_libraries(OptiMo Qt5::Concurrent Qt5::Gui Qt5::Widgets rbdl nlopt json11 three-dim-util parallel-util Eigen3::Eigen nlopt-util tinycolormap)

######################################################################
# Build rules for external projects
######################################################################

include(ExternalProject)
ExternalProject_Add(
	rbdl
	SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/rbdl-v2.5.0
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
	GIT_REPOSITORY ${CMAKE_CURRENT_SOURCE_DIR}/external/nlopt
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
add_dependencies(OptiMo rbdl nlopt)
