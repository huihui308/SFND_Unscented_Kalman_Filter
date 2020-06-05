cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set( prjName ukf_highway )

add_definitions(-std=c++11)


###### change the default location of executables
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)


set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(playback)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


set(
    giPrjSrc
    src/ukf.cc
    src/main.cc
    src/tools.cc
    src/highway.cc
    src/render/render.cc
)

set(
    giPrjLib
    libglog.a
    libgflags.a
    ${PCL_LIBRARIES}
)


add_executable( ${prjName} ${giPrjSrc} )
target_link_libraries( ${prjName} ${giPrjLib} )


