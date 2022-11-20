cmake_minimum_required(VERSION 3.5)
 
# 设置VCPKG_ROOT所在的位置
#set(VCPKG_ROOT "D:/GitHub/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE PATH "")
#set(CMAKE_TOOLCHAIN_FILE ${VCPKG_ROOT})

#message(${CMAKE_TOOLCHAIN_FILE})

project(GrorReg)

set(PCL_DIR "D:/ThirdLib/PCL 1.8.1/cmake")
#set(PCL_DIR "C:/Program Files/PCL 1.12.0/cmake")

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS} -D_EXPORTING; -D_CRT_SECURE_NO_WARNINGS;  -DPCL_NO_PRECOMPILE)
link_directories(${PCL_LIBRARIES})

FILE (GLOB ALL_SOURCES "${PROJECT_SOURCE_DIR}/include/*.h" "${PROJECT_SOURCE_DIR}/src/*.cpp" )
include_directories( ${PROJECT_SOURCE_DIR}/include)
# exe
add_executable(GrorReg ${ALL_SOURCES})
target_link_libraries(GrorReg  ${PCL_LIBRARIES})
