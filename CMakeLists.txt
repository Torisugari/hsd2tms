project(hsd2tms)
set(VERSION 1.0.0)
cmake_minimum_required(VERSION 2.8)

include(FindPkgConfig)
include(GNUInstallDirs)
include(InstallRequiredSystemLibraries)

# libpng
pkg_check_modules(LIBPNG libpng)
INCLUDE_DIRECTORIES(${LIBPNG_INCLUDE_DIRS})
LINK_DIRECTORIES(${LIBPNG_LIBRARY_DIRS})
LINK_LIBRARIES(${LIBPNG_LIBRARIES})

add_definitions(
  -DCTOP_LUT_MT2_V20_PATH=\"${CMAKE_SOURCE_DIR}/data/LUT/CTOP_LUT.mt2.v20.grd\"
)

add_definitions(
  -DHIMAWARI_TEST_DATA_DIR_PATH=\"${CMAKE_SOURCE_DIR}/test/HS_H08_20150125_0230_JP02/\"
)

set(CMAKE_C_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "-Wall -std=c++11")

ADD_CUSTOM_TARGET(debug
                    ${CMAKE_COMMAND} 
                      -DCMAKE_BUILD_TYPE:STRING=Debug ${PROJECT_SOURCE_DIR})
ADD_CUSTOM_TARGET(release
                    ${CMAKE_COMMAND}
                      -DCMAKE_BUILD_TYPE:STRING=Release ${PROJECT_SOURCE_DIR})

set(CMAKE_C_FLAGS_DEBUG "-O0 -g -DDEBUG -pg")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -DDEBUG -pg")
set(CMAKE_C_FLAGS_RELEASE "-O2 -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")

# Settings for clang's static-analyzer.
# Just type "make scan" in advance to "scan-build -V make".
ADD_CUSTOM_TARGET(scan
                    ${CMAKE_COMMAND}
                      -DCMAKE_BUILD_TYPE:STRING=Scan ${PROJECT_SOURCE_DIR})

if (CMAKE_BUILD_TYPE MATCHES "Scan")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2 -DNDEBUG") 
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -DNDEBUG") 

  # Why do I have to set full-pathes? No pkg-config file?
  set(CMAKE_C_COMPILER "/usr/share/clang/scan-build/ccc-analyzer")
  set(CMAKE_CXX_COMPILER "/usr/share/clang/scan-build/c++-analyzer")
endif(CMAKE_BUILD_TYPE MATCHES "Scan")

configure_file("${PROJECT_SOURCE_DIR}/src/test.sh.in"
               test.sh)

add_library(${PROJECT_NAME}util STATIC src/hsd2tms.cpp)
add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}util)

option(USE_MAKE_CHECK "Enables 'make check' command." ON)
if (USE_MAKE_CHECK)
  pkg_check_modules(CPPUTEST cpputest)
  INCLUDE_DIRECTORIES(${CPPUTEST_INCLUDE_DIRS})
  LINK_DIRECTORIES(${CPPUTEST_LIBRARY_DIRS})
  LINK_LIBRARIES(${CPPUTEST_LIBRARIES})

  add_executable(utest src/utestmain.cpp)
  target_link_libraries(utest ${PROJECT_NAME}util)
  target_link_libraries(utest CppUTest)
  add_custom_target(check ${PROJECT_BINARY_DIR}/utest -v)
endif ()
