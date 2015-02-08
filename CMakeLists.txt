project(hsd2tms)
set(VERSION 1.0.0)
cmake_minimum_required(VERSION 2.8)

include(FindPkgConfig)
include(GNUInstallDirs)
include(InstallRequiredSystemLibraries)

# cairo
pkg_check_modules(LIBPNG libpng)
INCLUDE_DIRECTORIES(${LIBPNG_INCLUDE_DIRS})
LINK_DIRECTORIES(${LIBPNG_LIBRARY_DIRS})
LINK_LIBRARIES(${LIBPNG_LIBRARIES})

set(CMAKE_C_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "-Wall -std=c++11")

ADD_CUSTOM_TARGET(debug
                    ${CMAKE_COMMAND} 
                      -DCMAKE_BUILD_TYPE:STRING=Debug ${PROJECT_SOURCE_DIR})
ADD_CUSTOM_TARGET(release
                    ${CMAKE_COMMAND}
                      -DCMAKE_BUILD_TYPE:STRING=Release ${PROJECT_SOURCE_DIR})

set(CMAKE_C_FLAGS_DEBUG "-O0 -g -DDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g -DDEBUG")
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

# Make sure to link to Harfbuzz's since it may be installed locally atm.
option(USE_LOCAL_HARFBUZZ "Use local harfbuzz instead of distro's" ON)
if(USE_LOCAL_HARFBUZZ)
  set(CMAKE_INSTALL_RPATH "${HARFBUZZ_LIBRARY_DIRS}")
  set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
endif(USE_LOCAL_HARFBUZZ)

configure_file("${PROJECT_SOURCE_DIR}/src/test.sh.in"
               test.sh)

ADD_CUSTOM_TARGET(test /bin/bash test.sh)


add_executable(${PROJECT_NAME}
               src/main.cpp src/hsd2tms.cpp)

