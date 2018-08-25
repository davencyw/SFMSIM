cmake_minimum_required(VERSION 2.8.3)
project(sfmsimulation)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")
EXECUTE_PROCESS( COMMAND gcc -dumpversion GCC_VERSION )
EXECUTE_PROCESS( COMMAND g++ -dumpversion GCC_VERSION )

##___________________________
##
##BUILD TYPE
##

# SET default build type to release for single generators
if (NOT CMAKE_BUILD_TYPE)
	SET(CMAKE_BUILD_TYPE "RELEASE" FORCE)
endif ()

SET (CMAKE_BUILD_TYPE "RELEASE")

message("mode")
message("${CMAKE_BUILD_TYPE}")
message("___________________")
message("___________________")
message("___________________")

##___________________________
##
##BENCHMARKING
##

##___________________________
##
##FLAGS
##

SET (RELEASE_FLAGS "-O3 -march=native  -mtune=native -mavx2 -mavx -msse -ftree-vectorize")
## -fopt-info-vec-optimized

SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -g")

SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -no-pie -std=c++17 -pthread  -fopenmp -lopencv_core -lopencv_imgproc -lopencv_highgui")

##___________________________
##
##LIBRARIES
##

#set(GTEST_ROOT "/usr/local/lib/gtest/")
#find_package(GTest REQUIRED)
#include_directories(${GTEST_INCLUDE_DIRS})
#enable_testing()

find_package(Ceres REQUIRED)
find_package( OpenCV REQUIRED )
find_library( OpenCV_SFM libopencv_sfm.so REQUIRED)
find_package( Eigen3 REQUIRED )
include_directories (${EIGEN3_INCLUDE_DIR})
find_package(Boost COMPONENTS program_options REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

##___________________________
##
##FILES & FOLDERS
##

include_directories(include)
#include_directories(ext)

SET (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
SET (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/dll)
SET (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

file(GLOB FRAMESIMSOURCES
		src/framesimulator/*.cc
)

file(GLOB SFMSIMULATORSOURCES
		src/sfmsimulator/*.cc
		src/pointclassifier/pointclassifier.cc
)

file(GLOB VISUALIZERSOURCES
		src/visualizer/*.cc
)

file(GLOB MAINSOURCES
  src/main.cc
)

file (GLOB TESTSOURCES
  tests/test_main.cc
  tests/testing_geometry.cc
)


##___________________________
##
##LIBRARIES & EXECUTABLES
##

add_library(visualizer ${VISUALIZERSOURCES})
add_library(sfmsimulator ${SFMSIMULATORSOURCES})
add_library(framesim ${FRAMESIMSOURCES})

target_link_libraries(visualizer ${OpenCV_LIBS})
target_link_libraries(sfmsimulator ${OpenCV_LIBS} ${OpenCV_SFM} glog)
target_link_libraries(framesim ${OpenCV_LIBS})

add_executable (${PROJECT_NAME}  ${MAINSOURCES})
target_link_libraries(${PROJECT_NAME} visualizer sfmsimulator framesim)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
