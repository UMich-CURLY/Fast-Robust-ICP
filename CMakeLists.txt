cmake_minimum_required(VERSION 2.8)
project(FRICP)

#--- CMake configuration
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

#--- Compiler configuration
set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

#--- OpenMP (cannot work in clang)
find_package(OpenMP QUIET)
if(OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
endif()

#--- Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

#--- Nanoflann
find_package(NanoFlann REQUIRED)
include_directories(${NANOFLANN_INCLUDE_DIR})

find_package(UnifiedCvo REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

#--- Build example
include_directories(.)
file(GLOB SOURCES "main.cpp")
file(GLOB HEADERS "*.h")
add_executable(FRICP ${SOURCES} ${HEADERS})

add_executable(kitti_test kitti.cpp)
target_include_directories(kitti_test PUBLIC ${HEADERS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(kitti_test UnifiedCvo::kitti UnifiedCvo::cvo_utils_lib UnifiedCvo::cvo_gpu_img_lib ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(tartan_test tartan.cpp )
target_include_directories(tartan_test PUBLIC ${HEADERS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(tartan_test UnifiedCvo::tartan UnifiedCvo::cvo_gpu_img_lib UnifiedCvo::cvo_utils_lib  ${PCL_LIBRARIES} ${OpenCV_LIBS})


add_executable(tum_test tum.cpp )
target_include_directories(tum_test PUBLIC ${HEADERS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(tum_test UnifiedCvo::tum UnifiedCvo::cvo_gpu_img_lib UnifiedCvo::cvo_utils_lib  ${PCL_LIBRARIES} ${OpenCV_LIBS})



if(APPLE OR UNIX)
    #--- Deploy data folder link
    execute_process(COMMAND ln -f -s ${CMAKE_SOURCE_DIR}/data WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
    #--- "Dummy" target to have data appear in QtCreator
    add_custom_target(data SOURCES ${CMAKE_SOURCE_DIR}/data)
else()
    file(COPY ${CMAKE_SOURCE_DIR}/data DESTINATION ${PROJECT_BINARY_DIR})
endif()
