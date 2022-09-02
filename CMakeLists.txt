cmake_minimum_required(VERSION 3.5)
project(Mini_Vio)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
   ${PROJECT_SOURCE_DIR}
   ${PROJECT_SOURCE_DIR}/include
   ${EIGEN3_INCLUDE_DIR}
   ${Pangolin_INCLUDE_DIRS}
)





set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
   src/ORBextractor.cc
   src/data.cc
   src/utils.cc
   src/tracking.cc
   src/visualize.cc
   src/poseGraph.cc
)

target_link_libraries(${PROJECT_NAME}
   ${OpenCV_LIBS}
   ${EIGEN3_LIBS}
   ${Pangolin_LIBRARIES}
   ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
   ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
   # ${EXTRA_LIBS}
)


add_executable(mono_vio_euroc
main_MonoVio_euroc.cpp)
target_link_libraries(mono_vio_euroc ${PROJECT_NAME})

# Boost ... save/load 
# message(STATUS "Compile With map save/load function")
# find_library(BOOST_SERIALIZATION boost_serialization)
# if (NOT BOOST_SERIALIZATION)
#     message(FATAL_ERROR "Can't find libboost_serialization")
# endif()
# target_link_libraries(${PROJECT_NAME} ${BOOST_SERIALIZATION})




# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Vocabulary)
# add_executable(bin_vocabulary Vocabulary/bin_vocabulary.cpp)
# target_link_libraries(bin_vocabulary ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so ${OpenCV_LIBS})