set(LIB srm_coord)
message("Configuring coord module...")

find_package(OpenCV 4 REQUIRED QUIET)
if(NOT ${OpenCV_FOUND})
  message(FATAL_ERROR "OpenCV not found.")
endif()

find_package(Glog REQUIRED QUIET)
if(NOT ${Glog_FOUND})
  message(FATAL_ERROR "Glog not found.")
endif()

find_package(Eigen3 REQUIRED)
if(NOT ${Eigen3_FOUND})
  message(FATAL_ERROR "Eigen3 not found.")
endif()

file(GLOB SRC src/*.cpp)
add_library(${LIB} SHARED ${SRC})

target_include_directories(
  ${LIB}
  PUBLIC ${OpenCV_INCLUDE_DIRS}
  PUBLIC ${GLOG_INCLUDE_DIRS}
  PUBLIC ${EIGEN3_INCLUDE_DIR})

target_link_libraries(${LIB} ${OpenCV_LIBS} glog::glog)
