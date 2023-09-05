set(LIB srm_core)
message("Configuring coord module...")

find_package(OpenCV 4 REQUIRED QUIET)
if(NOT ${OpenCV_FOUND})
  message(FATAL_ERROR "OpenCV not found.")
endif()

find_package(Glog REQUIRED QUIET)
if(NOT ${Glog_FOUND})
  message(FATAL_ERROR "Glog not found.")
endif()

file(GLOB SRC src/*.cpp)
add_library(${LIB} SHARED ${SRC})

target_include_directories(
  ${LIB}
  PUBLIC ${OpenCV_INCLUDE_DIRS}
  PUBLIC ${GLOG_INCLUDE_DIRS}
  PUBLIC ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(
  ${LIB}
  ${OpenCV_LIBS}
  glog::glog
  srm_common
  srm_video
  srm_coord 
  srm_robot
  srm_nn
)
