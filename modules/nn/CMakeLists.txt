set(LIB srm_nn)
message("Configuring nn module...")

find_package(OpenCV 4 REQUIRED QUIET)
if(NOT ${OpenCV_FOUND})
  message(FATAL_ERROR "OpenCV not found.")
endif()

find_package(Glog REQUIRED QUIET)
if(NOT ${Glog_FOUND})
  message(FATAL_ERROR "Glog not found.")
endif()

if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  file(GLOB SRC src/coreml/*.mm src/yolo.cpp)
  set(CoreML_LIBS "-framework AppKit -framework CoreML -framework Foundation -framework Vision -framework CoreVideo")
endif()

add_library(${LIB} SHARED ${SRC})
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  target_compile_options(${LIB} PUBLIC -x objective-c++ -fobjc-arc)
endif()

target_include_directories(
  ${LIB}
  PUBLIC ${OpenCV_INCLUDE_DIRS}
  PUBLIC ${GLOG_INCLUDE_DIRS}
  PUBLIC ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(
  ${LIB}
  ${OpenCV_LIBS}
  ${CoreML_LIBS}
  glog::glog
  srm_common
)
