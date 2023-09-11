set(LIB srm_common)
message("Configuring common module...")

find_package(OpenCV 4 REQUIRED QUIET)
if(NOT ${OpenCV_FOUND})
  message(FATAL_ERROR "OpenCV not found.")
endif()

find_package(Glog REQUIRED QUIET)
if(NOT ${Glog_FOUND})
  message(FATAL_ERROR "Glog not found.")
endif()

find_package(yaml-cpp REQUIRED)
if(NOT ${YAML_CPP_FOUND})
  message(FATAL_ERROR "yaml-cpp not found.")
endif()

file(GLOB SRC src/*.cpp)
add_library(${LIB} ${SRC})

target_compile_options(
  ${LIB}
  PUBLIC -fPIC
)

target_include_directories(
  ${LIB}
  PUBLIC ${OpenCV_INCLUDE_DIRS}
  PUBLIC ${GLOG_INCLUDE_DIRS}
  PUBLIC ${YAML_CPP_INCLUDE_DIR}
)

target_link_directories(
  ${LIB}
  PUBLIC ${YAML_CPP_LIBRARY_DIR}
)

target_link_libraries(
  ${LIB}
  ${OpenCV_LIBS}
  glog::glog
  ${YAML_CPP_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT}
)
