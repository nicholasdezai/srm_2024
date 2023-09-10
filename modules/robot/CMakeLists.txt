set(LIB srm_robot)
message("Configuring coord module...")

find_package(Glog REQUIRED QUIET)
if(NOT ${Glog_FOUND})
  message(FATAL_ERROR "Glog not found.")
endif()

file(GLOB SRC src/*.cpp)
add_library(${LIB} SHARED ${SRC})

target_include_directories(
  ${LIB}
  PUBLIC ${GLOG_INCLUDE_DIRS}
)

target_link_libraries(
  ${LIB} 
  glog::glog
  srm_common
)
