set(LIB srm_common)
message("Configuring common module...")

find_package(Ceres QUIET)
if(NOT ${CERES_FOUND})
  message(FATAL_ERROR "Ceres not found.")
endif()

add_library(${LIB} INTERFACE)
target_include_directories(${LIB} INTERFACE ${CERES_INCLUDE_DIRS})
target_link_libraries(${LIB} INTERFACE ${CERES_LIBRARIES})
