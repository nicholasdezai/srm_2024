file(GLOB SRC src/*.cpp)
add_library(srm_video SHARED ${SRC})

if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  set(MVS_PATH /Library/MVS_SDK) # MacOS: 海康相机驱动目录
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
  set(MVS_PATH /opt/MVS) # Linux: 大恒相机驱动目录
  set(Galaxy_PATH /opt/Galaxy_camera) # Linux: 海康相机驱动目录
endif()

if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  target_include_directories(srm_video PUBLIC ${MVS_PATH}/Includes)
  file(GLOB MVS_LIBS ${MVS_PATH}/lib/*.dylib)
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
target_include_directories(srm_video PUBLIC ${MVS_PATH}/include PUBLIC ${Galaxy_PATH}/inc)
  if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    file(GLOB MVS_LIBS ${MVS_PATH}/lib/64/*.so)
    file(GLOB Galaxy_LIBS ${Galaxy_PATH}/lib/x86_64/*.so)
  elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    file(GLOB MVS_LIBS ${MVS_PATH}/lib/aarch64/*.so)
    file(GLOB Galaxy_LIBS ${Galaxy_PATH}/lib/armv8/*.so)
  endif()
endif()

target_link_libraries(
  srm_video
  ${OpenCV_LIBS}
  glog::glog
  ${MVS_LIBS}
  ${CMAKE_THREAD_LIBS_INIT}
)