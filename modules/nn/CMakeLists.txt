set(LIB srm_nn)
message("Configuring nn module...")

# 设置目录位置
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
  set(CUDA_PATH /usr/local/cuda)  # Linux: CUDA 目录
  if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(TensorRT_PATH /opt/TensorRT) # Linux x86-64: TensorRT 目录
  elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(TensorRT_PATH /usr/src/tensorrt)
    set(TensorRT_LIB_PATH /usr/lib/aarch64-linux-gnu)
  endif()
endif()


# 查找软件包
find_package(OpenCV 4 REQUIRED QUIET)
if(NOT ${OpenCV_FOUND})
  message(FATAL_ERROR "OpenCV not found.")
endif()

find_package(Glog REQUIRED QUIET)
if(NOT ${Glog_FOUND})
  message(FATAL_ERROR "Glog not found.")
endif()

if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
  find_package(CUDA REQUIRED)
  find_package(TensorRT REQUIRED)
  file(GLOB SRC src/tensorrt/*.cpp src/yolo.cpp)
endif()


# 添加源文件
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  file(GLOB SRC src/coreml/*.mm src/yolo.cpp)
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
  file(GLOB SRC src/tensorrt/*.cpp src/yolo.cpp)
endif()

# 设置库的路径
add_library(${LIB} SHARED ${SRC})
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  target_compile_options(${LIB} PUBLIC -x objective-c++ -fobjc-arc)
  set(CoreML_LIBS "-framework AppKit -framework CoreML -framework Foundation -framework Vision -framework CoreVideo")
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
  file(GLOB CUDA_LIBS ${CUDA_PATH}/lib64/libcu*.so)
  if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    target_link_directories(${LIB} PUBLIC ${TensorRT_PATH}/lib)
    file(GLOB TensorRT_LIBS ${TensorRT_PATH}/lib/libnv*.so)
  elseif(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_link_directories(${LIB} PUBLIC ${TensorRT_LIB_PATH})
    file(GLOB TensorRT_LIBS ${TensorRT_LIB_PATH}/libnv*.so)
  endif()
endif()

# 添加头文件
target_include_directories(
  ${LIB}
  PUBLIC ${OpenCV_INCLUDE_DIRS}
  PUBLIC ${GLOG_INCLUDE_DIRS}
  PUBLIC ${EIGEN3_INCLUDE_DIR}
)

if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
  target_include_directories(
    ${LIB}
    PUBLIC ${TensorRT_INCLUDE_DIRS}
    PUBLIC ${CUDAToolkit_INCLUDE_DIRS}
  )
endif()

  target_link_directories(
    ${LIB}
    PUBLIC ${CoreML_LIBS}
  )

# 设置链接库
target_link_libraries(
  ${LIB}
  ${OpenCV_LIBS}
  glog::glog
)

if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  target_link_libraries(
    ${LIB}
    ${CoreML_LIBS}
  )
elseif(CMAKE_SYSTEM_NAME MATCHES "Linux")
  target_link_libraries(
    ${LIB}
    ${CUDA_LIBS}
    ${TensorRT_LIBS}
  )
endif()
