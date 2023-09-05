find_package(CUDA REQUIRED)

find_path(TensorRT_INCLUDE_DIRS NvInfer.h
        HINTS ${TensorRT_PATH} ${CUDA_TOOLKIT_ROOT_DIR}
        PATH_SUFFIXES include)

set(TensorRT_INCLUDE_DIRS
        ${TensorRT_INCLUDE_DIRS} ${TensorRT_PATH}/samples/common ${CUDA_INCLUDE_DIRS})

find_library(TensorRT_LIBRARY_INFER nvinfer
        HINTS ${TensorRT_PATH} ${CUDA_TOOLKIT_ROOT_DIR}
        PATH_SUFFIXES lib lib64 lib/x64)

find_library(TensorRT_LIBRARY_INFER_PLUGIN nvinfer_plugin
        HINTS ${TensorRT_PATH} ${CUDA_TOOLKIT_ROOT_DIR}
        PATH_SUFFIXES lib lib64 lib/x64)

find_library(TensorRT_LIBRARY_PARSERS nvparsers
        HINTS ${TensorRT_PATH} ${CUDA_TOOLKIT_ROOT_DIR}
        PATH_SUFFIXES lib lib64 lib/x64)

find_library(TensorRT_LIBRARY_NVONNXPARSER nvonnxparser
        HINTS ${TensorRT_PATH} ${CUDA_TOOLKIT_ROOT_DIR}
        PATH_SUFFIXES lib lib64 lib/x64)

set(TensorRT_LIBS
        ${TensorRT_LIBRARY_INFER} ${TensorRT_LIBRARY_INFER_PLUGIN} ${TensorRT_LIBRARY_PARSERS} ${TensorRT_LIBRARY_NVONNXPARSER} ${CUDA_LIBRARIES})

set(TensorRT_SOURCE
        ${TensorRT_PATH}/samples/common/logger.cpp)

find_package_handle_standard_args(
        TensorRT DEFAULT_MSG TensorRT_INCLUDE_DIRS TensorRT_LIBS TensorRT_SOURCE)
