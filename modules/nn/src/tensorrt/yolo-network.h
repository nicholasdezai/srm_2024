#ifndef _SRM_NN_TENSORRT_H_
#define _SRM_NN_TENSORRT_H_

#include "cuda-help.h"
#include "srm/nn/yolo.h"

namespace srm::nn {

/// YOLOv8 接口（TensorRT 实现）
class TensorRT final : public Yolo {
 public:
  TensorRT() = default;
  ~TensorRT() override;
  bool Initialize(std::string REF_IN model_file, int num_classes, int num_points) override;
  std::vector<Objects> Run(cv::Mat image) override;

 private:
  inline static auto registry_ = factory::RegistrySub<Yolo, TensorRT>("tensorrt");

  int batches_{}, channels_{};

  std::string model_file_path_, model_cache_path_;

  nvinfer1::IRuntime *runtime_{};
  nvinfer1::ICudaEngine *engine_{};
  nvinfer1::IExecutionContext *execution_context_{};
  cudaStream_t stream_{};
  TRTLogger logger_;

  int input_numel_ = 0;
  float *input_data_host_{};
  float *input_data_device_{};

  int output_numel_ = 0;
  float *output_data_host_{};
  float *output_data_device_{};

  void BuildEngineFromONNX();
  void BuildEngineFromCache();
};
}  // namespace srm::nn

#endif  // _SRM_NN_TENSORRT_YOLO_NETWORK_H_
