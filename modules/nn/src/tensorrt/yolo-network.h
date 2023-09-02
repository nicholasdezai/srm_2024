#ifndef _SRM_NN_TENSORRT_YOLO_NETWORK_H_
#define _SRM_NN_TENSORRT_YOLO_NETWORK_H_

#include "cuda-help.h"
#include "srm/nn/yolo.h"

namespace srm::nn {

/// YOLOv8 接口（TensorRT 实现）
class TensorRT final : public Yolo {
 public:
  /**
   * @brief 接口构造
   * @note 类的数量和识别的关键点数量必须在这里指定
   * @param num_classes 类的数量（装甲板：36，能量机关：2）
   * @param num_points 关键点的数量（装甲板：4，能量机关：5）
   * @param box_conf_thresh 物体框置信度阀值
   * @param max_nms NMS 合并框的最大值
   * @param iou_thresh NMS 合并的阀值
   */
  TensorRT() = default;
  ~TensorRT() = default;

  /**
   * @brief 初始化，载入模型文件
   * @param model_file_path 模型的文件路径
   */
  bool Initialize(std::string REF_IN model_file, int num_classes, int num_points) override;

  /**
   * @brief 识别图像
   * @param [in] _image 需要检测的图片
   * @return 检测到的物体
   */
  std::vector<Objects> Run(cv::Mat image) override;

 private:
  /// 相机注册信息
  inline static auto registry_ = factory::RegistrySub<Yolo, TensorRT>("tensorrt");

  int batches_{}, channels_{};

  std::string model_file_path_, model_cache_path_;

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
