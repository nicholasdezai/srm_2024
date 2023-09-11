#include <glog/logging.h>
#include <cstring>
#include <opencv2/opencv.hpp>

#include "srm/common/tags.h"
#include "srm/nn/yolo.h"

#import "Model.h"  //! OpenCV 的头文件必须在 Obj-C 前面

namespace srm::nn {

/**
 * @brief CoreML平台的Yolo推理类
 * @warning 禁止直接构造此类，请使用 @code srm::video::CreateYolo("coreml") @endcode 获取该类的公共接口指针
 */
class CoreML final : public Yolo {
 public:
  CoreML() = default;
  ~CoreML() = default;
  bool Initialize(std::string REF_IN model_file, int num_class, int num_points) override;
  std::vector<Objects> Run(cv::Mat image) override;

 private:
  Model *model_;                  ///< 模型指针
  CVPixelBufferRef pixelBuffer_;  ///< 网络原始输出

  inline static auto registry_ = RegistrySub<Yolo, CoreML>("coreml");  ///< 神经网络注册信息
};

bool CoreML::Initialize(std::string REF_IN model_file, int num_classes, int num_points) {
  num_classes_ = num_classes;
  num_points_ = num_points;

  const std::string &s = model_file;
  int p = s.rfind('.');
  if (s.substr(p + 1) != "mlmodelc") {
    LOG(ERROR) << "The extension of the file should be named as \'.mlmodelc\'";
    return false;
  }

  NSError *error = nil;
  NSString *file = [NSString stringWithCString:s.substr(0, p).c_str() encoding:NSUTF8StringEncoding];
  NSString *extension = [NSString stringWithCString:s.substr(p + 1).c_str() encoding:NSUTF8StringEncoding];
  NSURL *modelURL = [[NSBundle mainBundle] URLForResource:file withExtension:extension];

  model_ = [[Model alloc] initWithContentsOfURL:modelURL error:&error];
  if (error) {
    LOG(ERROR) << "Fail to load model: " << [[error localizedDescription] UTF8String];
    return false;
  }

  input_w_ = model_.inputW;
  input_h_ = model_.inputH;

  CVPixelBufferCreate(
      kCFAllocatorDefault, input_w_, input_h_, kCVPixelFormatType_32BGRA,
      (__bridge CFDictionaryRef)
          @{(NSString *)kCVPixelBufferIOSurfacePropertiesKey : @{}},
      &pixelBuffer_);

  return true;
}

std::vector<Objects> CoreML::Run(cv::Mat image) {
  float ro, dw, dh;
  LetterBox(image, ro, dw, dh);

  CVPixelBufferLockBaseAddress(pixelBuffer_, 0);
  void *dest = CVPixelBufferGetBaseAddress(pixelBuffer_);
  size_t bytesPerRow = CVPixelBufferGetBytesPerRow(pixelBuffer_);
  cv::Mat cvDest(image.rows, image.cols, CV_8UC4, dest, bytesPerRow);
  cv::cvtColor(image, cvDest, cv::COLOR_BGR2BGRA);
  CVPixelBufferUnlockBaseAddress(pixelBuffer_, 0);

  NSError *error = nil;
  ModelOutput *outputRaw = [model_ predictionFromImage:pixelBuffer_ error:&error];
  if (error) {
    LOG(ERROR) << "Failed to get output from model: " << [[error localizedDescription] UTF8String];
    return {};
  }
  output_data_ = reinterpret_cast<float *>(outputRaw.out.dataPointer);

  std::vector<Objects> objs;
  GetObjects(objs);
  NMS(objs);

  for (auto &[x1, y1, x2, y2, prob, cls, apex] : objs) {
    x1 -= dw, x2 -= dw, y1 -= dh, y2 -= dh;
    x1 /= ro, x2 /= ro, y1 /= ro, y2 /= ro;

    for (auto &[x, y] : apex) {
      x -= dw, y -= dh;
      x /= ro, y /= ro;
    }
  }

  delete[] output_data_;
  return objs;
}

}  // namespace srm::nn