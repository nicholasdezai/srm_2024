#include "tensorrt.h"

#include <NvOnnxParser.h>
#include <common.h>

#include <filesystem>

namespace srm::nn {

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

TensoRT::~TensorRT() {
  delete execution_context_;
  delete engine_;
  delete runtime;
  cudaFreeHost(input_data_host_);
  cudaFreeHost(output_data_host_);
}
bool TensorRT::Initialize(std::string REF_IN model_file, int num_classes, int num_points) {
  model_file_path_ = std::move(model_file);
  num_classes_ = std::move(num_classes);
  num_points_ = std::move(num_points);
  std::filesystem::path path(model_file_path_);
  path.replace_extension("cache");
  model_cache_path_ = path.c_str();
  path.replace_extension("int8");
  initLibNvInferPlugins(&logger_, "");
  if (!std::filesystem::exists(model_cache_path_)) BuildEngineFromONNX();
  BuildEngineFromCache();
  if (engine_ == nullptr) LOG(ERROR) << "Build CUDA engine failed.";
  if (engine_->getNbBindings() != 2) {
    LOG(ERROR) << "Invalid ONNX file type. The ONNX file must be SISO.";
    return false;
  }
  auto shape = engine_->getBindingDimensions(0);
  batches_ = shape.d[0];
  channels_ = shape.d[1];
  input_h_ = shape.d[2];
  input_w_ = shape.d[3];
  checkRuntime(cudaStreamCreate(&stream_));
  execution_context_ = engine_->createExecutionContext();
  input_numel_ = batches_ * channels_ * input_h_ * input_w_;
  checkRuntime(cudaMallocHost(&input_data_host_, input_numel_ * sizeof(float)));
  checkRuntime(cudaMalloc(&input_data_device_, input_numel_ * sizeof(float)));
  output_numel_ = batches_ * channels_ *
                  (input_w_ / 8 * input_h_ / 8 + input_w_ / 16 * input_h_ / 16 + input_w_ / 32 * input_h_ / 32) *
                  (5 + num_classes_ + 3 * num_points_);
  checkRuntime(cudaMallocHost(&output_data_host_, output_numel_ * sizeof(float)));
  checkRuntime(cudaMalloc(&output_data_device_, output_numel_ * sizeof(float)));
  return true;
}

void TensorRT::BuildEngineFromONNX() {
  LOG(INFO) << "Engine will be built from onnx.";
  auto builder = make_shared(nvinfer1::createInferBuilder(logger_));
  auto config = make_shared(builder->createBuilderConfig());
  auto network = make_shared(builder->createNetworkV2(1));
  auto parser = make_shared(nvonnxparser::createParser(*network, logger_));
  if (!parser->parseFromFile(model_file_path_.c_str(), 1)) LOG(ERROR) << "Failed to parse " << model_file_path_;
  if (builder->platformHasFastFp16()) {
    LOG(INFO) << "Platform supports fp16, fp16 is enabled.";
    config->setFlag(nvinfer1::BuilderFlag::kFP16);
  } else {
    LOG(INFO) << "FP16 is not supported on this platform, fp32 is enabled.";
    config->setFlag(nvinfer1::BuilderFlag::kTF32);
  }
  size_t free, total;
  cudaMemGetInfo(&free, &total);
  LOG(INFO) << "GPU memory total: " << (total >> 20) << "MB, free: " << (free >> 20) << "MB.";
  LOG(INFO) << "Max workspace size will use all of free GPU memory.";
  config->setMaxWorkspaceSize(free >> 1);
  auto profileStream = samplesCommon::makeCudaStream();
  config->setProfileStream(*profileStream);
  auto model_data = make_shared(builder->buildSerializedNetwork(*network, *config));
  FILE *f = fopen(model_cache_path_.c_str(), "wb");
  fwrite(model_data->data(), 1, model_data->size(), f);
  fclose(f);
  LOG(INFO) << "File has be written into cache.";
}

void TensorRT::BuildEngineFromCache() {
  LOG(INFO) << "Engine will be built from cache.";
  auto load_file = [&](const std::string &file) -> std::vector<unsigned char> {
    std::ifstream in(file, std::ios::in | std::ios::binary);
    if (!in.is_open()) return {};
    in.seekg(0, std::ios::end);
    auto length = in.tellg();
    std::vector<uint8_t> data;
    if (length > 0) {
      in.seekg(0, std::ios::beg);
      data.resize(length);
      in.read((char *)&data[0], length);
    }
    in.close();
    return data;
  };
  auto engine_data = load_file(model_cache_path_);
  runtime_ = nvinfer1::createInferRuntime(logger_);
  engine_ = runtime->deserializeCudaEngine(engine_data.data(), engine_data.size());
}

std::vector<Objects> TensorRT::Run(cv::Mat image) {
  float ro, dw, dh;
  LetterBox(image, ro, dw, dh);
  image.convertTo(image, CV_32F);
  image /= 255.f;
  cv::Mat image_splits[3];
  cv::split(image, image_splits);
  std::swap(image_splits[0], image_splits[2]);
  for (auto &&image_split : image_splits) {
    memcpy(input_data_host_, image_split.data, input_w_ * input_h_ * sizeof(float));
    input_data_host_ += input_w_ * input_h_;
  }
  input_data_host_ -= input_numel_;

  auto start = std::chrono::system_clock::now();
  checkRuntime(cudaMemcpyAsync(input_data_device_, input_data_host_, input_numel_ * sizeof(float),
                               cudaMemcpyHostToDevice, stream_));
  float *bindings[] = {input_data_device_, output_data_device_};
  execution_context_->enqueueV2((void **)bindings, stream_, nullptr);
  checkRuntime(cudaMemcpyAsync(output_data_host_, output_data_device_, output_numel_ * sizeof(float),
                               cudaMemcpyDeviceToHost, stream_));
  checkRuntime(cudaStreamSynchronize(stream_));
  auto end = std::chrono::system_clock::now();
  output_data_ = output_data_host_;
  DLOG(INFO) << "Detection time cost: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

  std::vector<Objects> objs;
  GetObjects(objs);
  NMS(objs);
  for (auto &&[x1, y1, x2, y2, prob, cls, apex] : objs) {
    x1 -= dw, x2 -= dw, y1 -= dh, y2 -= dh;
    x1 /= ro, x2 /= ro, y1 /= ro, y2 /= ro;
    for (auto &&[x, y] : apex) {
      x -= dw, y -= dh;
      x /= ro, y /= ro;
    }
  }
  return objs;
}
}  // namespace srm::nn
