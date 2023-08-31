#include <glog/logging.h>

#include <opencv2/core/persistence.hpp>

#include "srm/video/camera.h"
#include "srm/video/reader.h"

namespace srm::video {
/**
 * @brief 相机视频源接口类
 * @warning 禁止直接构造此类，请使用 @code srm::video::CreateReader("camera") @endcode 获取该类的公共接口指针
 */
class CameraReader final : public Reader {
 public:
  CameraReader() = default;
  ~CameraReader() final = default;

  bool Initialize(std::string REF_IN config_file) final;
  bool GetFrame(Frame REF_OUT frame, int id = 0) final;
  void RegisterFrameCallback(FrameCallback callback, void *obj, int id = 0) final;
  void UnregisterFrameCallback(FrameCallback callback, int id = 0) final;
  const cv::Mat &IntrinsicMat(int id = 0) const final;
  const cv::Mat &DistortionMat(int id = 0) const final;

 private:
  struct CameraInfo {
    std::unique_ptr<Camera> camera{};  ///< 相机指针
    cv::Mat intrinsic_mat;             ///< 相机针孔模型矩阵
    cv::Mat distortion_mat;            ///< 相机畸变矩阵
  };

  inline static auto registry_ = factory::RegistrySub<Reader, CameraReader>("camera");  ///< 视频源注册信息

  std::vector<std::unique_ptr<CameraInfo>> cams_info_{};  ///< 相机公共接口指针
};

bool CameraReader::Initialize(std::string REF_IN config_file) {
  cv::FileStorage init_config;
  init_config.open(config_file, cv::FileStorage::READ);
  if (!init_config.isOpened()) {
    LOG(ERROR) << "Failed to open camera initialization file " << config_file << ".";
    return false;
  }
  std::string cam_config_file;
  init_config["CAM_CONFIG"] >> cam_config_file;
  cv::FileStorage cam_config;
  cam_config.open(cam_config_file, cv::FileStorage::READ);
  if (!cam_config.isOpened()) {
    LOG(ERROR) << "Failed to open camera config file " << cam_config_file << ".";
    return false;
  }

  auto CreateCamera = [&init_config, &cam_config](std::string cam_name,
                                                  std::unique_ptr<CameraReader::CameraInfo> REF_OUT cam_info) {
    auto &[camera, intrinsic_mat, distortion_mat] = *cam_info;

    std::string cameratype;
    cam_config[cam_name]["TYPE"] >> cameratype;
    camera.reset(::srm::video::CreateCamera(cameratype));
    if (!camera) {
      LOG(ERROR) << "Failed to create camera object of type " << cameratype << ".";
      return false;
    }

    std::string serial_number, cameraconfig_file;
    cam_config[cam_name]["SN"] >> serial_number;
    cam_config[cam_name]["CONFIG"] >> cameraconfig_file;
    if (!camera->OpenCamera(serial_number, cameraconfig_file)) {
      camera.reset();
      return false;
    }

    int exposure_time;
    cam_config[cam_name]["EXPOSURE_TIME"] >> exposure_time;
    if (!camera->SetExposureTime(static_cast<uint32_t>(exposure_time)))
      LOG(WARNING) << "Failed to set exposure time of camera.";

    float gain_value;
    cam_config[cam_name]["GAIN_VALUE"] >> gain_value;
    if (!camera->SetGainValue(gain_value)) LOG(WARNING) << "Failed to set gain value of camera.";

    int time_stamp_ns = 0;
    cam_config[cam_name]["TIME_STAMP_NS"] >> time_stamp_ns;
    if (!camera->SetTimeStampNS(time_stamp_ns))
      LOG(WARNING) << "Invalid value for time stamp unit. Please check your configuration.";

    cam_config[cam_name]["IntrinsicMatrix"] >> intrinsic_mat;
    cam_config[cam_name]["DistortionMatrix"] >> distortion_mat;

    double frame_rate;
    init_config["FRAME_RATE"] >> frame_rate;
    if (!camera->SetFrameRate(frame_rate))
      LOG(WARNING) << "Failed to set frame rate of camera. Fallback to default value.";

    int hardware_trigger = 0;
    init_config["HARDWARE_TRIGGER"] >> hardware_trigger;
    if (!camera->SetHardwareTriggerMode(hardware_trigger))
      LOG(WARNING) << "Failed to set hardware trigger mode of camera. Fallback to software trigger mode.";

    if (!camera->StartStream()) {
      camera.reset();
      return false;
    }
    return true;
  };

  int cam_num;
  init_config["CAM_NUM"] >> cam_num;
  source_count_ = cam_num;
  cams_info_.resize(cam_num);

  for (int i = 0; i < cam_num; i++) {
    cams_info_[i] = std::make_unique<CameraInfo>();
    std::string cam_name;
    init_config["CAMERA" + std::to_string(i)] >> cam_name;
    if (!CreateCamera(cam_name, cams_info_[i])) {
      LOG(ERROR) << "Create camera" + std::to_string(i) + " failed.";
      return false;
    }
  }

  return true;
}

bool CameraReader::GetFrame(Frame REF_OUT frame, int id) {
  return cams_info_[id] && cams_info_[id]->camera && cams_info_[id]->camera->GetFrame(frame);
}

void CameraReader::RegisterFrameCallback(FrameCallback callback, void *obj, int id) {
  if (cams_info_[id] && cams_info_[id]->camera) cams_info_[id]->camera->RegisterFrameCallback(callback, obj);
  DLOG(INFO) << "Registered video source callback FUNC " << callback << " OBJ " << obj << ".";
}

void CameraReader::UnregisterFrameCallback(FrameCallback callback, int id) {
  if (cams_info_[id] && cams_info_[id]->camera) cams_info_[id]->camera->UnregisterFrameCallback(callback);
  DLOG(INFO) << "Unregistered video source callback FUNC " << callback << ".";
}

const cv::Mat &CameraReader::IntrinsicMat(int id) const { return cams_info_[id]->intrinsic_mat; }

const cv::Mat &CameraReader::DistortionMat(int id) const { return cams_info_[id]->distortion_mat; }
}  // namespace srm::video
