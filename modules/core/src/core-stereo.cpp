#include <srm/core.h>

namespace srm::core {

class StereoCore : public Core {
 public:
  ~StereoCore() final;
  bool Initialize() final;
  int Run() final;

 private:
  inline static auto registry_ = RegistrySub<Core, StereoCore>("stereo");  ///< 主控注册信息
};

int StereoCore::Run() {
  cv::namedWindow("test");
  while (!Core::exit_signal_) {
    video::Frame frame;
    while (!reader_->GetFrame(frame))
      ;
    cv::imshow("test", frame.image);
    cv::waitKey(1);
    fps_controller_->Tick();
    LOG_EVERY_N(INFO, 100) << fps_controller_->GetFPS();
  }
  return 0;
}

StereoCore::~StereoCore() {
  for (int i = 0; i < reader_->SourceCount(); i++) reader_->UnregisterFrameCallback(&frame_callbacks_[i]);
}

bool StereoCore::Initialize() {
  //  ---------------video模块初始化---------------
  auto reader_type = cfg.Get<std::string>({"video.reader"});
  auto video_from = cfg.Get<std::string>({"video.from"});
  reader_.reset(video::CreateReader(reader_type));
  if (!reader_) {
    LOG(ERROR) << "Failed to create " << reader_type << " video source.";
    return false;
  }
  if (!reader_->Initialize("video." + video_from + "." + reader_type)) {
    LOG(ERROR) << "Failed to initialize video source.";
    reader_.reset();
    return false;
  }
  if (reader_->SourceCount() != 2) {
    LOG(ERROR) << "Wrong camera number for stereo vision.";
    reader_.reset();
    return false;
  }
  video::Frame frame;
  for (int i = 0; i < reader_->SourceCount(); i++) {
    while (!reader_->GetFrame(frame, i)) {
      LOG(WARNING) << "Waiting for the first frame from video source" + std::to_string(i) + ".";
      sleep(1);
    }
  }
  if (cfg.Get<bool>({"video.writer"})) {
    writer_ = std::make_unique<video::Writer>();
    time_t t = time(nullptr);
    char t_str[128];
    strftime(t_str, sizeof(t_str), "%Y-%m-%d-%H.%M.%S", localtime(&t));
    if (!writer_->Open("../cache/" + video_from + "-" + t_str + ".avi", {frame.image.cols, frame.image.rows})) {
      LOG(ERROR) << "Failed to open video file. Please check your disk space.";
      reader_.reset();
      solver_.reset();
      serial_.reset();
      writer_.reset();
      return false;
    }
  }

  //  ---------------coord模块初始化---------------
  auto coord_from = cfg.Get<std::string>({"coord.from"});
  auto M0 = reader_->IntrinsicMat(0), D0 = reader_->DistortionMat(0);
  auto M1 = reader_->IntrinsicMat(1), D1 = reader_->DistortionMat(1);
  auto R = cfg.Get<cv::Mat>({"coord", coord_from, "R"});
  auto T = cfg.Get<cv::Mat>({"coord", coord_from, "T"});
  if (R.empty() || T.empty()) {
    LOG(ERROR) << "Invalid rotation matrix or translate matrix of camera1. Please check your configuration.";
    reader_.reset();
    return false;
  }
  cv::Mat R0, P0, R1, P1, map00, map01, map10, map11, dsp_to_dep_mat;
  stereoRectify(M0, D0, M1, D1, frame.image.size(), R, T, R0, R1, P0, P1, dsp_to_dep_mat, cv::CALIB_ZERO_DISPARITY);
  initUndistortRectifyMap(M0, D0, R0, P0, frame.image.size(), CV_16SC2, map00, map01);
  initUndistortRectifyMap(M1, D1, R1, P1, frame.image.size(), CV_16SC2, map10, map11);

  frame_callbacks_.resize(reader_->SourceCount());
  frame_callbacks_[0] = [map00, map01](void *obj, video::Frame &frame) {
    remap(frame.image, frame.image, map00, map01, cv::INTER_LINEAR);
    auto self = static_cast<StereoCore *>(obj);
    if (self->serial_) {
      auto *receive_packet = new robot::ReceivePacket();
      if (!self->serial_->ReadData(*receive_packet)) {
        LOG(WARNING) << "Failed to read data from serial port in frame callback function. Set this frame as invalid.";
        frame.valid = false;
      }
      frame.sync_data.reset(receive_packet);
    }
  };
  frame_callbacks_[1] = [map10, map11](void *obj, video::Frame &frame) {
    remap(frame.image, frame.image, map10, map11, cv::INTER_LINEAR);
  };
  for (int i = 0; i < reader_->SourceCount(); i++) reader_->RegisterFrameCallback(&frame_callbacks_[i], this);

  solver_ = std::make_unique<coord::Solver>();
  if (!solver_->Initialize("coord." + coord_from, P0(cv::Rect(0, 0, 3, 3)), {}, dsp_to_dep_mat)) {
    LOG(ERROR) << "Failed to initialize coordinate solver.";
    reader_.reset();
    solver_.reset();
    return false;
  }

  //  ---------------serial模块初始化---------------
  if (cfg.Get<bool>({"robot.serial"})) {
    serial_ = std::make_unique<robot::Serial>();
    if (!serial_->Open()) {
      LOG(ERROR) << "Failed to open serial communication.";
      reader_.reset();
      solver_.reset();
      serial_.reset();
      return false;
    }
  }

  //  ---------------fps功能初始化---------------
  fps_controller_ = std::make_unique<FPSController>();
  fps_controller_->Initialize(cfg.Get<double>({"core.fps_limit"}));

  LOG(INFO) << "Initialized base environment of " << cfg.Get<std::string>({"global.from"}) << " controller.";
  return true;
}
}  // namespace srm::core