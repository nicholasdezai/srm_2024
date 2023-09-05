#include "srm/core.h"

std::atomic_bool srm::core::Core::exit_signal_ = false;

void SignalHandler(int) {
  LOG(WARNING) << "Caught interrupt signal. Attempting to exit...";
  srm::core::Core::exit_signal_ = true;
}

int main(int arc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = FLAGS_colorlogtostderr = FLAGS_log_prefix = true;
  FLAGS_log_dir = "../log/";

  srm::cfg.Parse("../config.yaml");
  auto it = srm::core::CreateCore("stereo");
  it->Initialize();
  it->Run();

  google::ShutdownGoogleLogging();
  return 0;
}