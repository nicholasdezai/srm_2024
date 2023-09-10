#include <csignal>

#include "srm/core.h"

void SignalHandler(int) {
  LOG(WARNING) << "Caught interrupt signal. Attempting to exit...";
  srm::core::Core::exit_signal_ = true;
}

int main(int arc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = FLAGS_colorlogtostderr = FLAGS_log_prefix = true;
  FLAGS_log_dir = "../log/";
  srm::cfg.Parse("../config.yaml");
  std::unique_ptr<srm::core::Core> core;
  core.reset(srm::core::CreateCore("stereo"));
  if (!core) return -1;
  if (!core->Initialize()) return 1;
  std::signal(SIGINT, &SignalHandler);
  std::signal(SIGTERM, &SignalHandler);
  int ret = core->Run();
  google::ShutdownGoogleLogging();
  return ret;
}