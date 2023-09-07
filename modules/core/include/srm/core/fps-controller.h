#ifndef _SRM_CORE_FPS_
#define _SRM_CORE_FPS_

#include <srm/common/tags.h>

#include <chrono>

namespace srm::core {

class FPSController final {
 public:
  FPSController() = default;
  ~FPSController() = default;
  void Initialize(double target_fps);
  void Tick();
  attr_reader_val(actual_fps_, GetFPS);

 private:
  int frame_count_;
  double frame_interval_;
  double actual_fps_;
  std::chrono::time_point<std::chrono::high_resolution_clock> last_time_;
};

}  // namespace srm::core
#endif  // _SRM_CORE_FPS_
