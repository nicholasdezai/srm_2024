#ifndef _SRM_CORE_FPS_
#define _SRM_CORE_FPS_

#include <srm/common/tags.h>

#include <chrono>

namespace srm::core {

/// 运行帧率控制类
class FPSController final {
 public:
  FPSController() = default;
  ~FPSController() = default;

  /**
   * @brief 初始化
   * @param target_fps 要设置的帧率
   */
  void Initialize(double target_fps);

  /**
   * @brief 控制函数
   * @details 在循环内加入该语句来限制帧率
   */
  void Tick();

  /// 获取实际的帧数
  attr_reader_val(actual_fps_, GetFPS);

 private:
  double frame_interval_;
  double actual_fps_;
  std::chrono::time_point<std::chrono::high_resolution_clock> last_time_;
};

}  // namespace srm::core
#endif  // _SRM_CORE_FPS_
