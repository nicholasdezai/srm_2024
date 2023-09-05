#ifndef SRM_CORE_H_
#define SRM_CORE_H_

#include "srm/common.h"
#include "srm/coord.h"
#include "srm/nn.h"
#include "srm/robot.h"
#include "srm/video.h"

/// 抓取并处理来自操作系统的控制信号
void SignalHandler(int);

enable_factory(srm::core, Core);

namespace srm::core {

/// 机器人主控公共接口类
class Core {
  friend void ::SignalHandler(int);

 public:
  Core() = default;
  virtual ~Core() = default;

  /**
   * @brief 初始化机器人
   * @return 是否初始化成功
   */
  virtual bool Initialize() = 0;

  /**
   * @brief 执行主控制循环
   * @return 错误码，可作为 main() 的返回值
   */
  virtual int Run() = 0;

 protected:
  static std::atomic_bool exit_signal_;  ///< 主循环退出信号

  video::Frame frame_;                                                        ///< 帧数据
  std::vector<std::function<void(void *, video::Frame &)>> frame_callbacks_;  ///< 取图回调函数

  std::unique_ptr<video::Reader> reader_;  ///< 视频读入接口
  std::unique_ptr<video::Writer> writer_;  ///< 视频写入接口
  std::unique_ptr<robot::Serial> serial_;  ///< 串口收发接口
  std::unique_ptr<coord::Solver> solver_;  ///< 坐标求解接口
};

}  // namespace srm::core

#endif  // SRM_CORE_H_
