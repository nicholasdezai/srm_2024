#ifndef SRM_ROBOT_PACKET_H_
#define SRM_ROBOT_PACKET_H_

#include <ostream>

#include "srm/common/tags.h"

namespace srm::robot {

/// 串口接收数据结构体
struct ReceivePacket {
  int flag;            ///< 校验位
  int mode;            ///< 当前瞄准模式
  int board_size;      ///< 前哨站装甲板模式
  int reserved_i;      ///< 保留整数位
  int color;           ///< 自身颜色
  float bullet_speed;  ///< 子弹速度
  float yaw;           ///< 自身 yaw
  float pitch;         ///< 自身 pitch
  float roll;          ///< 自身 roll
  float reserved_f;    ///< 保留浮点位
};

/// 串口发送数据结构体
struct SendPacket {
  float yaw;          ///< 目标 yaw
  float pitch;        ///< 目标 pitch
  float reserved_f;   ///< 保留浮点位
  int distance_mode;  ///< 哨兵距离模式
  int fire;           ///< 哨兵是否开火
  int reserved_i[8];  ///< 保留整数位
  float check_sum;    ///< 校验和，在发送时自动计算，修改时无需更新
};

}  // namespace srm::robot

std::ostream &operator<<(std::ostream REF_OUT str, srm::robot::ReceivePacket REF_IN packet);
std::ostream &operator<<(std::ostream REF_OUT str, srm::robot::SendPacket REF_IN packet);

#endif  // SRM_ROBOT_PACKET_H_
