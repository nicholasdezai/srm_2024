#include "srm/robot/packet.h"

std::ostream &operator<<(std::ostream REF_OUT str, srm::robot::ReceivePacket REF_IN packet) {
  return str << packet.mode << " | " << packet.board_size << " | " << packet.color << " | " << packet.bullet_speed
             << " | " << packet.yaw << " | " << packet.pitch << " | " << packet.roll;
}

std::ostream &operator<<(std::ostream REF_OUT str, srm::robot::SendPacket REF_IN packet) {
  return str << packet.yaw << " | " << packet.pitch << " | " << packet.distance_mode << " | " << packet.fire;
}
