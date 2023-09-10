
#include "srm/robot/serial.h"

#include <fcntl.h>
#include <glog/logging.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

std::string GetUartDeviceName() {
#if defined(__APPLE__)
  FILE *ls = popen("ls --color=never /dev/cu.usb*", "r");
#elif defined(__linux__)
  FILE *ls = popen("ls --color=never /dev/ttyACM*", "r");
#endif
  char name[127];
  auto ret = fscanf(ls, "%s", name);
  pclose(ls);
  if (ret == -1) {
    LOG(ERROR) << "No UART device found. Please check the device connection.";
    return "";
  } else
    return name;
}

namespace srm::robot {

Serial::~Serial() { Close(); }

bool Serial::Open() {
  if (com_flag_) return false;
  serial_port_ = GetUartDeviceName();
  if (serial_port_.empty()) return false;
  if (!OpenPort()) {
    serial_port_ = "";
    return false;
  }
  com_flag_ = true;
  return true;
}

void Serial::Close() {
  if (!com_flag_) return;
  ClosePort();
  serial_port_ = "";
  com_flag_ = false;
}

bool Serial::ReadData(ReceivePacket REF_OUT data) {
  if (!com_flag_) return false;
  if (std::unique_lock<std::timed_mutex>(receive_data_lock_, std::chrono::milliseconds(8)).owns_lock()) {
    if (!SerialReceive()) return false;
    data = receive_data_;
    return true;
  } else {
    LOG(WARNING) << "Reading data from serial port " << serial_port_ << " timed out due to failure to acquire lock.";
    return false;
  }
}

bool Serial::WriteData(SendPacket REF_IN data) {
  if (!com_flag_) return false;
  if (std::unique_lock<std::timed_mutex>(send_data_lock_, std::chrono::milliseconds(8)).owns_lock()) {
    send_data_ = data;
    return SerialSend();
  } else {
    LOG(WARNING) << "Writing data to serial port " << serial_port_ << " timed out due to failure to acquire lock.";
    return false;
  }
}

bool Serial::OpenPort() {
#if defined(__APPLE__)
#elif defined(__linux__)
  if (chmod(serial_port_.c_str(), S_IRWXU | S_IRWXG | S_IRWXO) == -1) {
    LOG(WARNING) << "Running in user mode, manually setting permission is required.";
    LOG(WARNING) << "To set permission of current serial port, run this command as root:";
    LOG(WARNING) << "  $ chmod 777 " << serial_port_;
  }
#endif
  serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ == -1) {
    LOG(ERROR) << "Failed to open serial port " << serial_port_ << ".";
    serial_fd_ = 0;
    return false;
  }
  termios termios{};
  tcgetattr(serial_fd_, &termios);
  cfmakeraw(&termios);
  cfsetispeed(&termios, B115200);
  cfsetospeed(&termios, B115200);
  tcsetattr(serial_fd_, TCSANOW, &termios);
  termios.c_cflag &= ~PARENB;
  termios.c_cflag &= ~CSTOPB;
  termios.c_cflag &= ~CSIZE;
  termios.c_cflag |= CS8;
  termios.c_cflag &= ~INPCK;
  termios.c_cflag |= (B115200 | CLOCAL | CREAD);
  termios.c_cflag &= ~(INLCR | ICRNL);
  termios.c_cflag &= ~(IXON);
  termios.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  termios.c_oflag &= ~OPOST;
  termios.c_oflag &= ~(ONLCR | OCRNL);
  termios.c_iflag &= ~(ICRNL | INLCR);
  termios.c_iflag &= ~(IXON | IXOFF | IXANY);
  termios.c_cc[VTIME] = 1;
  termios.c_cc[VMIN] = 1;
  tcflush(serial_fd_, TCIOFLUSH);
  LOG(INFO) << "Serial port " << serial_port_ << " is open.";
  return true;
}

void Serial::ClosePort() {
  tcflush(serial_fd_, TCIOFLUSH);
  if (close(serial_fd_) == -1)
    LOG(WARNING) << "Failed to close serial port " << serial_port_ << ".";
  else
    LOG(INFO) << "Serial port " << serial_port_ << " is closed.";
  serial_fd_ = 0;
}

bool Serial::SerialSend() {
  constexpr auto size = sizeof(SendPacket);
  tcflush(serial_fd_, TCOFLUSH);
  ssize_t send_count = write(serial_fd_, &send_data_, size);
  if (send_count < size) {
    LOG(ERROR) << "Failed to send " << size - send_count << " / " << size << " bytes of data to serial port "
               << serial_port_ << ".";
    return false;
  } else {
    DLOG(INFO) << "Sent " << size << " bytes of data to serial port " << serial_port_ << ".";
    DLOG(INFO) << send_data_;
    return true;
  }
}

bool Serial::SerialReceive() {
  constexpr auto size = sizeof(ReceivePacket);
  char read_buffer[size << 1];
  ssize_t read_count = 0;
  auto start_time = std::chrono::high_resolution_clock::now();
  while (read_count < size) {
    if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start_time)
            .count() > 8) {
      LOG(ERROR) << "Receiving " << size - read_count << " / " << size << " bytes of data from serial port "
                 << serial_port_ << " timed out.";
      LOG(FATAL) << "Exit.";
      return false;
    }
    read_count += std::max(read(serial_fd_, read_buffer + read_count, sizeof(read_buffer) - read_count), ssize_t{0});
  }
  tcflush(serial_fd_, TCIFLUSH);
  for (ssize_t i = 0; i < read_count; ++i)
    if (*((int *)(&read_buffer[i])) == 0xffffffff) {
      memcpy(&receive_data_, read_buffer + i, size);
      break;
    }
  DLOG(INFO) << "Received " << size << " bytes of data from serial port " << serial_port_ << ".";
  DLOG(INFO) << receive_data_;
  return true;
}

}  // namespace srm::robot