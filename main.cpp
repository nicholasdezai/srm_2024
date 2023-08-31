#include <iostream>

#include "srm/video.h"

int main() {
  auto reader = srm::video::CreateReader("file");
  std::cout << reader << std::endl;
  return 0;
}