#include <srm/nn.h>

#include <iostream>

#include "srm/video.h"

int main() {
  auto it = srm::nn::CreateYolo("coreml");
  std::cout << it << std::endl;
  it->Initialize("test", 36, 4);
  return 0;
}