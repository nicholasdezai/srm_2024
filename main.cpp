#include <srm/nn.h>

#include <iostream>
#include <opencv2/opencv.hpp>

int main() {
  auto it = srm::nn::CreateYolo("tensorrt");
  std::cout << it << std::endl;
  it->Initialize("armor.onnx", 36, 4);
  auto img = cv::imread("test.jpg");
  auto objs = it->Run(img);
  for (auto &x : objs) std::cout << x.x1 << " " << x.y1 << " " << x.x2 << " " << x.y2 << std::endl;
  return 0;
}