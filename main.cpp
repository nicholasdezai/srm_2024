#include <srm/nn.h>

#include <iostream>
#include <opencv2/opencv.hpp>

int main() {
  auto it = srm::nn::CreateYolo("tensorrt");
  std::cout << it << std::endl;
  it->Initialize("armor.onnx", 36, 4);
  auto img = cv::imread("test.jpg");
  auto objs = it->Run(img);
  cv::line(img, objs[0].pts[0], objs[0].pts[1], {255, 255, 255});
  cv::line(img, objs[0].pts[1], objs[0].pts[2], {255, 255, 255});
  cv::line(img, objs[0].pts[2], objs[0].pts[3], {255, 255, 255});
  cv::line(img, objs[0].pts[3], objs[0].pts[0], {255, 255, 255});
  cv::imshow("a", img);
  cv::waitKey();
  return 0;
}