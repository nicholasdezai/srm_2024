#include "srm/nn/yolo.h"
namespace srm::nn {
void Yolo::LetterBox(cv::Mat REF_OUT image, float REF_OUT ro, float REF_OUT dw, float REF_OUT dh) {
  cv::Size shape = image.size();
  cv::Size new_shape = {input_w_, input_h_};
  ro = std::min(new_shape.width / (float)shape.width, new_shape.height / (float)shape.height);

  // Compute padding
  cv::Size new_unpad = {(int)round(shape.width * ro), (int)round(shape.height * ro)};
  dw = new_shape.width - new_unpad.width,
  dh = new_shape.height - new_unpad.height;  // wh padding

  // divide padding into 2 sides
  dw /= 2.0, dh /= 2.0;

  if (shape != new_unpad)  // resize
    cv::resize(image, image, new_unpad, 0, 0, cv::INTER_LINEAR);

  int top = round(dh - 0.1), bottom = round(dh + 0.1);
  int left = round(dw - 0.1), right = round(dw + 0.1);
  cv::copyMakeBorder(image, image, top, bottom, left, right, cv::BORDER_CONSTANT, {114, 114, 114});  // add border
}

void Yolo::GetObjects(std::vector<Objects> REF_OUT objs) {
  const int n = (input_w_ / 8 * input_h_ / 8 + input_w_ / 16 * input_h_ / 16 + input_w_ / 32 * input_h_ / 32);
  const int m = (4 + num_classes_ + 2 * num_points_);
  for (int i = 0; i < n; i++) {
    float data[m];
    for (int j = 0; j < m; j++) data[j] = output_data_[j * n + i];

    int cls = std::max_element(data + 4, data + 4 + num_classes_) - (data + 4);
    float prob = data[4 + cls];
    if (prob < box_conf_thresh_) continue;
    Objects obj;
    obj.cls = cls;
    obj.prob = prob;

    float x = data[0];
    float y = data[1];
    float w = data[2];
    float h = data[3];

    obj.x1 = x - w / 2;
    obj.y1 = y - h / 2;
    obj.x2 = x + w / 2;
    obj.y2 = y + h / 2;

    for (int j = 4 + num_classes_; j < m; j += 2) obj.pts.push_back({data[j], data[j + 1]});

    objs.push_back(obj);
  }
}

void Yolo::NMS(std::vector<Objects> REF_OUT objs) {
  std::sort(objs.begin(), objs.end(), [](Objects &a, Objects &b) { return a.prob > b.prob; });
  if (objs.size() > max_nms_) objs.resize(max_nms_);
  std::vector<float> vArea(objs.size());
  for (size_t i = 0; i < objs.size(); i++) {
    vArea[i] = (objs[i].x2 - objs[i].x1 + 1) * (objs[i].y2 - objs[i].y1 + 1);
  }
  for (size_t i = 0; i < objs.size(); i++) {
    for (size_t j = i + 1; j < objs.size();) {
      float xx1 = std::max(objs[i].x1, objs[j].x1);
      float yy1 = std::max(objs[i].y1, objs[j].y1);
      float xx2 = std::min(objs[i].x2, objs[j].x2);
      float yy2 = std::min(objs[i].y2, objs[j].y2);
      float w = std::max(float(0), xx2 - xx1 + 1);
      float h = std::max(float(0), yy2 - yy1 + 1);
      float inter = w * h;
      float ovr = inter / (vArea[i] + vArea[j] - inter);
      if (ovr >= iou_thresh_) {
        objs.erase(objs.begin() + j);
        vArea.erase(vArea.begin() + j);
      } else
        j++;
    }
  }
}
}  // namespace srm::nn