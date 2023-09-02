#include "srm/coord/coord.h"

#include <glog/logging.h>

#include <Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace srm::coord {

EAngle RMatToEAngle(RMat REF_IN rm) {
  if (rm(1, 0) * rm(1, 0) + rm(1, 1) * rm(1, 1) < 1e-8)
    return {atan2(rm(0, 1), rm(2, 1)), asin(-rm(1, 2)), 0};
  else
    return {atan2(rm(0, 2), rm(2, 2)), asin(-rm(1, 2)), atan2(rm(1, 0), rm(1, 1))};
}

RMat EAngleToRMat(EAngle REF_IN ea) {
  RMat rm_x, rm_y, rm_z;
  rm_x << 1, 0, 0, 0, cos(ea[1]), -sin(ea[1]), 0, sin(ea[1]), cos(ea[1]);
  rm_y << cos(ea[0]), 0, sin(ea[0]), 0, 1, 0, -sin(ea[0]), 0, cos(ea[0]);
  rm_z << cos(ea[2]), -sin(ea[2]), 0, sin(ea[2]), cos(ea[2]), 0, 0, 0, 1;
  return rm_y * rm_x * rm_z;
}

CTVec STVecToCTVec(STVec REF_IN stv) {
  return {stv.z() * cos(stv.y()) * sin(stv.x()), -stv.z() * sin(stv.y()), stv.z() * cos(stv.y()) * cos(stv.x())};
}

STVec CTVecToSTVec(CTVec REF_IN ctv) {
  return {atan2(ctv.x(), ctv.z()), atan2(-ctv.y(), sqrt(ctv.x() * ctv.x() + ctv.z() * ctv.z())),
          sqrt(ctv.x() * ctv.x() + ctv.y() * ctv.y() + ctv.z() * ctv.z())};
}

RMat RVecToRMat(RVec REF_IN rv) {
  cv::Mat rv_cv, rm_cv;
  RMat rm;
  cv::eigen2cv(rv, rv_cv);
  cv::Rodrigues(rv_cv, rm_cv);
  cv::cv2eigen(rm_cv, rm);
  return rm;
}

RVec RMatToRVec(RMat REF_IN rm) {
  cv::Mat rv_cv, rm_cv;
  RVec rv;
  cv::eigen2cv(rm, rm_cv);
  cv::Rodrigues(rm_cv, rv_cv);
  cv::cv2eigen(rv_cv, rv);
  return rv;
}

}  // namespace srm::coord
