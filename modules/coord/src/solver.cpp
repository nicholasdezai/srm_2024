#include "srm/coord/solver.h"

#include <glog/logging.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

namespace srm::coord {

bool CoordSolver::Initialize(std::string REF_IN config_file, cv::Mat intrinsic_mat, cv::Mat distortion_mat,
                             cv::Mat dsp_to_dep_mat) {
  cv::FileStorage coord_config;
  coord_config.open(config_file, cv::FileStorage::READ);
  if (!coord_config.isOpened()) {
    LOG(ERROR) << "Failed to open coordinate configuration file " << config_file << ".";
    return false;
  }
  std::vector<double> ctv_iw_std, ctv_ci_std, ctv_mi_std, ea_ci_std, ea_mi_std;
  coord_config["CTV_IMU_WORLD"] >> ctv_iw_std;
  coord_config["CTV_CAM_IMU"] >> ctv_ci_std;
  coord_config["CTV_MUZZLE_IMU"] >> ctv_mi_std;
  coord_config["EA_CAM_IMU"] >> ea_ci_std;
  coord_config["EA_MUZZLE_IMU"] >> ea_mi_std;
  if (ctv_iw_std.size() != 3 || ctv_ci_std.size() != 3 || ctv_mi_std.size() != 3 || ea_ci_std.size() != 3 ||
      ea_mi_std.size() != 3) {
    LOG(ERROR) << "Failed to read coordinate configurations.";
    return false;
  }
  ctv_iw_ << ctv_iw_std[0], ctv_iw_std[1], ctv_iw_std[2];
  ctv_iw_ *= 1e-3;
  ctv_ci_ << ctv_ci_std[0], ctv_ci_std[1], ctv_ci_std[2];
  ctv_ci_ *= 1e-3;
  ctv_mi_ << ctv_mi_std[0], ctv_mi_std[1], ctv_mi_std[2];
  ctv_mi_ *= 1e-3;
  ctv_cw_ = ctv_ci_ + ctv_iw_;
  ctv_mw_ = ctv_mi_ + ctv_iw_;
  EAngle ea_ci;
  ea_ci << ea_ci_std[0], ea_ci_std[1], ea_ci_std[2];
  ea_ci *= M_PI / 180;
  rm_ci_ = EAngleToRMat(ea_ci);
  EAngle ea_mi;
  ea_mi << ea_mi_std[0], ea_mi_std[1], ea_mi_std[2];
  ea_mi *= M_PI / 180;
  rm_mi_ = EAngleToRMat(ea_mi);
  htm_ci_ << rm_ci_(0, 0), rm_ci_(0, 1), rm_ci_(0, 2), ctv_ci_.x(), rm_ci_(1, 0), rm_ci_(1, 1), rm_ci_(1, 2),
      ctv_ci_.y(), rm_ci_(2, 0), rm_ci_(2, 1), rm_ci_(2, 2), ctv_ci_.z(), 0, 0, 0, 1;
  htm_mi_ << rm_mi_(0, 0), rm_mi_(0, 1), rm_mi_(0, 2), ctv_mi_.x(), rm_mi_(1, 0), rm_mi_(1, 1), rm_mi_(1, 2),
      ctv_mi_.y(), rm_mi_(2, 0), rm_mi_(2, 1), rm_mi_(2, 2), ctv_mi_.z(), 0, 0, 0, 1;
  intrinsic_mat_ = std::move(intrinsic_mat);
  distortion_mat_ = std::move(distortion_mat);
  if (!dsp_to_dep_mat.empty()) cv::cv2eigen(std::move(dsp_to_dep_mat), dsp_to_dep_mat_);
  cv::cv2eigen(intrinsic_mat_, intrinsic_mat_eigen_);
  fx_ = intrinsic_mat_eigen_(0, 0);
  fy_ = intrinsic_mat_eigen_(1, 1);
  cx_ = intrinsic_mat_eigen_(0, 2);
  cy_ = intrinsic_mat_eigen_(1, 2);
  htm_ic_ = htm_ci_.inverse();
  htm_im_ = htm_mi_.inverse();
  LOG(INFO) << "Initialized coordinate solver.";
  return true;
}

void CoordSolver::SolvePnP(std::array<cv::Point3d, 4> REF_IN p3d_world, std::array<cv::Point2f, 4> REF_IN p2d_pic,
                           RMat REF_IN rm_imu, PnPInfo REF_OUT pnp_info) {
  cv::Mat rv_cam_cv, ctv_cam_cv, rm_cam_cv;
  cv::solvePnP(p3d_world, p2d_pic, intrinsic_mat_, distortion_mat_, rv_cam_cv, ctv_cam_cv, false,
               cv::SOLVEPNP_ITERATIVE);
  cv::Rodrigues(rv_cam_cv, rm_cam_cv);
  cv::cv2eigen(rv_cam_cv, pnp_info.rv_cam);
  cv::cv2eigen(rm_cam_cv, pnp_info.rm_cam);
  cv::cv2eigen(ctv_cam_cv, pnp_info.ctv_cam);
  pnp_info.ea_cam = RMatToEAngle(pnp_info.rm_cam);
  pnp_info.ctv_world = CamToWorld(pnp_info.ctv_cam, rm_imu);
  pnp_info.stv_cam = CTVecToSTVec(pnp_info.ctv_cam);
  pnp_info.stv_world = CTVecToSTVec(pnp_info.ctv_world);
}

CTVec CoordSolver::CamToWorld(CTVec REF_IN ctv_cam, RMat REF_IN rm_imu) const {
  HCTVec hctv_cam, hctv_imu;
  CTVec ctv_imu;
  hctv_cam << ctv_cam[0], ctv_cam[1], ctv_cam[2], 1;
  hctv_imu = htm_ci_ * hctv_cam;
  ctv_imu << hctv_imu[0], hctv_imu[1], hctv_imu[2];
  ctv_imu += ctv_iw_;
  return rm_imu * ctv_imu;
}

CTVec CoordSolver::WorldToCam(CTVec REF_IN ctv_world, RMat REF_IN rm_imu) const {
  HCTVec hctv_cam, hctv_imu;
  CTVec ctv_imu;
  ctv_imu = rm_imu.transpose() * ctv_world;
  ctv_imu -= ctv_iw_;
  hctv_imu << ctv_imu[0], ctv_imu[1], ctv_imu[2], 1;
  hctv_cam = htm_ic_ * hctv_imu;
  return {hctv_cam[0], hctv_cam[1], hctv_cam[2]};
}

CTVec CoordSolver::MuzzleToWorld(CTVec REF_IN ctv_muzzle, RMat REF_IN rm_imu) const {
  HCTVec hctv_muzzle, hctv_imu;
  CTVec ctv_imu;
  hctv_muzzle << ctv_muzzle[0], ctv_muzzle[1], ctv_muzzle[2], 1;
  hctv_imu = htm_mi_ * hctv_muzzle;
  ctv_imu << hctv_imu[0], hctv_imu[1], hctv_imu[2];
  ctv_imu += ctv_iw_;
  return rm_imu * ctv_imu;
}

CTVec CoordSolver::WorldToMuzzle(CTVec REF_IN ctv_world, RMat REF_IN rm_imu) const {
  HCTVec hctv_muzzle, hctv_imu;
  CTVec ctv_imu, ctv_muzzle;
  ctv_imu = rm_imu.transpose() * ctv_world;
  ctv_imu -= ctv_iw_;
  hctv_imu << ctv_imu[0], ctv_imu[1], ctv_imu[2], 1;
  hctv_muzzle = htm_im_ * hctv_imu;
  return {hctv_muzzle[0], hctv_muzzle[1], hctv_muzzle[2]};
}

cv::Point2f CoordSolver::CamToPic(CTVec REF_IN ctv_cam) {
  CTVec result = (1.f / ctv_cam.z()) * intrinsic_mat_eigen_ * ctv_cam;
  return {static_cast<float>(result.x()), static_cast<float>(result.y())};
}

CTVec CoordSolver::DisparityToDepth(HCTVec REF_IN hctv_pic_d) {
  HCTVec hctv_cam = dsp_to_dep_mat_ * hctv_pic_d;
  return {hctv_cam[0] / hctv_cam[3], hctv_cam[1] / hctv_cam[3], hctv_cam[2] / hctv_cam[3]};
}
}  // namespace srm::coord