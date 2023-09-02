#ifndef SRM_COORD_SOLVER_H_
#define SRM_COORD_SOLVER_H_

#include <srm/coord/coord.h>

#include <Eigen/LU>
#include <opencv2/core/mat.hpp>

namespace srm::coord {

/// 坐标系求解器类
class CoordSolver {
  // 世界坐标系：原点为车身上的固定点，无旋转，随车身平移
  // 陀螺仪坐标系：原点为陀螺仪，无旋转，与世界坐标系原点位置关系固定
  // 相机坐标系：原点为相机镜头，自身可旋转，与陀螺仪坐标系原点位置关系固定

 private:
  cv::Mat intrinsic_mat_;                ///< 相机矩阵
  cv::Mat distortion_mat_;               ///< 畸变矩阵
  Eigen::Matrix3d intrinsic_mat_eigen_;  ///< Eigen格式相机矩阵
  Eigen::Matrix4d dsp_to_dep_mat_;       ///< 双目视差转深度矩阵

  CTVec ctv_iw_;  ///< 陀螺仪相对世界坐标系原点的位移
  CTVec ctv_ci_;  ///< 相机相对陀螺仪的位移
  CTVec ctv_mi_;  ///< 枪口相对陀螺仪的位移
  CTVec ctv_cw_;  ///< 相机相对世界坐标系原点的位移
  CTVec ctv_mw_;  ///< 枪口相对世界坐标系原点的位移
  CTVec ctv_mc_;  ///< 枪口相对相机的位移
  RMat rm_ci_;    ///< 相机相对陀螺仪的旋转矩阵
  RMat rm_mi_;    ///< 枪口相对陀螺仪的旋转矩阵
  HTMat htm_ic_;  ///< 陀螺仪坐标系转换到相机坐标系
  HTMat htm_ci_;  ///< 相机坐标系转换到陀螺仪坐标系
  HTMat htm_im_;  ///< 陀螺仪坐标系转换到枪口坐标系
  HTMat htm_mi_;  ///< 枪口坐标系转换到陀螺仪坐标系

  double fx_, fy_, cx_, cy_;  ///< 相机内参

 public:
  attr_reader_ref(ctv_iw_, CTVecIMUWorld);     ///< 陀螺仪相对世界坐标系原点的位移
  attr_reader_ref(ctv_ci_, CTVecCamIMU);       ///< 相机相对陀螺仪的位移
  attr_reader_ref(ctv_mi_, CTVecMuzzleIMU);    ///< 枪口相对陀螺仪的位移
  attr_reader_ref(ctv_cw_, CTVecCamWorld);     ///< 相机相对世界坐标系原点的位移
  attr_reader_ref(ctv_mw_, CTVecMuzzleWorld);  ///< 枪口相对世界坐标系原点的位移
  attr_reader_ref(rm_ci_, RMatCamIMU);         ///< 相机相对陀螺仪的旋转矩阵
  attr_reader_ref(rm_mi_, RMatMuzzleIMU);      ///< 枪口相对陀螺仪的旋转矩阵

  /**
   * @brief 初始化坐标系参数
   * @param [in] config_file 配置文件名
   * @param intrinsic_mat 相机矩阵
   * @param distortion_mat 畸变矩阵
   * @param dsp_to_dep_mat 双目视差转深度矩阵
   * @return 初始化过程是否正常完成
   */
  bool Initialize(std::string REF_IN config_file, cv::Mat intrinsic_mat, cv::Mat distortion_mat,
                  cv::Mat dsp_to_dep_mat);

  /**
   * @brief 解算 PnP 数据
   * @param [in] p3d_world 参考世界坐标
   * @param [in] p2d_pic 图像点位
   * @param [in] rm_imu 当前姿态
   * @param [out] pnp_info 输出信息
   */
  void SolvePnP(std::array<cv::Point3d, 4> REF_IN p3d_world, std::array<cv::Point2f, 4> REF_IN p2d_pic,
                RMat REF_IN rm_imu, PnPInfo REF_OUT pnp_info);

  /**
   * @brief 将相机坐标系坐标转换为世界坐标系坐标
   * @param [in] ctv_cam 相机坐标系坐标
   * @param [in] rm_imu 当前云台姿态
   * @return 世界坐标系坐标
   */
  CTVec CamToWorld(CTVec REF_IN ctv_cam, RMat REF_IN rm_imu) const;

  /**
   * @brief 将世界坐标系坐标转换为相机坐标系坐标
   * @param [in] ctv_world 世界坐标系坐标
   * @param [in] rm_imu 当前云台姿态
   * @return 相机坐标系坐标
   */
  CTVec WorldToCam(CTVec REF_IN ctv_world, RMat REF_IN rm_imu) const;

  /**
   * @brief 将枪口坐标系坐标转换为世界坐标系坐标
   * @param [in] ctv_muzzle 枪口坐标系坐标
   * @param [in] rm_imu 当前云台姿态
   * @return 世界坐标系坐标
   */
  CTVec MuzzleToWorld(CTVec REF_IN ctv_muzzle, RMat REF_IN rm_imu) const;

  /**
   * @brief 将世界坐标系坐标转换为枪口坐标系坐标
   * @param [in] ctv_world 世界坐标系坐标
   * @param [in] rm_imu 当前云台姿态
   * @return 枪口坐标系坐标
   */
  CTVec WorldToMuzzle(CTVec REF_IN ctv_world, RMat REF_IN rm_imu) const;

  /**
   * @brief 将相机坐标系坐标投影到图像坐标系中
   * @param [in] ctv_cam 相机坐标系坐标
   * @return 图像坐标系点位
   */
  cv::Point2f CamToPic(CTVec REF_IN ctv_cam);

  /**
   * @brief 将图像坐标系的视差转为相机坐标系坐标
   * @warning 仅在双目模式下能使用
   * @param [in] hctv_pic_d 视差，格式为(u,v,d,1)
   * @return CTVec 相机坐标系坐标
   */
  CTVec DisparityToDepth(HCTVec REF_IN hctv_pic_d);
};

}  // namespace srm::coord

#endif  // SRM_COORD_SOLVER_H_