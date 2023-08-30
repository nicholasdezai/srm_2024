#ifndef SRM_COMMON_EKF_H_
#define SRM_COMMON_EKF_H_

#include <ceres/jet.h>
#include <srm/common/tags.h>

#include <Eigen/Geometry>

namespace srm {
/**
 * @brief 利用ceres的自动求导实现的扩展卡尔曼滤波
 * @tparam N_X 状态变量的个数
 * @tparam N_Y 观测变量的个数
 */
template <int N_X, int N_Y>
class AdaptiveEKF {
 public:
  using MatrixXX = Eigen::Matrix<double, N_X, N_X>;
  using MatrixYX = Eigen::Matrix<double, N_Y, N_X>;
  using MatrixXY = Eigen::Matrix<double, N_X, N_Y>;
  using MatrixYY = Eigen::Matrix<double, N_Y, N_Y>;
  using VectorX = Eigen::Matrix<double, N_X, 1>;
  using VectorY = Eigen::Matrix<double, N_Y, 1>;
  explicit AdaptiveEKF(const VectorX &X0 = VectorX::Zero())
      : Xe(X0), P(MatrixXX::Identity()), Q(MatrixXX::Identity()), R(MatrixYY::Identity()) {}
  /**
   * @brief 利用先验进行预测
   * @param [in] func 传入一个结构体，要求重载operator(), 格式为void (const T x0[N_x], T
   * x1[N_X]), 实现内容为：状态x0转移到x1的方程
   * @return 返回预测的值
   */
  template <class Func>
  VectorX predict(Func FWD_IN func) {
    ceres::Jet<double, N_X> Xe_auto_jet[N_X];
    for (int i = 0; i < N_X; i++) {
      Xe_auto_jet[i].a = Xe[i];
      Xe_auto_jet[i].v[i] = 1;
    }
    ceres::Jet<double, N_X> Xp_auto_jet[N_X];
    func(Xe_auto_jet, Xp_auto_jet);
    for (int i = 0; i < N_X; i++) {
      Xp[i] = Xp_auto_jet[i].a;
      F.block(i, 0, 1, N_X) = Xp_auto_jet[i].v.transpose();
    }
    P = F * P * F.transpose() + Q;
    // 如果某阶段只有预测没有估计，自动将预测变为估计
    Xe = Xp;
    return Xp;
  }

  /**
   * @brief 利用后验进行估计
   * @param [in] func 传入一个结构体，要求重载operator(), 格式为void (const T x0[N_x], T
   * z0[N_Y]), 实现内容为：根据x0描述z0的方程
   * @param [in] Y 观测到的量
   * @return VectorX 返回最大似然估计
   */
  template <class Func>
  VectorX update(Func &&func, const VectorY Y) {
    ceres::Jet<double, N_X> Xp_auto_jet[N_X];
    for (int i = 0; i < N_X; i++) {
      Xp_auto_jet[i].a = Xp[i];
      Xp_auto_jet[i].v[i] = 1;
    }
    ceres::Jet<double, N_X> Yp_auto_jet[N_Y];
    func(Xp_auto_jet, Yp_auto_jet);
    for (int i = 0; i < N_Y; i++) {
      Yp[i] = Yp_auto_jet[i].a;
      H.block(i, 0, 1, N_X) = Yp_auto_jet[i].v.transpose();
    }
    K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    Xe = Xp + K * (Y - Yp);
    P = (MatrixXX::Identity() - K * H) * P;
    return Xe;
  }

  VectorX Xe;  ///< [Xe] 估计状态变量
  VectorX Xp;  ///< [Xp] 预测状态变量
  MatrixXX F;  ///< [F] 预测雅克比
  MatrixYX H;  ///< [H] 观测雅克比
  MatrixXX P;  ///< [P] 状态协方差
  MatrixXX Q;  ///< [Q] 预测过程协方差，需要在做预测前指定
  MatrixYY R;  ///< [R] 观测过程协方差, 需要在做预测前指定
  MatrixXY K;  ///< [K] 卡尔曼增益
  VectorY Yp;  ///< [Yp] 预测观测量
};
}  // namespace srm
#endif  // SRM_COMMON_EKF_H_