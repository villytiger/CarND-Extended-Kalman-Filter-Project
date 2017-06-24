#ifndef EKF_KALMAN_FILTER_H_
#define EKF_KALMAN_FILTER_H_

#include "Eigen/Dense"

class KalmanFilter {
 public:
  KalmanFilter();

  void Init(const Eigen::Vector2d& x, const Eigen::Matrix4d& P);

  void Predict(double delta);

  template <typename Helper>
  void Update(Helper* helper) {
    auto y = helper->y(x_);
    auto H = helper->H(x_);
    auto Ht = H.transpose();
    auto S = H * P_ * Ht + helper->R();
    auto K = P_ * Ht * S.inverse();

    x_ = x_ + (K * y);
    P_ = (Eigen::Matrix4d::Identity() - K * H) * P_;
  }

  const Eigen::Vector4d& State() const;

 private:
  Eigen::Matrix4d F_;
  Eigen::Vector4d x_;
  Eigen::Matrix4d P_;
};

#endif  // EKF_KALMAN_FILTER_H_
