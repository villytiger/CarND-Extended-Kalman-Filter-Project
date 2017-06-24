#include "ekf/kalman_filter.h"

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector2d;
using Eigen::Vector4d;

namespace {

constexpr const int kNoiseAx = 5;
constexpr const int kNoiseAy = 5;

}  // namespace

KalmanFilter::KalmanFilter() {
  // clang-format off
  F_ <<
    1, 0, 1, 0,
    0, 1, 0, 1,
    0, 0, 1, 0,
    0, 0, 0, 1;
  // clang-format on
}

void KalmanFilter::Init(const Vector2d& x, const Matrix4d& P) {
  x_ << x(0), x(1), 0, 0;
  P_ = P;
}

void KalmanFilter::Predict(double delta) {
  F_(0, 2) = F_(1, 3) = delta;
  x_ = F_ * x_;

  double dt2 = delta * delta;
  double dt3 = dt2 * delta;
  double dt4 = dt3 * delta;

  Matrix4d Q;
  // clang-format off
  Q <<
    dt4 * kNoiseAx / 4, 0, dt3 * kNoiseAx / 2, 0,
    0, dt4 * kNoiseAy / 4, 0, dt3 * kNoiseAy / 2,
    dt3 * kNoiseAx / 2, 0, dt2 * kNoiseAx, 0,
    0, dt3 * kNoiseAy / 2, 0, dt2 * kNoiseAy;
  // clang-format on

  P_ = F_ * P_ * F_.transpose() + Q;
}

const Vector4d& KalmanFilter::State() const { return x_; }
