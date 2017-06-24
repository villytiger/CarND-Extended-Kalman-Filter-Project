#include "ekf/fusion_ekf.h"

#include "Eigen/Dense"

#include "ekf/tools.h"

using Eigen::Array4d;
using Eigen::Matrix;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;

namespace {

static const auto kPi = acos(-1);

}  // namespace

const Matrix<double, 2, 4>& Helper<LidarMeasurement>::H(
    const Eigen::Vector4d&) const {
  static auto H = []() {
    Matrix<double, 2, 4> H;
    // clang-format off
    H <<
      1, 0, 0, 0,
      0, 1, 0, 0;
    // clang-format on

    return H;
  }();

  return H;
}

const Matrix2d& Helper<LidarMeasurement>::R() const {
  static auto R = []() {
    Matrix2d R;
    // clang-format off
    R <<
      0.0225, 0,
      0, 0.0225;
    // clang-format on

    return R;
  }();

  return R;
}

Vector3d Helper<RadarMeasurement>::y(const Vector4d& state) const {
  const auto px = state(0);
  const auto py = state(1);
  const auto vx = state(2);
  const auto vy = state(3);
  const auto c = sqrt(pow(px, 2) + pow(py, 2));

  Vector3d y = z_ - Vector3d(c, atan2(py, px), (px * vx + py * vy) / c);

  if (y(1) < -kPi)
    y(1) += 2 * kPi;
  else if (y(1) > kPi)
    y(1) -= 2 * kPi;

  return y;
}

Matrix<double, 3, 4> Helper<RadarMeasurement>::H(const Vector4d& state) const {
  const auto px = state(0);
  const auto py = state(1);
  const auto vx = state(2);
  const auto vy = state(3);

  Eigen::Matrix<double, 3, 4> Hj = {};
  if (px == 0 && py == 0) return Hj;

  const auto c1 = pow(px, 2) + pow(py, 2);
  const auto c2 = sqrt(c1);
  const auto c3 = pow(c2, 3);
  const auto c4 = px / c2;
  const auto c5 = py / c2;
  const auto c6 = px * vy - py * vx;

  // clang-format off
  Hj <<
    c4, c5, 0, 0,
    -py / c1, px / c1, 0, 0,
    -py * c6 / c3, px * c6 / c3, px / c2, py / c2;
  // clang-format on

  return Hj;
}

const Matrix3d& Helper<RadarMeasurement>::R() const {
  static auto R = []() {
    Matrix3d R;
    // clang-format off
    R <<
      0.09, 0, 0,
      0, 0.0009, 0,
      0, 0, 0.09;
    // clang-format on

    return R;
  }();

  return R;
}
