#ifndef EKF_FUSION_EKF_H_
#define EKF_FUSION_EKF_H_

#include <fstream>

#include "Eigen/Dense"

#include "ekf/kalman_filter.h"
#include "ekf/measurement_package.h"
#include "ekf/tools.h"

template <typename Measurement>
class Helper;

template <>
class Helper<LidarMeasurement> {
 public:
  Helper(const Eigen::Vector2d& z) : z_(z) {}

  Eigen::Vector2d Coords() const { return z_; }

  Eigen::Vector2d y(const Eigen::Vector4d& state) const {
    return z_ - H(state) * state;
  }

  const Eigen::Matrix<double, 2, 4>& H(const Eigen::Vector4d&) const;

  const Eigen::Matrix2d& R() const;

 private:
  Eigen::Vector2d z_;
};

template <>
class Helper<RadarMeasurement> {
 public:
  Helper(const Eigen::Vector3d& z) : z_(z) {}

  Eigen::Vector2d Coords() const {
    const auto rho = z_(0);
    const auto phi = z_(1);
    return {rho * cos(phi), rho * sin(phi)};
  }

  Eigen::Vector3d y(const Eigen::Vector4d& state) const;

  Eigen::Matrix<double, 3, 4> H(const Eigen::Vector4d& state) const;

  const Eigen::Matrix3d& R() const;

 private:
  Eigen::Vector3d z_;
};

class FusionEkf {
 public:
  template <typename Measurement>
  void ProcessMeasurement(const Measurement& measurement) {
    Helper<Measurement> helper(measurement.data);

    if (!is_initialized_) {
      Eigen::Matrix4d P;
      // clang-format off
      P <<
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;
      // clang-format on

      filter_.Init(helper.Coords(), P);

      previous_timestamp_ = measurement.timestamp;
      is_initialized_ = true;

      return;
    }

    filter_.Predict((measurement.timestamp - previous_timestamp_) / 1000000.0);
    filter_.Update(&helper);

    previous_timestamp_ = measurement.timestamp;
  }

  Eigen::Array4d Estimate() const { return filter_.State(); }

 private:
  bool is_initialized_ = false;
  uint_fast64_t previous_timestamp_ = 0;
  KalmanFilter filter_;
};

#endif  // EKF_FUSION_EKF_H_
