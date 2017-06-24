#ifndef EKF_MEASUREMENT_PACKAGE_H_
#define EKF_MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

struct LidarMeasurement {
  uint_fast64_t timestamp;
  Eigen::Vector2d data;
};

struct RadarMeasurement {
  uint_fast64_t timestamp;
  Eigen::Vector3d data;
};

#endif  // EKF_MEASUREMENT_PACKAGE_H_
