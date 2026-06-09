// SPDX-License-Identifier: ISC
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef FILTERING_ALGORITHMS_FLYBY_OD_UKF_TYPES_H
#define FILTERING_ALGORITHMS_FLYBY_OD_UKF_TYPES_H

#include <Eigen/Core>

namespace filtering::flybyODuKF {

// Single heading observation (unit vector toward the target body in the
// inertial frame), with timestamp, validity flag, and per-axis observation
// covariance. Marshalled by the host adapter from whatever its native
// optical-nav message looks like.
struct HeadingMeasurement {
    double          timeTag = 0;          // [s]   observation time
    Eigen::Vector3d rhat_BN_N = Eigen::Vector3d::Zero();
                                          // [-]   unit vector body→target in N
    Eigen::Matrix3d covarN    = Eigen::Matrix3d::Identity();
                                          // [-]   observation covariance in N
    bool            valid     = false;
};

// Snapshot of the filter's mean and covariance, valid at `timeTag`.
struct FilterStateOutput {
    double                       timeTag    = 0;
    Eigen::Matrix<double, 6, 1>  state      = Eigen::Matrix<double, 6, 1>::Zero();
    Eigen::Matrix<double, 6, 6>  covariance = Eigen::Matrix<double, 6, 6>::Zero();
};

// Pre-fit and post-fit residuals from the most recent measurement update.
struct ResidualsOutput {
    bool            valid       = false;
    Eigen::Vector3d observation = Eigen::Vector3d::Zero();
    Eigen::Vector3d preFit      = Eigen::Vector3d::Zero();
    Eigen::Vector3d postFit     = Eigen::Vector3d::Zero();
};

}  // namespace filtering::flybyODuKF

#endif
