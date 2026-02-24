// SPDX-License-Identifier: ISC

// Created by Patrick Kenneally on 5/24/25.
//

#ifndef BASILISK_STATE_H
#define BASILISK_STATE_H

#include <Eigen/Core>

template <int StateDim>
struct State {
    using Vector = Eigen::Matrix<double, StateDim, 1>;
    using Covariance = Eigen::Matrix<double, StateDim, StateDim>;

    Vector x = Vector::Zero();
    Covariance P = Covariance::Identity();
};

#endif  // BASILISK_STATE_H
