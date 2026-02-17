// SPDX-License-Identifier: ISC

#ifndef BASILISK_MEASUREMENT_H
#define BASILISK_MEASUREMENT_H

#include "IMeasurement.hpp"

#include <Eigen/Core>

#include <iostream>

template <int ObsDim, int StateDim, typename Model>
class Measurement final : public IMeasurement<StateDim> {
   public:
    using ObsVector = Eigen::Matrix<double, ObsDim, 1>;

    Measurement(const Model& model, const ObsVector& z) : model(model), z(z) {}

    void apply(State<StateDim>& state) const override {
        using ObsToState = Eigen::Matrix<double, ObsDim, StateDim>;

        Eigen::Matrix<double, ObsDim, StateDim> H = model.jacobian(state.x);
        Eigen::Matrix<double, ObsDim, ObsDim> R = model.noiseCov();
        Eigen::Matrix<double, ObsDim, 1> y = z - model.measure(state.x);
        Eigen::Matrix<double, ObsDim, ObsDim> S = H * state.P * H.transpose() + R;
        Eigen::Matrix<double, StateDim, ObsDim> K = state.P * H.transpose();
        //        Eigen::Matrix<double, ObsDim, ObsDim> thing = S.inverse();
        //        state.x += K * y;
        //        state.P -= K * H * state.P;
    }

    double getTimeTag() const override { return time; }
    bool getValidity() const override { return isValid; }

   private:
    Model model;
    ObsVector z;
    double time;
    bool isValid;
};

#endif  // BASILISK_MEASUREMENT_H
