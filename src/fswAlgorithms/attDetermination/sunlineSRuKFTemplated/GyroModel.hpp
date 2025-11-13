#ifndef BASILISK_GYROMODEL_H
#define BASILISK_GYROMODEL_H

#include "ObservationModel.hpp"

#include <Eigen/Core>

class GyroModel : public ObservationModel<3, 6> {
   public:
    ObsVector measure(const StateVector& x) const override {
        return x.template segment<3>(3);  // Angular velocity
    }

    ObsCov noiseCov() const override { return ObsCov::Identity() * 0.01; }

    ObsToState jacobian(const StateVector& /*x*/) const override {
        ObsToState H = ObsToState::Zero();
        H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
        return H;
    }
};

#endif  // BASILISK_GYROMODEL_H
