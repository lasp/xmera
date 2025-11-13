//
// Created by Patrick Kenneally on 5/24/25.
//

#ifndef BASILISK_OBSERVATIONMODEL_H
#define BASILISK_OBSERVATIONMODEL_H

#include "State.hpp"
#include <Eigen/Core>

template <int ObsDim, int StateDim>
class ObservationModel {
   public:
    using ObsVector = Eigen::Matrix<double, ObsDim, 1>;
    using ObsCov = Eigen::Matrix<double, ObsDim, ObsDim>;
    using ObsToState = Eigen::Matrix<double, ObsDim, StateDim>;
    using StateVector = typename State<StateDim>::Vector;

    virtual ObsVector measure(const StateVector& x) const = 0;
    virtual ObsCov noiseCov() const = 0;
    virtual ObsToState jacobian(const StateVector& x) const = 0;
    virtual ~ObservationModel() = default;

    double getTimeTag() const { return this->timeTag; };
    void setTimeTag(double measurementTimeTag) { this->timeTag = measurementTimeTag; };

   private:
    double timeTag{};  //!< [-] Observation time tag
};

#endif  // BASILISK_OBSERVATIONMODEL_H
