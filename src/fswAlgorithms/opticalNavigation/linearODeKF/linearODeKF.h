// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

/*! @brief Top level structure for the flyby OD linear kalman filter.
 Used to estimate the spacecraft's inertial position relative to a body.
 */

#ifndef LINEAR_OD_EKF_H
#define LINEAR_OD_EKF_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/FilterMsgPayload.h>
#include <architecture/msgPayloadDef/FilterResidualsMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>
#include <architecture/msgPayloadDef/OpNavUnitVecMsgPayload.h>
#include <architecture/utilities/macroDefinitions.h>
#include <architecture/utilities/orbitalMotion.h>

#include <fswAlgorithms/_GeneralModuleFiles/ekfInterface.h>
#include <fswAlgorithms/_GeneralModuleFiles/measurementModels.h>

class LinearODeKF : public SysModel {
   public:
    explicit LinearODeKF(FilterType type){};
    ~LinearODeKF() = default;

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;

    void setMeasurementNoiseScale(double measurementNoiseScale);
    double getMeasurementNoiseScale() const;

   private:
    void customReset();
    void readFilterMeasurements();
    void writeOutputMessages(uint64_t currentSimNanos);
    static Eigen::MatrixXd measurementMatrix(const FilterStateVector& state);

   public:
    ReadFunctor<OpNavUnitVecMsgPayload> opNavHeadingMsg;
    OpNavUnitVecMsgPayload opNavHeadingBuffer;
    Message<NavTransMsgPayload> navTransOutMsg;
    Message<FilterMsgPayload> opNavFilterMsg;
    Message<FilterResidualsMsgPayload> opNavResidualMsg;

    void setConstantVelocity(const Eigen::Vector3d& velocity);
    std::optional<Eigen::Vector3d> getConstantVelocity() const;

   private:
    EkfInterface ekf{FilterType::Classical};
    double measNoiseScaling = 1;  //!< [-] Scale factor for the measurement noise
    std::array<std::optional<MeasurementModel>, MAX_MEASUREMENT_NUMBER> measurements;  //!< [Measurements] All
    uint64_t previousSimNanos = 0;

    std::optional<Eigen::Vector3d> constantVelocity;         //!< Unestimated constant velocity
    std::optional<Eigen::Vector3d> constantVelocityInitial;  //!< Initial value of constant velocity
};

#endif
