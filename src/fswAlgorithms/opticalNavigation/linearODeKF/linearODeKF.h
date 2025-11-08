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

struct LinearODeKFMeasurementModel : public EkfMeasurementModel {
public:
    //! [-] observation measurement model
    Eigen::MatrixXd model(const FilterStateVector& state) const override {
        return state.getPositionStates() / state.getPositionStates().norm();
    }

    /*! Compute the measurement matrix to linearize the measurement model
        @param FilterStateVector state
        @return Eigen::MatrixXd
    */
    Eigen::MatrixXd measurementMatrix(const FilterStateVector& state) const override {
        Eigen::Vector3d position = state.getPositionStates();

        Eigen::MatrixXd measurementMatrix = Eigen::MatrixXd::Zero(position.size(), state.size());
        measurementMatrix.block(0, 0, position.size(), position.size()) =
            ( 1 / position.norm()
            * ( Eigen::MatrixXd::Identity(position.size(), position.size())
              - 1 / pow(position.norm(), 2) * position * position.transpose()
              )
            );

        return measurementMatrix;
    }

    Eigen::VectorXd subtract(
        const Eigen::VectorXd& observed,
        const Eigen::VectorXd& predicted
    ) const override {
        return observed - predicted;
    }

    Eigen::VectorXd getObservation() const override {
        return this->observation;
    }

    Eigen::MatrixXd getNoise() const override {
        return this->measNoise;
    }

    void setPreFitResiduals(Eigen::VectorXd const& preFitResiduals) override {
        this->preFitResiduals = preFitResiduals;
    }

    void setPostFitResiduals(Eigen::VectorXd const& postFitResiduals) override {
        this->postFitResiduals = postFitResiduals;
    }

public:
    Eigen::VectorXd observation = {};       //!< [-] Observation data vector
    Eigen::MatrixXd measNoise = {};         //!< [-] Measurement noise

public:
    Eigen::VectorXd preFitResiduals = {};   //!< [-] Observation pre fit residuals
    Eigen::VectorXd postFitResiduals = {};  //!< [-] Observation post fit residuals
};

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
    xmera::measurement_queue<LinearODeKFMeasurementModel, 1> measurements = {};  //!< [Measurements] All
    uint64_t previousSimNanos = 0;

    std::optional<Eigen::Vector3d> constantVelocity;         //!< Unestimated constant velocity
    std::optional<Eigen::Vector3d> constantVelocityInitial;  //!< Initial value of constant velocity
};

#endif
