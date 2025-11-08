// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

/*! @brief Top level structure for the flyby OD unscented kalman filter.
 Used to estimate the spacecraft's inertial position relative to a body.
 */

#ifndef FLYBYODUKF_H
#define FLYBYODUKF_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/FilterMsgPayload.h>
#include <architecture/msgPayloadDef/FilterResidualsMsgPayload.h>
#include <architecture/msgPayloadDef/NavTransMsgPayload.h>
#include <architecture/msgPayloadDef/OpNavUnitVecMsgPayload.h>
#include <architecture/utilities/macroDefinitions.h>
#include <architecture/utilities/orbitalMotion.h>

#include <fswAlgorithms/_GeneralModuleFiles/kalmanFilter.h>
#include <fswAlgorithms/_GeneralModuleFiles/measurementModels.h>
#include <fswAlgorithms/_GeneralModuleFiles/srukfInterface.h>

struct FlybyODuKFMeasurementModel final : public SRukfMeasurementModel {
public:
    Eigen::MatrixXd model(const FilterStateVector& state) const override {
        return state.getPositionStates() / state.getPositionStates().norm();
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
    Eigen::VectorXd observation = {};  //!< [-] Observation data vector
    Eigen::MatrixXd measNoise = {};    //!< [-] Measurement noise

public:
    Eigen::VectorXd preFitResiduals = {};   //!< [-] Observation pre fit residuals
    Eigen::VectorXd postFitResiduals = {};  //!< [-] Observation post fit residuals
};

class FlybyODuKF : public SysModel {
   public:
    FlybyODuKF() = default;
    ~FlybyODuKF() = default;

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;

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

    void setCentralBodyGravitationParameter(double mu);
    double getCentralBodyGravitationParameter() const;

    void setMeasurementNoiseScale(double measurementNoiseScale);
    double getMeasurementNoiseScale() const;

   private:
    SRukfInterface srukf{};
    xmera::measurement_queue<FlybyODuKFMeasurementModel, 1> measurements = {};
    double muCentral = 1;  //!< [GM] gravitation parameter of central body
    double measNoiseScaling = 1;  //!< [-] Scale factor for the measurement noise
    uint64_t previousSimNanos = 0;
};

#endif
