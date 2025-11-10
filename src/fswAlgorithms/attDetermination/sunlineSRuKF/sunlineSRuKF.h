// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2024, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef SUNLINESRUKF_H
#define SUNLINESRUKF_H

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CSSArraySensorMsgPayload.h>
#include <architecture/msgPayloadDef/CSSConfigMsgPayload.h>
#include <architecture/msgPayloadDef/CSSUnitConfigMsgPayload.h>
#include <architecture/msgPayloadDef/FilterMsgPayload.h>
#include <architecture/msgPayloadDef/FilterResidualsMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <architecture/utilities/macroDefinitions.h>
#include <architecture/utilities/orbitalMotion.h>
#include <fswAlgorithms/_GeneralModuleFiles/srukfInterface.h>

enum class SunlineSRuKFMeasurementType { Gyro, Css };

struct SunlineSRuKFMeasurementModel final : public SRukfMeasurementModel {
public:
    Eigen::MatrixXd model(const FilterStateVector& state) const override {
        switch (this->type) {
        case SunlineSRuKFMeasurementType::Gyro:
            return state.getVelocityStates();

        case SunlineSRuKFMeasurementType::Css: {
            Eigen::VectorXd observed = this->hMatrix * state.getPositionStates();
            if (state.hasBias()) {
                observed = observed * state.getBiasStates().value();
            }
            return observed;
        }
        }
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
    SunlineSRuKFMeasurementType type = SunlineSRuKFMeasurementType::Gyro;
    Eigen::VectorXd observation = {};       //!< [-] Observation data vector
    Eigen::MatrixXd measNoise = {};         //!< [-] Measurement noise
    Eigen::MatrixXd hMatrix = {};

public:
    Eigen::VectorXd preFitResiduals = {};   //!< [-] Observation pre fit residuals
    Eigen::VectorXd postFitResiduals = {};  //!< [-] Observation post fit residuals
};

class SunlineSRuKF : public SysModel {
   public:
    SunlineSRuKF() = default;
    ~SunlineSRuKF() override = default;
    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;

    ReadFunctor<NavAttMsgPayload> navAttInMsg;
    ReadFunctor<CSSArraySensorMsgPayload> cssDataInMsg;
    ReadFunctor<CSSConfigMsgPayload> cssConfigInMsg;
    Message<NavAttMsgPayload> navAttOutMsg;
    Message<FilterMsgPayload> filterOutMsg;
    Message<FilterResidualsMsgPayload> filterGyroResOutMsg;
    Message<FilterResidualsMsgPayload> filterCssResOutMsg;

    void setCssMeasurementNoiseStd(double cssMeasurementNoiseStd);
    void setGyroMeasurementNoiseStd(double gyroMeasurementNoiseStd);
    void setMeasurementNoiseScale(double measurementNoiseScale);
    double getCssMeasurementNoiseStd() const;
    double getGyroMeasurementNoiseStd() const;
    double getMeasurementNoiseScale() const;
    void setSensorThreshold(double threshold);
    double getSensorThreshold() const;

    void setBiasUpperBound(double biasUpperBound);
    double getBiasUpperBound() const;
    void setBiasLowerBound(double biasLowerBound);
    double getBiasLowerBound() const;

   private:
    void customReset();
    void readCssMeasurements();
    void readGyroMeasurements();
    void readFilterMeasurements();
    void customFinalizeUpdate();
    void writeOutputMessages(uint64_t currentSimNanos);
    static FilterStateVector stateDerivative(double t, const FilterStateVector& state);

    SRukfInterface srukf{};
    //! measure up to one star tracker and one gyro
    xmera::measurement_queue<SunlineSRuKFMeasurementModel, 1 + 1> measurements = {};
    int filterMeasurement = 0;    //!< [-] Number of measurements of different types being read
    int numActiveCss = 0;         //!< [-] Number of currently active CSS sensors
    double sensorUseThresh = 0;   //!< Threshold below which we discount sensors
    double cssMeasNoiseStd = 0;   //!< [-] CSS measurement noise std
    double gyroMeasNoiseStd = 0;  //!< [rad/s] rate gyro measurement noise std
    double measNoiseScaling = 1;  //!< [-] Scale factor for the measurement noise
    CSSConfigMsgPayload cssConfigInputBuffer;

    double biasLowerBound = 0.5;
    double biasUpperBound = 1.5;

    uint64_t previousSimNanos = 0;
};

#endif
