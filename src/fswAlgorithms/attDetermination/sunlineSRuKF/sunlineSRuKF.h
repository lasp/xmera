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
#include <fswAlgorithms/_GeneralModuleFiles/measurementModels.h>
#include <fswAlgorithms/_GeneralModuleFiles/srukfInterface.h>

class SunlineSRuKF : public SRukfInterface {
   public:
   private:
    void customreset() final;
    void readCssMeasurements();
    void readGyroMeasurements();
    void readFilterMeasurements() final;
    void customFinalizeUpdate() final;
    void writeOutputMessages(uint64_t currentSimNanos) final;
    static FilterStateVector stateDerivative(double t, const FilterStateVector& state);

    int filterMeasurement = 0;    //!< [-] Number of measurements of different types being read
    int numActiveCss = 0;         //!< [-] Number of currently active CSS sensors
    double sensorUseThresh = 0;   //!< Threshold below which we discount sensors
    double cssMeasNoiseStd = 0;   //!< [-] CSS measurement noise std
    double gyroMeasNoiseStd = 0;  //!< [rad/s] rate gyro measurement noise std
    CSSConfigMsgPayload cssConfigInputBuffer;

    double biasLowerBound = 0.5;
    double biasUpperBound = 1.5;

   public:
    ReadFunctor<NavAttMsgPayload> navAttInMsg;
    ReadFunctor<CSSArraySensorMsgPayload> cssDataInMsg;
    ReadFunctor<CSSConfigMsgPayload> cssConfigInMsg;
    Message<NavAttMsgPayload> navAttOutMsg;
    Message<FilterMsgPayload> filterOutMsg;
    Message<FilterResidualsMsgPayload> filterGyroResOutMsg;
    Message<FilterResidualsMsgPayload> filterCssResOutMsg;

    void setCssMeasurementNoiseStd(double cssMeasurementNoiseStd);
    void setGyroMeasurementNoiseStd(double gyroMeasurementNoiseStd);
    double getCssMeasurementNoiseStd() const;
    double getGyroMeasurementNoiseStd() const;
    void setSensorThreshold(double threshold);
    double getSensorThreshold() const;

    void setBiasUpperBound(double biasUpperBound);
    double getBiasUpperBound() const;
    void setBiasLowerBound(double biasLowerBound);
    double getBiasLowerBound() const;
};

#endif
