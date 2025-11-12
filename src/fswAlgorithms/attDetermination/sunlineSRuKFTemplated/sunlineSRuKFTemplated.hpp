#ifndef SUNLINESRUKFTEMPLATE_H
#define SUNLINESRUKFTEMPLATE_H

#include "SRUnscentedKalmanFilter.hpp"

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CSSArraySensorMsgPayload.h>
#include <architecture/msgPayloadDef/CSSConfigMsgPayload.h>
#include <architecture/msgPayloadDef/CSSUnitConfigMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>
#include <architecture/msgPayloadDef/FilterMsgPayload.h>
#include <architecture/msgPayloadDef/FilterResidualsMsgPayload.h>
#include <architecture/utilities/macroDefinitions.h>
#include <architecture/utilities/orbitalMotion.h>

#include <Eigen/Core>

class SunlineSRuKFTemplated : public SysModel {
public:
    SunlineSRuKFTemplated();

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;

private:
    SRUnscentedKalmanFilter<6> ukf{};
    void readCssMeasurements();
    void readGyroMeasurements();
    void readFilterMeasurements();
//    void customFinalizeUpdate();
    void writeOutputMessages(uint64_t currentSimNanos);
//    static FilterStateVector<StateConfig> stateDerivative(double t, const FilterStateVector<StateConfig> &state);
//
//    int filterMeasurement = 0;    //!< [-] Number of measurements of different types being read
//    int numActiveCss = 0;         //!< [-] Number of currently active CSS sensors
    double sensorUseThresh = 0;   //!< Threshold below which we discount sensors
    double cssMeasNoiseStd = 0;   //!< [-] CSS measurement noise std
    double gyroMeasNoiseStd = 0;  //!< [rad/s] rate gyro measurement noise std
    CSSConfigMsgPayload cssConfigInputBuffer{};

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

//    State<6> stateDerivative(double t, const State<6> &state);
};

#endif
