// SPDX-License-Identifier: ISC
// Copyright (c) 2023, Autonomous Vehicle System Lab, University of Colorado at Boulder
// Copyright (c) 2026, Laboratory for Atmospheric and Space Physics, University of Colorado at Boulder

#ifndef SUNLINESRUKF_H
#define SUNLINESRUKF_H

#include "sunlineSRuKFSpecs.h"

#include <architecture/_GeneralModuleFiles/sys_model.h>
#include <architecture/messaging/messaging.h>
#include <architecture/msgPayloadDef/CSSArraySensorMsgPayload.h>
#include <architecture/msgPayloadDef/CSSConfigMsgPayload.h>
#include <architecture/msgPayloadDef/CSSUnitConfigMsgPayload.h>
#include <architecture/msgPayloadDef/FilterMsgPayload.h>
#include <architecture/msgPayloadDef/FilterResidualsMsgPayload.h>
#include <architecture/msgPayloadDef/NavAttMsgPayload.h>

#include <Eigen/Core>

#include <cstdint>
#include <memory>

namespace filtering::sunlineSRuKF {
class  SunlineSRuKFAlgorithm;
struct SunlineSRuKFOutput;
}

/*! @brief xmera adapter for the sunline SRuKF. Pack CSS and gyro
 *  messages into the algorithm's input types, run update(), and write the output data to messages. */
class SunlineSRuKF : public SysModel {
   public:
    SunlineSRuKF();
    ~SunlineSRuKF() override;

    void reset(uint64_t currentSimNanos) override;
    void updateState(uint64_t currentSimNanos) override;

    void            setAlpha(double newAlpha);
    double          getAlpha() const;
    void            setBeta(double newBeta);
    double          getBeta() const;
    void            setProcessNoise(Eigen::MatrixXd const& newProcessNoise);
    Eigen::MatrixXd getProcessNoise() const;
    void            setInitialState(Eigen::VectorXd const& newInitialState);
    Eigen::VectorXd getInitialState() const;
    void            setInitialCovariance(Eigen::MatrixXd const& newInitialCovariance);
    Eigen::MatrixXd getInitialCovariance() const;
    void            setBiasLowerBound(double lowerBound);
    double          getBiasLowerBound() const;
    void            setBiasUpperBound(double upperBound);
    double          getBiasUpperBound() const;
    void            setCssMeasurementNoiseStd(double noiseStd);
    double          getCssMeasurementNoiseStd() const;
    void            setGyroMeasurementNoiseStd(double noiseStd);
    double          getGyroMeasurementNoiseStd() const;
    void            setSensorThreshold(double threshold);
    double          getSensorThreshold() const;

    ReadFunctor<NavAttMsgPayload>         navAttInMsg;          //!< gyro rate input
    ReadFunctor<CSSArraySensorMsgPayload> cssDataInMsg;         //!< CSS array reading input
    ReadFunctor<CSSConfigMsgPayload>      cssConfigInMsg;       //!< CSS geometry config input (read at reset)

    Message<NavAttMsgPayload>          navAttOutMsg;            //!< sun-pointing vector output
    Message<FilterMsgPayload>          filterOutMsg;            //!< full filter state + covariance output
    Message<FilterResidualsMsgPayload> filterGyroResOutMsg;     //!< gyro residuals output
    Message<FilterResidualsMsgPayload> filterCssResOutMsg;      //!< CSS residuals output

   private:
    void writeOutputMessages(uint64_t currentSimNanos,
                             filtering::sunlineSRuKF::SunlineSRuKFOutput const& filterOutput);

    std::unique_ptr<filtering::sunlineSRuKF::SunlineSRuKFAlgorithm> algorithm;

    double lastNavAttTimeTag = -1;  //!< [s] last NavAtt payload timeTag consumed; -1
    double lastCssTimeTag    = -1;  //!< [s] last CSS payload timeTag consumed; -1
};

#endif
